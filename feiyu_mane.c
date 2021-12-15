
/*
 * Feiyu gimbal hack
 *
 * Copyright (C) 2016-2018 Adam Williams <broadcast at earthling dot net>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 */



// build with
// make


#include "feiyu_feedback.h"
#include "feiyu_imu.h"
#include "feiyu_mane.h"
#include "feiyu_hall.h"
#include "feiyu_motor.h"
#include "feiyu_uart.h"
#include "arm_linux.h"
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_hal_rcc.h"
#include "arm_math.h"

volatile int mane_time = 0;
feiyu_t fei;

// IMU data from the UART
#ifndef BOARD2


void (*imu_parsing_function)(unsigned char c);
unsigned char imu_buffer[IMU_PACKET_SIZE];
volatile int imu_offset;


void get_imu_sync1(unsigned char c);
void get_imu_sync2(unsigned char c);
void get_imu_data(unsigned char c)
{
	int size = IMU_PACKET_SIZE;

	#ifdef BOARD1
// stuff this board's hall effect sensor
	if(imu_offset == 18)
	{
		c = fei.hall1 & 0xff;
	}
	else
	if(imu_offset == 19)
	{
		c = (fei.hall1 >> 8) & 0xff;
	}
// resend the byte without waiting for the full packet
	send_uart(&uart, &c, 1);
	
	
	#endif // BOARD1


	imu_buffer[imu_offset++] = c;

	if(imu_offset >= size)
	{
		imu_parsing_function = get_imu_sync1;

	
	#ifdef BOARD0
// parse it

//TRACE
//print_buffer(&uart, imu_buffer, size);		
//print_buffer(&uart, imu_buffer, size);		

		fei.hall0 = hall.value;
		fei.hall1 = (imu_buffer[19] << 8) | imu_buffer[18];
		fei.hall2 = (imu_buffer[17] << 8) | imu_buffer[16];

		do_ahrs(imu_buffer);
		do_feedback();

// must write motors to get another readout
		int size = MOTOR_PACKET_SIZE;
		unsigned char buffer[size];
		buffer[0] = 0xff;
		buffer[1] = SYNC_CODE;

		buffer[2] = fei.x_phase & 0xff;
		buffer[3] = (fei.x_phase >> 8) & 0xff;
		buffer[4] = (fei.x_phase >> 16) & 0xff;
		buffer[5] = (fei.x_phase >> 24) & 0xff;

// debug
//		buffer[2] = 0x00;
//		buffer[3] = 0x00;
//		buffer[4] = 0x00;
//		buffer[5] = 0x00;

		buffer[6] = fei.y_phase & 0xff;
		buffer[7] = (fei.y_phase >> 8) & 0xff;
		buffer[8] = (fei.y_phase >> 16) & 0xff;
		buffer[9] = (fei.y_phase >> 24) & 0xff;

// debug
//			buffer[6] = 0x12;
//			buffer[7] = 0x34;
//			buffer[8] = 0x56;
//			buffer[9] = 0x78;

		buffer[10] = motor.power & 0xff;
		buffer[11] = (motor.power >> 8) & 0xff;

		send_uart(&uart2, buffer, size);
		flush_uart(&uart2);

	#endif // BOARD0
	}
}

void get_imu_sync2(unsigned char c)
{
	if(c == SYNC_CODE)
	{
		imu_parsing_function = get_imu_data;
		imu_buffer[1] = SYNC_CODE;
		imu_offset = 2;
#ifdef BOARD1
// forward the byte without waiting for the full packet
		send_uart(&uart, &c, 1);
#endif
	}
}

void get_imu_sync1(unsigned char c)
{
	if(c == 0xff)
	{
		imu_parsing_function = get_imu_sync2;
		imu_buffer[0] = 0xff;
#ifdef BOARD1
// store the hall value for future packing
		fei.hall1 = hall.value;
// forward the byte without waiting for the full packet
		send_uart(&uart, &c, 1);
#endif
	}
	else
	{
// print it for debugging
		send_uart(&uart, &c, 1);
	}
}



#endif // !BOARD2




// motor data
#ifndef BOARD0


unsigned char motor_buffer[MOTOR_PACKET_SIZE];
volatile int motor_offset;
void (*motor_parsing_function)(unsigned char c);
int got_motor = 0;

void get_motor_sync1(unsigned char c);
void get_motor_sync2(unsigned char c);
void get_motor_data(unsigned char c)
{
	motor_buffer[motor_offset++] = c;
	int size = MOTOR_PACKET_SIZE;

	if(motor_offset >= size)
	{
// got the packet
		motor.power = (motor_buffer[11] << 8) |
			motor_buffer[10];

	#ifdef BOARD1
		motor.phase = (motor_buffer[5] << 24) | 
			(motor_buffer[4] << 16) | 
			(motor_buffer[3] << 8) | 
			motor_buffer[2];
		write_motor();
	#endif

	#ifdef BOARD2
		motor.phase = (motor_buffer[9] << 24) | 
			(motor_buffer[8] << 16) | 
			(motor_buffer[7] << 8) | 
			motor_buffer[6];
		write_motor();
//print_hex8(&uart, motor.phase);
//print_buffer(&uart, motor_buffer, MOTOR_PACKET_SIZE);
//print_lf(&uart);
	#endif



// reset
		motor_parsing_function = get_motor_sync1;
// release the mane loop
		got_motor = 1;
	}
}

void get_motor_sync2(unsigned char c)
{
	if(c == SYNC_CODE)
	{
		motor_parsing_function = get_motor_data;
		motor_buffer[1] = SYNC_CODE;
		motor_offset = 2;
	}
}

void get_motor_sync1(unsigned char c)
{
	if(c == 0xff)
	{
		motor_parsing_function = get_motor_sync2;
		motor_buffer[0] = 0xff;
	}
}


#endif // !BOARD0











// 1ms clock
void SysTick_Handler()
{
	mane_time++;
}

uint32_t HAL_GetTick(void)
{
	return mane_time;
}

void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};
  
// 12Mhz crystal -> 72Mhz clock
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
  oscinitstruct.HSEState        = RCC_HSE_ON;
  oscinitstruct.LSEState        = RCC_LSE_OFF;
  oscinitstruct.HSIState        = RCC_HSI_OFF;
  oscinitstruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  oscinitstruct.HSEPredivValue    = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
}


void main()
{
// relocate interrupt vector table
	SCB->VTOR = PROGRAM_START;

	SET_PIN(BLUE_LED_GPIO, 1 << BLUE_LED_PIN)
	SET_PIN(RED_LED_GPIO, 1 << RED_LED_PIN)

	init_linux();
// must reset all the variables even though the bootloader configured the clock
	HAL_Init();
	SystemClock_Config();

#if defined(BOARD1) || defined(BOARD2)
	init_uart(FAST_BAUD, FAST_BAUD);
#else
	init_uart(SLOW_BAUD, FAST_BAUD);
#endif

// re-enable the mane interrupt
	__enable_irq();


#ifdef BOARD0
	print_text(&uart, "Welcome to Feiyu BOARD0\n");
	imu_parsing_function = get_imu_sync1;
	fei.calibrate_imu = 1;
	fei.gyro_x_min = 65535;
	fei.gyro_y_min = 65535;
	fei.gyro_z_min = 65535;
	fei.gyro_x_max = -65535;
	fei.gyro_y_max = -65535;
	fei.gyro_z_max = -65535;
// rotate the IMU rather than this
	fei.target_roll = 0;
	fei.target_pitch = 0;
	fei.target_heading = 0;
	init_feedback();
#endif

#ifdef BOARD1
	print_text(&uart, "Welcome to Feiyu BOARD1\n");
	imu_parsing_function = get_imu_sync1;
	motor_parsing_function = get_motor_sync1;
#endif

#ifdef BOARD2
	print_text(&uart, "Welcome to Feiyu BOARD2\n");
	
/*
 * 	print_text(&uart, "sysclock=");
 * 	print_number(&uart, HAL_RCC_GetSysClockFreq());
 * 	print_text(&uart, "hclock=");
 * 	print_number(&uart, HAL_RCC_GetHCLKFreq());
 * 	print_text(&uart, "pclock1=");
 * 	print_number(&uart, HAL_RCC_GetPCLK1Freq());
 * 	print_text(&uart, "pclock2=");
 * 	print_number(&uart, HAL_RCC_GetPCLK2Freq());
 */
	
	init_imu();
	
	motor_parsing_function = get_motor_sync1;
	
#endif // BOARD2


	init_adc();
	init_hall();
	init_motor();


	int test_time = mane_time;
	fei.test_period = 10;
	fei.test_step = 1;
	fei.test_scale = FRACTION;





#ifdef BOARD2
	while(!imu.initialized)
	{
		handle_imu();
	}

	while(1)
	{
		do_imu();
		flush_uart(&uart);

// can't simultaneously service the UART & drive I2C
		got_motor = 0;
		int motor_timeout = mane_time;
		while(!got_motor && 
			mane_time - motor_timeout < HZ / 10)
		{
			handle_hall();
			if(uart_got_input(&uart))
			{
				unsigned char c = uart_get_input(&uart);
				motor_parsing_function(c);
			}
		}
	}
#else // BOARD2






	while(1)
	{
// if(mane_time - debug_time >= HZ)
// {
// debug_time = mane_time;
// TRACE
// //print_fixed(&uart, top_z_step);
// //print_number(&uart, fei.hall0);
// //print_number(&uart, fei.hall1);
// print_number(&uart, uart2.total_in);
// print_number(&uart, uart2.input_size);
// }



		handle_hall();
		handle_adc();

		handle_uart();

// IMU data
		if(uart_got_input(&uart2))
		{
			unsigned char c = uart_get_input(&uart2);
			imu_parsing_function(c);
		}

// motor command
#ifdef BOARD1
		if(uart_got_input(&uart))
		{
			unsigned char c = uart_get_input(&uart);
// resend it
			send_uart(&uart2, &c, 1);

			motor_parsing_function(c);
		}
#endif // BOARD1





#ifndef CALIBRATE_MOTOR
//		if(mane_time - test_time > fei.test_period)
//		{
//			TRACE
//			print_number(&uart, hall.value);
//			print_number(&uart, motor.phase / FRACTION);

//			test_time = mane_time;
//			motor.phase += fei.test_step * FRACTION;
//			motor.hall += fei.test_step;
//			write_motor();
//		}
#endif 

#ifdef CALIBRATE_MOTOR
		motor_test();
#endif



#ifdef BOARD0
// user command
		int print_pid = 0;
		int print_phase = 0;
		int print_scale = 0;
		if(uart_got_input(&uart))
		{
			unsigned char c = uart_get_input(&uart);
			switch(c)
			{
				case '2':
				{
//					if(fei.yaw_roll_fraction < FRACTION / 4)
					{
						fei.target_heading -= FRACTION;
					}
// 					else
// 					{
// 						fei.target_heading -= FRACTION / 4;
// 					}
					
					if(fei.target_heading < -180 * FRACTION)
					{
						fei.target_heading += 360 * FRACTION;
					}
				}
				break;
			
				case '1':
				{
//					if(fei.yaw_roll_fraction < FRACTION / 4)
					{
						fei.target_heading += FRACTION;
					}
// 					else
// 					{
// 						fei.target_heading += FRACTION / 4;
// 					}
					
					if(fei.target_heading > 180 * FRACTION)
					{
						fei.target_heading -= 360 * FRACTION;
					}
				}
				break;

#ifdef TEST_MOTOR
				case '3':
				{
					fei.x_phase -= FRACTION;
					motor.phase = fei.z_phase;
					write_motor();
					FIX_PHASE(fei.x_phase);
					TRACE
					print_number(&uart, fei.hall1);
					print_fixed(&uart, fei.x_phase);
				}
				break;
			
				case '4':
				{
					fei.x_phase += FRACTION;
					motor.phase = fei.z_phase;
					write_motor();
					FIX_PHASE(fei.x_phase);
					TRACE
					print_number(&uart, fei.hall1);
					print_fixed(&uart, fei.x_phase);
				}
				break;
				
				
#endif // TEST_MOTOR


#ifdef TEST_KINEMATICS
				case 'w':
					fei.x_phase += FRACTION;
//					print_phase = 1;
					break;
				case 'z':
					fei.x_phase -= FRACTION;
//					print_phase = 1;
					break;
				case 'u':
					fei.z_phase += FRACTION;
					print_phase = 1;
					break;
				case 'n':
					fei.z_phase -= FRACTION;
					print_phase = 1;
					break;
				case 't':
					fei.x_phase += FRACTION;
					fei.z_phase -= FRACTION;
					print_phase = 1;
					break;
				case 'v':
					fei.x_phase -= FRACTION;
					fei.z_phase += FRACTION;
					print_phase = 1;
					break;
#else


				case 'w':
				{
					fei.roll_ipd.p += 1;
					print_pid = 1;

	//				fei.test_step++;
	//				fei.test_period++;
	//				motor.phase += 1 * FRACTION;
	//				write_motor();
	//				motor.hall += 10;
	//				TRACE
	//				print_text(&uart, "phase=");
	//				print_number(&uart, motor.phase / FRACTION);
	//				print_number(&uart, motor.hall);
	//				motor.pwm2 += 10;
	//				write_motor2();
				}
				break;
				
			
				case 'z':
				{
					fei.roll_ipd.p -= 1;
					print_pid = 1;

	//				fei.test_step--;
	//				fei.test_period--;
	//				motor.phase -= 1 * FRACTION;
	//				write_motor();
	//				motor.hall -= 10;
	//				TRACE
	//				print_text(&uart, "phase=");
	//				print_number(&uart, motor.phase / FRACTION);
	//				print_number(&uart, motor.hall);
	//				motor.pwm2 -= 10;
	//				write_motor2();
				}
				break;
			
				case 'u':
				{
					fei.roll_ipd.d += 1;
					print_pid = 1;


	//				motor.pwm1 += 10;
	//				write_motor2();
	//				TRACE
	//				print_text(&uart, "pwm1=");
	//				print_number(&uart, motor.pwm1);
	//				if(motor.power < MAX_POWER)
	//				{
	//					motor.power += 1;
	//				}
	//				print_text(&uart, "power=");
	//				print_number(&uart, motor.power);
	//				write_motor();
				}
				break;
			
				case 'n':
				{
					fei.roll_ipd.d -= 1;
					print_pid = 1;


	//				motor.pwm1 -= 10;
	//				write_motor2();
	//				TRACE
	//				print_text(&uart, "pwm1=");
	//				print_number(&uart, motor.pwm1);
	//				if(motor.power > 0)
	//				{
	//					motor.power -= 1;
	//				}
	//				print_text(&uart, "power=");
	//				print_number(&uart, motor.power);
	//				write_motor();
				}
				break;


#endif // !TEST_KINEMATICS

			}
			
			if(print_pid)
			{
				TRACE
				print_fixed(&uart, fei.roll_ipd.p);
				print_fixed(&uart, fei.roll_ipd.d);
			}
			
			if(print_phase)
			{
				TRACE
				print_text(&uart, "X=");
				print_fixed(&uart, fei.x_phase);
				print_text(&uart, "Y=");
				print_fixed(&uart, fei.y_phase);
				print_text(&uart, "Z=");
				print_fixed(&uart, fei.z_phase);
				print_text(&uart, "current_pitch2=");
				print_fixed(&uart, fei.current_pitch2);
			}
			
			if(print_scale)
			{
				TRACE
				print_fixed(&uart, fei.test_scale);
			}
			
			
//			TRACE
//			print_fixed(&uart, fei.target_heading);
			
		}
#endif // BOARD0
	}
#endif // !BOARD2
}












