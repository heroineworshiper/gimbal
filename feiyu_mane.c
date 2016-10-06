// bootloader for the feiyu stm32f103c8 
// STM32Cube_FW_F1_V1.3.0 only defines the stm32f103xb

// build with
// make



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
volatile int debug_time = 0;
feiyu_t fei;

// IMU data from the UART
#ifndef BOARD2

static void do_feedback();

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


		send_uart(&uart2, buffer, size);


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










#ifdef BOARD0


static void init_ipd(ipd_t *ptr, 
	int i, 
	int p, 
	int d, 
	int error_limit, 
	int rate_limit)
{
	ptr->i = i;
	ptr->p = p;
	ptr->d = d;
	ptr->error_limit = error_limit;
	ptr->rate_limit = rate_limit;
}


static int get_step(ipd_t *table, int error, int rate, int derivative)
{
//	int limit = 0x7fffffff / table->i;
//	CLAMP(error, -limit, limit);

	int i_result = error * table->i / FRACTION;
	CLAMP(i_result, -table->error_limit, table->error_limit);

//	limit = 0x7fffffff / table->p;
//	CLAMP(rate, -limit, limit);

	int p_result = rate * table->p / FRACTION;
	CLAMP(p_result, -table->rate_limit, table->rate_limit);
	
//	limit = 0x7fffffff / table->d;
//	CLAMP(derivative, -limit, limit);

	int d_result = derivative * table->d / FRACTION;
	CLAMP(d_result, -table->rate_limit, table->rate_limit);

	int result = i_result + p_result + d_result;
	CLAMP(result, -table->rate_limit, table->rate_limit);
	return result;
}




// motor feedback is on board 0 only
void init_feedback()
{
	init_derivative(&fei.roll_accel, ROLL_D_SIZE);
	init_derivative(&fei.pitch_accel, PITCH_D_SIZE);
	init_derivative(&fei.heading_accel, HEADING_D_SIZE);
	
	
	
// effect of pitch motor on pitch
	init_ipd(&fei.top_y,  
		1 * FRACTION,   // I
		FRACTION / 20,   // P
		FRACTION / 4,   // D
		255 * FRACTION,   // error limit
		255 * FRACTION);  // rate limit
	
	
// effect of roll motor on roll
	init_ipd(&fei.top_x,  
		1 * FRACTION, 
		FRACTION / 10, 
		FRACTION / 2, 
		255 * FRACTION, 
		255 * FRACTION);
// effect of yaw motor on yaw
	init_ipd(&fei.top_z, 
		1 * FRACTION,  // I
		FRACTION / 10,  // P
		FRACTION / 2,  // D
		255 * FRACTION,  // error limit
		255 * FRACTION); // rate limit
	
	
	
// effect of roll motor on yaw
	init_ipd(&fei.back_x,  
		1 * FRACTION, 
		FRACTION / 10, 
		FRACTION / 2, 
		1 * FRACTION, 
		1 * FRACTION);
// effect of yaw motor on roll
	init_ipd(&fei.back_z, 
		1 * FRACTION,  // I
		FRACTION / 10,  // P
		FRACTION / 2,  // D
		255 * FRACTION,  // error limit
		255 * FRACTION); // rate limit


	init_ipd(&fei.top_y2, 
		10 * FRACTION,  // I
		1 * FRACTION,  // P
		0,  // D
		60 * FRACTION,  // error limit
		60 * FRACTION); // rate limit
	init_ipd(&fei.top_x2, 
		10 * FRACTION,  // I
		1 * FRACTION,  // P
		0,  // D
		60 * FRACTION,  // error limit
		60 * FRACTION); // rate limit
	init_ipd(&fei.top_z2, 
		10 * FRACTION,  // I
		1 * FRACTION,  // P
		0,  // D
		60 * FRACTION,  // error limit
		60 * FRACTION); // rate limit
}


static void do_feedback()
{
#ifndef TEST_MOTOR
	if(!fei.calibrate_imu)
	{

// angle errors at the camera
		int x_error = get_angle_change_fixed(fei.current_roll, fei.target_roll);
		int y_error = get_angle_change_fixed(fei.current_pitch, fei.target_pitch);
		int z_error = get_angle_change_fixed(fei.current_heading, fei.target_heading);

// angle rates of rates
		update_derivative(&fei.roll_accel, (fei.gyro_x * FRACTION - fei.gyro_x_center));
		update_derivative(&fei.pitch_accel, (fei.gyro_y * FRACTION - fei.gyro_y_center));
		update_derivative(&fei.heading_accel, (fei.gyro_z * FRACTION - fei.gyro_z_center));


// effect of pitch motor on pitch
// use hall sensor feedback
		if(abs_fixed(y_error) > 5 * FRACTION)
		{
// hall  phase
// 23860 -360
// 26191 0
// 28507 360
// predicted phase based on hall effect sensor
			int top_y_phase2 = ((fei.hall2 - 26191) % 2323) * 
				360 * FRACTION / 
				2323;
// required change in phase
			int top_y_step2 = get_step(&fei.top_y2, 
				y_error, 
				-(fei.gyro_y * FRACTION - fei.gyro_y_center) / FRACTION, 
				0);
			fei.y_phase = top_y_phase2 + top_y_step2;
		}
		else
// use gyro feedback
		{
			int top_y_step = get_step(&fei.top_y, 
				-y_error, 
				(fei.gyro_y * FRACTION - fei.gyro_y_center) / FRACTION, 
				get_derivative(&fei.pitch_accel) / FRACTION);
			fei.y_phase -= top_y_step;
		}


		FIX_PHASE(fei.y_phase);


// ranges for the hall sensors
// use top values
#define PITCH_VERTICAL 25100
// use back values
#define PITCH_UP 29255
// not implemented
#define PITCH_DOWN 21000

#define ROLL_VERTICAL 21200
#define ROLL_MIN 19000
#define ROLL_MAX 23100

// absolute pitch in degrees
		int current_pitch2 = (fei.hall2 - PITCH_VERTICAL) * 90 * FRACTION / 
			(PITCH_UP - PITCH_VERTICAL);
		CLAMP(current_pitch2, 0, 90 * FRACTION);
		

// Amount yaw motor contributes to roll
		int yaw_roll_fraction = 1 * FRACTION - (cos_fixed(current_pitch2 * 2) + 1 * FRACTION) / 2;
//		int yaw_roll_fraction = (fei.hall2 - PITCH_VERTICAL) * FRACTION / 
//			(PITCH_UP - PITCH_VERTICAL);

// Amount yaw motor contributes to pitch
//		int yaw_pitch_fraction = 1 * FRACTION - (cos_fixed(imu2.current_roll * 2) + 1 * FRACTION) / 2;


// if either Z or X motor is off, use hall sensor feedback for both
		if(abs_fixed(x_error) > 5 * FRACTION ||
			abs_fixed(z_error) > 5 * FRACTION)
		{
// Z hall phase
// 5875 -360
// 8218 0
// 10548 360
			int top_z_phase2 = ((fei.hall0 - 8218) % 2336) * 
				360 * FRACTION / 
				2336;
			int top_z_step2 = get_step(&fei.top_y2, 
				-z_error, 
				(fei.gyro_z * FRACTION - fei.gyro_z_center) / FRACTION, 
				0);
			fei.z_phase = top_z_phase2 + top_z_step2;

		
// X hall phase
// 19925 0
// 22309 360
			int top_x_phase2 = ((fei.hall1 - 19925) % 2384) * 
				360 * FRACTION / 
				2384;
			int top_x_step2 = get_step(&fei.top_x2, 
				-x_error, 
				(fei.gyro_x * FRACTION - fei.gyro_x_center) / FRACTION, 
				0);
			fei.x_phase = top_x_phase2 + top_x_step2;
		}
		else
		{
// use IMU feedback
	// get motor steps for heading motor on top
	// effect of roll motor on roll
			int top_x_step = get_step(&fei.top_x, 
				x_error, 
				-(fei.gyro_x * FRACTION - fei.gyro_x_center) / FRACTION, 
				-get_derivative(&fei.roll_accel) / FRACTION);
	// effect of yaw motor on yaw
			int top_z_step = get_step(&fei.top_z, 
				-z_error, 
				(fei.gyro_z * FRACTION - fei.gyro_z_center) / FRACTION, 
				get_derivative(&fei.heading_accel) / FRACTION);


	// get motor steps if heading motor behind camera
	// effect of roll motor on yaw
			int back_x_step = get_step(&fei.back_x, 
				z_error, 
				-(fei.gyro_z * FRACTION - fei.gyro_z_center) / FRACTION, 
				-get_derivative(&fei.heading_accel) / FRACTION);
	// effect of yaw motor on roll
			int back_z_step = get_step(&fei.back_z, 
				-x_error, 
				(fei.gyro_x * FRACTION - fei.gyro_x_center) / FRACTION, 
				get_derivative(&fei.roll_accel) / FRACTION);

			int total_x_step = ((FRACTION - yaw_roll_fraction) * top_x_step +
				yaw_roll_fraction * back_x_step) / FRACTION;
			int total_z_step = ((FRACTION - yaw_roll_fraction) * top_z_step +
				yaw_roll_fraction * back_z_step) / FRACTION;

	// scale feedback as it approaches 45'.  failed
	//		int feedback_scale = abs_fixed(yaw_roll_fraction - FRACTION / 2) +
	//			FRACTION / 2;
	//		total_x_step = total_x_step * feedback_scale / FRACTION;
	//		total_z_step = total_z_step * feedback_scale / FRACTION;

	// fade yaw motor as it rolls.
	//		total_z_step = total_z_step * (FRACTION - yaw_pitch_fraction) / FRACTION;

			fei.x_phase += total_x_step;
			FIX_PHASE(fei.x_phase);
			fei.z_phase += total_z_step;
			FIX_PHASE(fei.z_phase);
		}



if(mane_time - debug_time >= HZ / 10)
{
debug_time = mane_time;
//TRACE
//print_fixed(&uart, feedback_scale);
//print_number(&uart, fei.hall2);
//print_fixed(&uart, test_phase);
//print_fixed(&uart, fei.y_phase);
//print_fixed(&uart, y_error);
// print_fixed(&uart, yaw_roll_fraction);
// //print_number(&uart, fei.hall0);
// //print_number(&uart, fei.hall1); // roll motor
// //print_number(&uart, fei.hall2); // pitch motor
}

// TODO: calculate power by size of step
// write the yaw motor directly from board 0
		motor.phase = fei.z_phase;
		write_motor();
		
		
	}
#endif // TEST_MOTOR
}



#endif // BOARD0




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
	fei.target_roll = -5 * FRACTION;
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





#ifdef BOARD2
	while(!imu.initialized)
	{
		handle_imu();
	}

	while(1)
	{
		do_imu();

// can't simultaneously service the UART & drive I2C
		got_motor = 0;
		int motor_timeout = mane_time;
		while(uart_got_output(&uart) ||
			(!got_motor && 
			mane_time - motor_timeout < HZ / 10))
		{
			handle_uart();
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
		if(uart_got_input(&uart))
		{
			unsigned char c = uart_get_input(&uart);
			switch(c)
			{
				case '1':
				{
					fei.target_heading -= FRACTION;
					if(fei.target_heading < -180 * FRACTION)
					{
						fei.target_heading += 360 * FRACTION;
					}
				}
				break;
			
				case '2':
				{
					fei.target_heading += FRACTION;
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



				case 'w':
				{
					fei.top_y.p += 1;


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
					fei.top_y.p -= 1;

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
					fei.top_y.d += 1;


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
					fei.top_y.d -= 1;


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
			}
			
//			TRACE
//			print_fixed(&uart, fei.target_heading);
			
		}
#endif // BOARD0
	}
#endif // !BOARD2
}












