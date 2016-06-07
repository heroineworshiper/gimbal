// bootloader for the feiyu stm32f103c8 
// STM32Cube_FW_F1_V1.3.0 only defines the stm32f103xb

// build with
// make


// program with 
// feiyu_program feiyu_mane.bin 
// feiyu_program -p 1 feiyu_mane.bin 

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

// IMU data
#ifndef BOARD2


void (*imu_parsing_function)(unsigned char c);
unsigned char imu_buffer[32];
int imu_offset;


void get_imu_sync1(unsigned char c);
void get_imu_sync2(unsigned char c);
void get_imu_data(unsigned char c)
{
	imu_buffer[imu_offset++] = c;
	int size = 20;

	if(imu_offset >= size)
	{
		imu_parsing_function = get_imu_sync1;
#ifdef BOARD1
// append this board's hall effect sensor & resend
		imu_buffer[18] = hall.value & 0xff;
		imu_buffer[19] = (hall.value >> 8) & 0xff;
		send_uart(&uart, imu_buffer, size);


#else // BOARD1
// parse it

//TRACE
//print_buffer(&uart, imu_buffer, size);		
//print_buffer(&uart, imu_buffer, size);		

		fei.hall0 = hall.value;
		fei.hall1 = (imu_buffer[19] << 8) | imu_buffer[18];
		fei.hall2 = (imu_buffer[17] << 8) | imu_buffer[16];

		do_ahrs(imu_buffer);


#endif // !BOARD1
	}
}

void get_imu_sync2(unsigned char c)
{
	if(c == SYNC_CODE)
	{
		imu_parsing_function = get_imu_data;
		imu_buffer[1] = SYNC_CODE;
		imu_offset = 2;
	}
}

void get_imu_sync1(unsigned char c)
{
	if(c == 0xff)
	{
		imu_parsing_function = get_imu_sync2;
		imu_buffer[0] = 0xff;
	}
}



#endif // !BOARD2




// motor data
#ifndef BOARD0


unsigned char motor_buffer[32];
int motor_offset;
void (*motor_parsing_function)(unsigned char c);

void get_motor_sync1(unsigned char c);
void get_motor_sync2(unsigned char c);
void get_motor_data(unsigned char c)
{
	motor_buffer[motor_offset++] = c;
	int size = 8;

	if(motor_offset >= size)
	{
		motor_parsing_function = get_motor_sync1;
#ifndef BOARD2
// resend it
		send_uart(&uart2, motor_buffer, size);
#endif // !BOARD1
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
	init_uart();

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
	while(1)
	{
		handle_uart();
#ifdef BOARD2
// DEBUG
//		handle_imu();
#endif

		handle_hall();
		handle_adc();


// IMU data
#ifndef BOARD2
		if(uart_got_input(&uart2))
		{
			unsigned char c = uart_get_input(&uart2);
			imu_parsing_function(c);
			
		}
#endif // !BOARD2

// motor command
#ifndef BOARD0
// DEBUG
// 		if(uart_got_input(&uart))
// 		{
// 			unsigned char c = uart_get_input(&uart);
// 			motor_parsing_function(c);
// 		}
#endif // !BOARD0

// user command
#ifdef BOARD0
		if(uart_got_input(&uart))
		{
			unsigned char c = uart_get_input(&uart);
			
		}
#endif // BOARD0


// DEBUG
		if(mane_time - test_time > fei.test_period)
		{
			TRACE
			print_number(&uart, hall.value);
			print_number(&uart, motor.phase / FRACTION);

			test_time = mane_time;
			motor.phase += fei.test_step * FRACTION;
			write_motor();
		}


// DEBUG
		if(uart_got_input(&uart))
		{
			char c = uart_get_input(&uart);
			if(c == 'w')
			{
//				fei.test_step++;
//				fei.test_period++;
				motor.phase += 1 * FRACTION;
				TRACE
				print_number(&uart, motor.phase / FRACTION);
				write_motor();
			}
			else
			if(c == 'z')
			{
//				fei.test_step--;
//				fei.test_period--;
				motor.phase -= 1 * FRACTION;
				TRACE
				print_number(&uart, motor.phase / FRACTION);
				write_motor();
			}
			else
			if(c == 'u')
			{
				if(motor.power < MAX_POWER)
				{
					motor.power += 16;
				}
				TRACE
				print_number(&uart, motor.power);
				write_motor();
			}
			else
			if(c == 'n')
			{
				if(motor.power > 0)
				{
					motor.power -= 16;
				}
				TRACE
				print_number(&uart, motor.power);
				write_motor();
			}
		}
	}
}












