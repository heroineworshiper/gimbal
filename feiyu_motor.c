#include "stm32f1xx_hal_dma.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_hal_tim.h"

#include "feiyu_mane.h"
#include "feiyu_adc.h"
#include "feiyu_motor.h"
#include "feiyu_uart.h"
#include "arm_math.h"
#include "arm_linux.h"

motor_t motor;

TIM_HandleTypeDef TimHandle;
TIM_OC_InitTypeDef sConfig;

#define N_MOSFET_PIN1 13
#define N_MOSFET_PIN2 14
#define N_MOSFET_PIN3 15
#define N_MOSFET_GPIO GPIOB

#define P_MOSFET_PIN1 8
#define P_MOSFET_PIN2 9
#define P_MOSFET_PIN3 10
#define P_MOSFET_GPIO GPIOA

#define PERIOD 3600
#define MIN_DEADBAND 196
// PERIOD - PWM cutoff value
#define MAX_SIN PERIOD
// PWM cutoff value
#define MIN_SIN 1

const uint16_t sin_table[] = {
	0x7fff, 0x8323, 0x8647, 0x896a, 0x8c8b, 0x8faa, 0x92c7, 0x95e1, 0x98f8, 0x9c0a, 0x9f19, 0xa223, 0xa527, 0xa826, 0xab1e, 0xae10, 
	0xb0fb, 0xb3de, 0xb6b9, 0xb98c, 0xbc55, 0xbf16, 0xc1cd, 0xc47a, 0xc71c, 0xc9b3, 0xcc3f, 0xcebf, 0xd132, 0xd39a, 0xd5f4, 0xd842, 
	0xda81, 0xdcb3, 0xded6, 0xe0eb, 0xe2f1, 0xe4e7, 0xe6ce, 0xe8a5, 0xea6c, 0xec23, 0xedc9, 0xef5e, 0xf0e1, 0xf254, 0xf3b4, 0xf503, 
	0xf640, 0xf76b, 0xf883, 0xf989, 0xfa7c, 0xfb5c, 0xfc28, 0xfce2, 0xfd89, 0xfe1c, 0xfe9c, 0xff08, 0xff61, 0xffa6, 0xffd7, 0xfff5, 
	0xffff, 0xfff5, 0xffd7, 0xffa6, 0xff61, 0xff08, 0xfe9c, 0xfe1c, 0xfd89, 0xfce2, 0xfc28, 0xfb5c, 0xfa7c, 0xf989, 0xf883, 0xf76b, 
	0xf640, 0xf503, 0xf3b4, 0xf254, 0xf0e1, 0xef5e, 0xedc9, 0xec23, 0xea6c, 0xe8a5, 0xe6ce, 0xe4e7, 0xe2f1, 0xe0eb, 0xded6, 0xdcb3, 
	0xda81, 0xd842, 0xd5f4, 0xd39a, 0xd132, 0xcebf, 0xcc3f, 0xc9b3, 0xc71c, 0xc47a, 0xc1cd, 0xbf16, 0xbc55, 0xb98c, 0xb6b9, 0xb3de, 
	0xb0fb, 0xae10, 0xab1e, 0xa826, 0xa527, 0xa223, 0x9f19, 0x9c0a, 0x98f8, 0x95e1, 0x92c7, 0x8faa, 0x8c8b, 0x896a, 0x8647, 0x8323, 
	0x7fff, 0x7cdb, 0x79b7, 0x7694, 0x7373, 0x7054, 0x6d37, 0x6a1d, 0x6706, 0x63f4, 0x60e5, 0x5ddb, 0x5ad7, 0x57d8, 0x54e0, 0x51ee, 
	0x4f03, 0x4c20, 0x4945, 0x4672, 0x43a9, 0x40e8, 0x3e31, 0x3b84, 0x38e2, 0x364b, 0x33bf, 0x313f, 0x2ecc, 0x2c64, 0x2a0a, 0x27bc, 
	0x257d, 0x234b, 0x2128, 0x1f13, 0x1d0d, 0x1b17, 0x1930, 0x1759, 0x1592, 0x13db, 0x1235, 0x10a0, 0x0f1d, 0x0daa, 0x0c4a, 0x0afb, 
	0x09be, 0x0893, 0x077b, 0x0675, 0x0582, 0x04a2, 0x03d6, 0x031c, 0x0275, 0x01e2, 0x0162, 0x00f6, 0x009d, 0x0058, 0x0027, 0x0009, 
	0x0000, 0x0009, 0x0027, 0x0058, 0x009d, 0x00f6, 0x0162, 0x01e2, 0x0275, 0x031c, 0x03d6, 0x04a2, 0x0582, 0x0675, 0x077b, 0x0893, 
	0x09be, 0x0afb, 0x0c4a, 0x0daa, 0x0f1d, 0x10a0, 0x1235, 0x13db, 0x1592, 0x1759, 0x1930, 0x1b17, 0x1d0d, 0x1f13, 0x2128, 0x234b, 
	0x257d, 0x27bc, 0x2a0a, 0x2c64, 0x2ecc, 0x313f, 0x33bf, 0x364b, 0x38e2, 0x3b84, 0x3e31, 0x40e8, 0x43a9, 0x4672, 0x4945, 0x4c20, 
	0x4f03, 0x51ee, 0x54e0, 0x57d8, 0x5ad7, 0x5ddb, 0x60e5, 0x63f4, 0x6706, 0x6a1d, 0x6d37, 0x7054, 0x7373, 0x7694, 0x79b7, 0x7cdb
};

//const uint16_t sin_table[] = { 0xffff, 0xffff, 0xffff, 0x0000, 0x0000, 0x0000 };

// need a different table for each motor, since the voltage reaching the motor
// is lower than the voltage reaching the ADC, depending on how many motors it's 
// passing to.  Try getting current sensors to work.

// deadband vs. battery for 2.4W, BOARD2
const uint16_t deadband_table[] = 
{
	1025, 217, // 6V 0.4A
	1200, 233, // 7V 0.34A
	1450, 243, // 8V 0.3A
	1666, 250, // 9V 0.27A
	1800, 255, // 10V, 0.24A
};

#define WAVEFORM_SIZE (sizeof(sin_table) / sizeof(uint16_t))
#define DEADBAND_ENTRIES (sizeof(deadband_table) / sizeof(uint16_t) / 2)


void print_phase(int value)
{
	int scaled = value * 79 / PERIOD;
	int i;
	for(i = 0; i < scaled; i++)
	{
		send_uart(&uart, "#", 1);
	}
	for(i = scaled; i < 79; i++)
	{
		send_uart(&uart, " ", 1);
	}
	print_lf(&uart);
}

void print_phases()
{
//	const char *clear_screen = "\033[2J";
//	send_uart(&uart, clear_screen, strlen(clear_screen));
	TRACE
	print_number(&uart, fei.test_period);
	send_uart(&uart, "     ", 5);
	print_lf(&uart);
	print_phase(TimHandle.Instance->CCR1);
	print_phase(TimHandle.Instance->CCR2);
	print_phase(TimHandle.Instance->CCR3);


	const char *cursor_up = "\033[4A";
	send_uart(&uart, cursor_up, strlen(cursor_up));
	flush_uart(&uart);
}


#define FIX_PHASE(x) \
	while(x < 0) x += 360 * FRACTION; \
	while(x >= 360 * FRACTION) x -= 360 * FRACTION;
#define CALCULATE_WAVEFORM(x) ((uint32_t)sin_table[x] * (max_sin - min_sin) / 65535 + min_sin)
//#define CALCULATE_WAVEFORM(x) ((uint32_t)sin_table[x] * (PERIOD) / 65535)

// set the PWM values for the phase
void write_motor()
{
	FIX_PHASE(motor.phase)
	
	int max_sin = MAX_SIN - (MAX_POWER - motor.power);
	int min_sin = MIN_SIN + (MAX_POWER - motor.power);
	int index1 = (motor.phase * WAVEFORM_SIZE / 360 / FRACTION) % WAVEFORM_SIZE;
	int index2 = (index1 + 120 * WAVEFORM_SIZE / 360) % WAVEFORM_SIZE;
	int index3 = (index1 + 240 * WAVEFORM_SIZE / 360) % WAVEFORM_SIZE;
//TRACE
//print_number(&uart, max_sin);
//print_number(&uart, min_sin);
	
	TimHandle.Instance->CCR1 = CALCULATE_WAVEFORM(index1);
	TimHandle.Instance->CCR2 = CALCULATE_WAVEFORM(index2);
	TimHandle.Instance->CCR3 = CALCULATE_WAVEFORM(index3);

//TRACE
//print_number(&uart, index1);
//print_number(&uart, CALCULATE_WAVEFORM(index1));
//print_number(&uart, CALCULATE_WAVEFORM(index2));
//print_number(&uart, CALCULATE_WAVEFORM(index3));
//	print_phases();
}

void update_deadband()
{
	if(adc.battery > 0)
	{
		int i;
		int new_deadband = 0;
		if(adc.battery_avg <= deadband_table[0])
		{
			new_deadband = deadband_table[1];
		}
		else
		if(adc.battery_avg >= deadband_table[DEADBAND_ENTRIES * 2 - 2])
		{
			new_deadband = deadband_table[DEADBAND_ENTRIES * 2 - 1];
		}
		else
		{
			for(i = 0; i < DEADBAND_ENTRIES; i++)
			{
				if(adc.battery_avg < deadband_table[i * 2])
				{
					int deadband1 = deadband_table[i * 2 - 1];
					int deadband2 = deadband_table[i * 2 + 1];
					int voltage1 = deadband_table[i * 2 - 2];
					int voltage2 = deadband_table[i * 2];
					new_deadband = (adc.battery_avg - voltage1) *
						(deadband2 - deadband1) / 
						(voltage2 - voltage1) +
						deadband1;
					break;
				}
			}
		}
		
		if(new_deadband > motor.deadband)
		{
			motor.deadband++;
		}
		else
		if(new_deadband < motor.deadband)
		{
			motor.deadband--;
		}



TRACE
print_number(&uart, adc.battery_avg);
print_number(&uart, new_deadband);



	}
}



void init_motor()
{
	motor.deadband = MIN_DEADBAND;
	motor.phase = 0 * FRACTION;
	motor.power = MAX_POWER;

	TimHandle.Instance = TIM1;
	TimHandle.Init.Prescaler         = 0;
	TimHandle.Init.Period            = PERIOD;
	TimHandle.Init.ClockDivision     = 0;
	TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	TimHandle.Init.RepetitionCounter = 0;
	HAL_TIM_PWM_Init(&TimHandle);


	sConfig.OCMode       = TIM_OCMODE_PWM1;
	sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
	
	sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
	sConfig.OCIdleState  = TIM_OCIDLESTATE_SET;
	
	sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
	sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	sConfig.Pulse = 0;


	HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_3);

	write_motor();

	TimHandle.Instance->BDTR = TIM_BDTR_MOE | TIM_BDTR_OSSI | motor.deadband;

	HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&TimHandle, TIM_CHANNEL_1);
	
	HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&TimHandle, TIM_CHANNEL_2);
	
	HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&TimHandle, TIM_CHANNEL_3);
}


void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   GPIO_InitStruct;
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* TIMx Peripheral clock enable */
  __HAL_RCC_TIM1_CLK_ENABLE();

  /* Enable all GPIO Channels Clock requested */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;


  GPIO_InitStruct.Pin = (1 << N_MOSFET_PIN1) |
  	(1 << N_MOSFET_PIN2) |
  	(1 << N_MOSFET_PIN3);
  HAL_GPIO_Init(N_MOSFET_GPIO, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = (1 << P_MOSFET_PIN1) |
  	(1 << P_MOSFET_PIN2) |
  	(1 << P_MOSFET_PIN3);
  HAL_GPIO_Init(P_MOSFET_GPIO, &GPIO_InitStruct);

}





