#include "stm32f1xx_hal_dma.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_hal_tim.h"

#include "feiyu_mane.h"
#include "feiyu_adc.h"
#include "feiyu_motor.h"
#include "feiyu_uart.h"
#include "math.h"

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
#define MAX_DEADBAND 255

const uint16_t sin_table[] = {
	0x0708, 0x0734, 0x0760, 0x078c, 0x07b8, 0x07e4, 0x0810, 0x083b, 0x0867, 0x0892, 0x08bd, 0x08e8, 0x0912, 0x093c, 0x0966, 0x098f, 
	0x09b8, 0x09e1, 0x0a09, 0x0a31, 0x0a58, 0x0a7f, 0x0aa5, 0x0aca, 0x0af0, 0x0b14, 0x0b38, 0x0b5b, 0x0b7d, 0x0b9f, 0x0bc0, 0x0be1, 
	0x0c00, 0x0c1f, 0x0c3d, 0x0c5a, 0x0c77, 0x0c93, 0x0cad, 0x0cc7, 0x0ce0, 0x0cf8, 0x0d0f, 0x0d26, 0x0d3b, 0x0d4f, 0x0d63, 0x0d75, 
	0x0d86, 0x0d97, 0x0da6, 0x0db5, 0x0dc2, 0x0dce, 0x0dda, 0x0de4, 0x0ded, 0x0df5, 0x0dfc, 0x0e02, 0x0e07, 0x0e0b, 0x0e0d, 0x0e0f, 
	0x0e10, 0x0e0f, 0x0e0d, 0x0e0b, 0x0e07, 0x0e02, 0x0dfc, 0x0df5, 0x0ded, 0x0de4, 0x0dda, 0x0dce, 0x0dc2, 0x0db5, 0x0da6, 0x0d97, 
	0x0d86, 0x0d75, 0x0d63, 0x0d4f, 0x0d3b, 0x0d26, 0x0d0f, 0x0cf8, 0x0ce0, 0x0cc7, 0x0cad, 0x0c93, 0x0c77, 0x0c5a, 0x0c3d, 0x0c1f, 
	0x0c00, 0x0be1, 0x0bc0, 0x0b9f, 0x0b7d, 0x0b5b, 0x0b38, 0x0b14, 0x0af0, 0x0aca, 0x0aa5, 0x0a7f, 0x0a58, 0x0a31, 0x0a09, 0x09e1, 
	0x09b8, 0x098f, 0x0966, 0x093c, 0x0912, 0x08e8, 0x08bd, 0x0892, 0x0867, 0x083b, 0x0810, 0x07e4, 0x07b8, 0x078c, 0x0760, 0x0734, 
	0x0708, 0x06db, 0x06af, 0x0683, 0x0657, 0x062b, 0x05ff, 0x05d4, 0x05a8, 0x057d, 0x0552, 0x0527, 0x04fd, 0x04d3, 0x04a9, 0x0480, 
	0x0457, 0x042e, 0x0406, 0x03de, 0x03b7, 0x0390, 0x036a, 0x0345, 0x031f, 0x02fb, 0x02d7, 0x02b4, 0x0292, 0x0270, 0x024f, 0x022e, 
	0x020f, 0x01f0, 0x01d2, 0x01b5, 0x0198, 0x017c, 0x0162, 0x0148, 0x012f, 0x0117, 0x0100, 0x00e9, 0x00d4, 0x00c0, 0x00ac, 0x009a, 
	0x0089, 0x0078, 0x0069, 0x005a, 0x004d, 0x0041, 0x0035, 0x002b, 0x0022, 0x001a, 0x0013, 0x000d, 0x0008, 0x0004, 0x0002, 0x0000, 
	0x0000, 0x0000, 0x0002, 0x0004, 0x0008, 0x000d, 0x0013, 0x001a, 0x0022, 0x002b, 0x0035, 0x0041, 0x004d, 0x005a, 0x0069, 0x0078, 
	0x0089, 0x009a, 0x00ac, 0x00c0, 0x00d4, 0x00e9, 0x0100, 0x0117, 0x012f, 0x0148, 0x0162, 0x017c, 0x0198, 0x01b5, 0x01d2, 0x01f0, 
	0x020f, 0x022e, 0x024f, 0x0270, 0x0292, 0x02b4, 0x02d7, 0x02fb, 0x031f, 0x0345, 0x036a, 0x0390, 0x03b7, 0x03de, 0x0406, 0x042e, 
	0x0457, 0x0480, 0x04a9, 0x04d3, 0x04fd, 0x0527, 0x0552, 0x057d, 0x05a8, 0x05d4, 0x05ff, 0x062b, 0x0657, 0x0683, 0x06af, 0x06db
};

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

// set the PWM values for the phase
void write_motor()
{
	int index1 = (motor.phase * WAVEFORM_SIZE / 360 / FRACTION) % WAVEFORM_SIZE;
	int index2 = (index1 + 120 * WAVEFORM_SIZE / 360) % WAVEFORM_SIZE;
	int index3 = (index1 + 240 * WAVEFORM_SIZE / 360) % WAVEFORM_SIZE;
	
	TimHandle.Instance->CCR1 = sin_table[index1];
	TimHandle.Instance->CCR2 = sin_table[index2];
	TimHandle.Instance->CCR3 = sin_table[index3];
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
			set_deadband();
		}
		else
		if(new_deadband < motor.deadband)
		{
			motor.deadband--;
			set_deadband();
		}



TRACE
print_number(&uart, adc.battery_avg);
print_number(&uart, new_deadband);



	}
}

void set_deadband()
{
	TimHandle.Instance->BDTR = TIM_BDTR_MOE | TIM_BDTR_OSSI | motor.deadband;
//	__HAL_TIM_MOE_ENABLE(&TimHandle);
}

void init_motor()
{
// start at lowest power
	motor.deadband = MAX_DEADBAND;
	motor.phase = 0 * FRACTION;

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
	set_deadband();

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





