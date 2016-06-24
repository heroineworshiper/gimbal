#include "stm32f1xx_hal_dma.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_hal_tim.h"

#include "feiyu_mane.h"
#include "feiyu_adc.h"
#include "feiyu_hall.h"
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



#define N_SIN 512
static uint16_t motor_table[N_SIN];

#define WAVEFORM_SIZE (sizeof(motor_table) / sizeof(uint16_t))




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
#define CALCULATE_WAVEFORM(x) ((uint32_t)motor_table[x] * (max_sin - min_sin) / 65535 + min_sin)

// set the PWM values for the phase
void write_motor()
{
	FIX_PHASE(motor.phase)
	CLAMP(motor.power, 0, MAX_POWER);


	
	int max_sin = MAX_SIN - (MAX_POWER - motor.power);
	int min_sin = MIN_SIN + (MAX_POWER - motor.power);
	int index1 = (motor.phase * WAVEFORM_SIZE / 360 / FRACTION) % WAVEFORM_SIZE;
	int index2 = (index1 + 120 * WAVEFORM_SIZE / 360) % WAVEFORM_SIZE;
	int index3 = (index1 + 240 * WAVEFORM_SIZE / 360) % WAVEFORM_SIZE;


#ifdef TEST_MOTOR
// short 1 MOSFET
	motor.pwm1 = TimHandle.Instance->CCR1 = MIN_SIN;
	motor.pwm2 = TimHandle.Instance->CCR2 = MAX_SIN / 2;
	motor.pwm3 = TimHandle.Instance->CCR3 = MIN_SIN;
#else

	motor.pwm1 = TimHandle.Instance->CCR1 = CALCULATE_WAVEFORM(index1);
	motor.pwm2 = TimHandle.Instance->CCR2 = CALCULATE_WAVEFORM(index2);
	motor.pwm3 = TimHandle.Instance->CCR3 = CALCULATE_WAVEFORM(index3);
#endif


// print_number(&uart, TimHandle.Instance->CCR1);
// print_number(&uart, TimHandle.Instance->CCR2);
// print_number(&uart, TimHandle.Instance->CCR3);
// flush_uart(&uart);

//TRACE
//print_number(&uart, index1);
//print_number(&uart, CALCULATE_WAVEFORM(index1));
//print_number(&uart, CALCULATE_WAVEFORM(index2));
//print_number(&uart, CALCULATE_WAVEFORM(index3));
//	print_phases();
}

#ifdef TEST_MOTOR
void write_motor2()
{
	TimHandle.Instance->CCR1 = motor.pwm1;
	TimHandle.Instance->CCR2 = motor.pwm2;
	TimHandle.Instance->CCR3 = motor.pwm3;
}
#endif






void init_motor()
{
	motor.deadband = MIN_DEADBAND;
	motor.phase = 0 * FRACTION;
	motor.power = MAX_POWER / 2;



	int i;
	for(i = 0; i < N_SIN; i++)
	{
		int angle = 360 * FRACTION * i / N_SIN;
		int value = 0x7fff + (sin_fixed14(angle) << 1);
//      motor_table[i] = 0x7fff + sin_fixed(angle) * 0x7f;

		if(value < 0)
		{
			value = 0;
		}
      	motor_table[i] = value;
//TRACE
//print_number(&uart, motor_table[i]);
//flush_uart(&uart);
	}
	


	TimHandle.Instance = TIM1;
	TimHandle.Init.Prescaler         = 0;
//	TimHandle.Init.Prescaler         = 20;
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





