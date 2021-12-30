
/*
 * Feiyu gimbal hack
 *
 * Copyright (C) 2016 Adam Williams <broadcast at earthling dot net>
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

#define MIN_DEADBAND 196



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


#define CALCULATE_WAVEFORM(x) ((uint32_t)motor_table[x] * (max_sin - min_sin) / 65535 + min_sin)

#ifdef ANTICOGGING


#ifdef BOARD2
const char index_offset[] =
{
	0, 3, 14, 14, 14, 14, 14, 14, 15, 15, 15, 16, 16, 16, 17, 17, 
	17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 
	18, 18, 18, 18, 18, 17, 17, 16, 16, 15, 15, 14, 13, 13, 12, 12, 
	12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 7, 
	7, 6, 6, 5, 6, 15, 16, 16, 16, 15, 15, 15, 14, 15, 24, 24, 
	24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 
	19, 19, 18, 18, 18, 17, 17, 17, 16, 16, 16, 15, 15, 14, 14, 13, 
	13, 12, 12, 11, 10, 10, 9, 9, 8, 7, 7, 7, 7, 6, 6, 6, 
	5, 5, 4, 3, 3, 2, 1, 1, 0, -1, -2, -2, -3, -4, -4, -5, 
	-6, -6, -7, -8, -8, -9, -9, -10, -11, -11, -12, -13, -14, -14, -15, -16, 
	-17, -18, -18, -19, -20, -21, -22, -22, -23, -24, -25, -26, -26, -27, -28, -28, 
	-29, -29, -30, -30, -31, -31, -31, -31, -31, -32, -32, -32, -31, -21, -21, -21, 
	-22, -22, -22, -21, -10, -10, -10, -10, -10, -10, -10, -10, -11, -11, -11, -11, 
	-11, -11, -11, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -12, -12, 
	-12, -12, -12, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, 
	-12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, 
	-10, 0, 1, 2, 9, 14, 14, 14, 15, 15, 15, 15, 16, 16, 16, 17, 
	17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 
	18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 17, 17, 17, 16, 16, 16, 
	15, 15, 15, 14, 14, 13, 13, 12, 12, 11, 11, 11, 16, 21, 21, 20, 
	20, 20, 21, 27, 32, 32, 32, 31, 31, 30, 29, 29, 28, 28, 27, 26, 
	26, 25, 24, 23, 23, 22, 21, 21, 20, 20, 19, 19, 18, 18, 17, 17, 
	16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 13, 12, 12, 11, 11, 10, 
	10, 9, 8, 8, 7, 7, 6, 6, 5, 5, 5, 4, 4, 3, 3, 3, 
	2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -4, -5, -5, -6, 
	-6, -7, -8, -8, -9, -9, -9, -10, -10, -11, -11, -12, -12, -13, -13, -14, 
	-15, -15, -16, -16, -17, -17, -18, -19, -19, -20, -20, -21, -21, -21, -22, -22, 
	-23, -23, -24, -25, -25, -26, -26, -27, -28, -28, -29, -29, -30, -30, -30, -20, 
	-19, -20, -20, -20, -20, -20, -13, -11, -11, -11, -12, -12, -13, -14, -14, -14, 
	-14, -14, -14, -14, -14, -15, -15, -15, -15, -15, -15, -16, -15, -15, -15, -15, 
	-15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, 
	-15, -15, -14, -14, -14, -14, -14, -13, -13, -13, -13, -13, -12, -12, -4, -1
};
#endif // BOARD2

#ifdef BOARD1

const char index_offset[] =
{
	0, 0, 8, 13, 14, 14, 13, 13, 13, 12, 12, 11, 11, 11, 10, 10, 
	10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 10, 10, 10, 9, 9, 8, 
	8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 6, 6, 6, 5, 5, 
	4, 4, 3, 3, 2, 2, 1, 1, 1, 0, 0, 0, -1, -1, -1, -2, 
	-2, -2, -3, -3, -3, -4, -4, -5, -5, -5, -6, -6, -7, -7, -7, -8, 
	-8, -8, -1, 2, 5, 5, 5, 5, 5, 14, 6, 15, 15, 15, 15, 14, 
	14, 13, 13, 12, 12, 12, 11, 10, 10, 10, 9, 9, 8, 8, 8, 7, 
	7, 7, 6, 6, 6, 6, 5, 5, 5, 4, 4, 3, 3, 3, 2, 2, 
	1, 1, 1, 0, 0, -1, -1, -1, -2, -2, -3, -3, -3, -4, -4, -4, 
	-5, -5, -6, -6, -6, -7, -7, -7, -8, -8, -9, -9, -9, -10, -10, -10, 
	-11, -11, -12, -12, -12, -13, -13, -14, -14, -14, -14, -6, -13, -3, -3, -2, 
	-2, 6, 10, 10, 10, 10, 10, 9, 9, 8, 8, 7, 7, 6, 6, 5, 
	5, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 1, 1, 
	1, 0, 0, -1, -2, -2, -3, -3, -4, -4, -4, -5, -5, -5, -5, -5, 
	-5, -5, -5, -5, -5, -6, -6, -6, -6, -7, -7, -7, -8, -8, -8, -8, 
	-8, -8, -8, -8, -8, -8, -8, -8, -9, -9, -9, -9, -9, -9, 4, 5, 
	5, 6, 6, 16, 16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 13, 13, 
	12, 12, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 9, 9, 9, 
	8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 2, 1, 
	1, 0, 0, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -3, -3, 
	-3, -3, -4, -4, -4, -5, -5, -6, -6, -6, -6, -7, -7, -7, 0, 2, 
	4, 5, 5, 5, 6, 17, 17, 17, 17, 17, 17, 16, 16, 15, 15, 14, 
	13, 13, 12, 12, 11, 11, 10, 10, 9, 9, 8, 8, 8, 8, 7, 7, 
	7, 7, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 3, 2, 2, 
	1, 1, 0, 0, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -3, -3, 
	-3, -4, -4, -5, -5, -6, -6, -7, -7, -8, -8, -9, -9, -9, -10, -10, 
	-11, -11, -11, -11, -11, -11, -12, -12, -12, -13, -13, -13, -14, -14, -5, -4, 
	-1, 0, 0, 0, 1, 9, 9, 9, 9, 9, 8, 8, 7, 7, 6, 6, 
	5, 5, 4, 4, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 
	2, 1, 1, 0, 0, -1, -1, -1, -2, -2, -3, -3, -3, -3, -3, -4, 
	-4, -4, -4, -4, -4, -5, -5, -5, -6, -6, -7, -7, -7, -8, -8, -9, 
	-9, -10, -10, -10, -10, -11, -11, -11, -11, -11, -11, -12, -12, -12, -8, -11
};


#endif // BOARD1


#ifdef BOARD0


const char index_offset[] =
{
	0, 1, 2, 7, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 
	14, 14, 15, 15, 16, 16, 17, 17, 17, 17, 17, 17, 16, 16, 16, 15, 
	15, 15, 14, 14, 14, 13, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 
	15, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 13, 12, 12, 12, 19, 
	19, 19, 19, 26, 26, 26, 27, 28, 37, 37, 36, 36, 35, 35, 34, 34, 
	33, 33, 32, 31, 31, 30, 29, 29, 28, 27, 27, 26, 25, 25, 24, 23, 
	22, 22, 21, 20, 20, 19, 18, 17, 17, 16, 15, 14, 14, 13, 12, 12, 
	11, 10, 9, 9, 8, 7, 7, 6, 5, 4, 4, 3, 2, 2, 1, 0, 
	0, -1, -2, -2, -3, -4, -4, -5, -6, -6, -7, -7, -8, -9, -9, -10, 
	-10, -11, -11, -12, -12, -13, -13, -14, -14, -15, -15, -16, -16, -17, -17, -17, 
	-18, -18, -19, -19, -20, -20, -21, -21, -22, -22, -23, -23, -24, -24, -25, -25, 
	-26, -26, -27, -27, -28, -28, -29, -29, -30, -30, -31, -31, -31, -31, -32, -31, 
	-32, -18, -18, -18, -9, -9, -7, -7, -7, -8, -8, -8, -9, -9, -10, -10, 
	-11, -12, -12, -13, -13, -14, -15, -15, -16, -16, -17, -17, -18, -19, -19, -19, 
	-20, -20, -20, -20, -15, -14, -14, -14, -14, -14, -15, -15, -15, -16, -16, -17, 
	-17, -17, -18, -18, -18, -19, -19, -19, -19, -17, -13, -12, -12, -12, -12, -12, 
	-12, 0, 1, 0, 2, 8, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 
	16, 16, 17, 18, 19, 19, 20, 20, 20, 20, 20, 20, 19, 19, 19, 19, 
	18, 18, 18, 18, 17, 17, 17, 17, 16, 16, 16, 15, 15, 15, 15, 14, 
	14, 14, 14, 13, 13, 13, 13, 12, 12, 12, 12, 11, 11, 11, 11, 10, 
	10, 20, 20, 22, 22, 22, 22, 22, 23, 24, 31, 31, 31, 30, 30, 30, 
	29, 29, 28, 28, 27, 26, 26, 25, 24, 24, 23, 22, 22, 21, 20, 20, 
	19, 18, 18, 17, 16, 16, 15, 14, 14, 13, 12, 12, 11, 11, 10, 10, 
	9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 9, 9, 9, 8, 8, 
	7, 6, 6, 5, 4, 4, 3, 2, 2, 1, 0, 0, -1, -2, -3, -3, 
	-4, -5, -6, -6, -7, -8, -9, -9, -10, -11, -12, -13, -13, -14, -15, -16, 
	-17, -17, -18, -19, -20, -21, -21, -22, -23, -24, -25, -25, -26, -27, -28, -29, 
	-29, -30, -31, -32, -33, -33, -34, -35, -36, -36, -37, -38, -25, -25, -24, -24, 
	-24, -24, -23, -23, -17, -15, -8, -7, -7, -7, -7, -7, -8, -8, -8, -9, 
	-9, -10, -10, -11, -11, -11, -12, -12, -13, -13, -13, -13, -13, -13, -13, -12, 
	-11, -11, -11, -10, -10, -11, -11, -11, -11, -12, -12, -12, -13, -13, -13, -13, 
	-14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14, -13, -13, -12
};



#endif // BOARD0

#endif // ANTICOGGING


// set the PWM values for the phase
void write_motor()
{
// writing it without initializing it crashes the chip
	if(!motor.initialized)
	{
		return;
	}
	
	
	FIX_PHASE(motor.phase)
	CLAMP(motor.power, 0, MAX_POWER);

	
	int max_sin = MAX_SIN - (MAX_POWER - motor.power);
	int min_sin = MIN_SIN + (MAX_POWER - motor.power);
	int index1 = (motor.phase * WAVEFORM_SIZE / 360 / FRACTION) % WAVEFORM_SIZE;

#ifdef ANTICOGGING
	int offset = index_offset[index1];
	if(offset >= 128)
	{
		offset -= 256;
	}
	index1 += offset;
	if(index1 < 0)
	{
		index1 += WAVEFORM_SIZE;
	}
	index1 %= WAVEFORM_SIZE;
#endif



	int index2 = (index1 + 120 * WAVEFORM_SIZE / 360) % WAVEFORM_SIZE;
	int index3 = (index1 + 240 * WAVEFORM_SIZE / 360) % WAVEFORM_SIZE;


	motor.pwm1 = TimHandle.Instance->CCR1 = CALCULATE_WAVEFORM(index1);
	motor.pwm2 = TimHandle.Instance->CCR2 = CALCULATE_WAVEFORM(index2);
	motor.pwm3 = TimHandle.Instance->CCR3 = CALCULATE_WAVEFORM(index3);


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



#ifdef CALIBRATE_MOTOR

// develop the anti cogging table
void motor_test()
{
	static int test_time = 0;
	static int phase_to_hall[N_SIN] = { 0 };
	static int phase_index = 0;
	static int iterations = 0;
	static int done = 0;
	static int hall_offset = -1;
	
	if(!done && mane_time > 5 * HZ && 
		mane_time - test_time > /* HZ / 2 */ HZ / 100)
	{
// store hall value from the previous phase index
		test_time = mane_time;
		if(hall_offset < 0)
		{
			hall_offset = hall.value;
		}
		
		phase_to_hall[phase_index] += hall.value - hall_offset;
		
		TRACE
		print_number(&uart, hall.value - hall_offset);
		print_fixed(&uart, phase_index * 360 * FRACTION / N_SIN);
		print_lf(&uart);

		phase_index++;
		if(phase_index >= N_SIN)
		{
			iterations++;
			hall_offset = -1;
			phase_index = 0;
		}

// set new motor phase
		motor.phase = phase_index * 360 * FRACTION / N_SIN;
		write_motor();
// done
		if(iterations >= 4)
		{
			done = 1;


			int i;
			print_text(&uart, "\n\nconst char index_offset[] =\n{\n\t");

			for(i = 0; i < N_SIN; i++)
			{
//				TRACE
//				print_number(&uart, phase_to_hall[i] / iterations);
//				print_number(&uart, i * 360 / N_SIN);

// desired encoder value
				int want_hall = i * 
					(phase_to_hall[N_SIN - 1] - phase_to_hall[0]) /
					N_SIN + 
					phase_to_hall[0];

// required phase from measuring
				int j;
				int best_j;
				int best_diff;
				for(j = 0; j < N_SIN; j++)
				{
					int current_diff = ABS(phase_to_hall[j] - want_hall);
					if(j == 0 || best_diff > current_diff)
					{
						best_j = j;
						best_diff = current_diff;
					}
				}

#if 0
				print_number(&uart, want_hall / iterations);
// convert to degrees
				print_number(&uart, best_j * 360 / N_SIN);
				print_lf(&uart);
#endif

#if 1
#ifdef ANTICOGGING
				int offset = index_offset[i];
				if(offset >= 128)
				{
					offset -= 256;
				}
				print_number_nospace(&uart, offset + (best_j - i));
#else
				print_number_nospace(&uart, best_j - i);
#endif
				
				if(i < N_SIN - 1)
				{
					print_text(&uart, ", ");
				}

				if(i < N_SIN - 1 &&
					!((i + 1) % 16))
				{
					print_text(&uart, "\n\t");
				}
#endif

				flush_uart(&uart);
			}

			print_text(&uart, "\n};\n");
 		}
	}
	
	
	
	
}
#endif // CALIBRATE_MOTOR





void init_motor()
{
	motor.initialized = 1;
	motor.deadband = MIN_DEADBAND;
//	motor.deadband = 0;
	motor.phase = 0 * FRACTION;
	motor.power = MAX_POWER / 4;



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
//	TimHandle.Init.Prescaler         = 60;
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


  GPIO_InitStruct.Pin = 
  	(1 << N_MOSFET_PIN1) |
  	(1 << N_MOSFET_PIN2) | 
  	(1 << N_MOSFET_PIN3);

  HAL_GPIO_Init(N_MOSFET_GPIO, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = 
  	(1 << P_MOSFET_PIN1) | 
 	(1 << P_MOSFET_PIN2) |
  	(1 << P_MOSFET_PIN3);

  HAL_GPIO_Init(P_MOSFET_GPIO, &GPIO_InitStruct);

}





