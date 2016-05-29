#include "stm32f1xx_hal_dma.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_hal_tim.h"

#include "feiyu_mane.h"
#include "feiyu_motor.h"
#include "feiyu_uart.h"

motor_t motor;

TIM_HandleTypeDef    TimHandle;

void init_motor()
{
  TimHandle.Instance = TIM1;
  TimHandle.Init.Prescaler         = 0;
  TimHandle.Init.Period            = 65535;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;
  HAL_TIM_PWM_Init(&TimHandle);
	
	
	
}





