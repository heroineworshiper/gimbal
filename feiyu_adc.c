#include "feiyu_mane.h"
#include "feiyu_motor.h"
#include "feiyu_adc.h"
#include "feiyu_uart.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_hal_adc.h"


adc_t adc;
ADC_HandleTypeDef    AdcHandle1;
ADC_HandleTypeDef    AdcHandle2;
ADC_ChannelConfTypeDef   sConfig;

#define ADC_PIN0 0
#define ADC_PIN1 1
#define ADC_PIN2 2
#define ADC_GPIO GPIOA
#define OVERSAMPLE 2048


void get_adc0();


#if 1

/*
 * void get_adc2()
 * {
 * 	if(HAL_IS_BIT_SET(AdcHandle1.Instance->SR, ADC_FLAG_EOC))
 * 	{
 * 
 * 		adc.current2 = AdcHandle1.Instance->DR;
 * 		adc.current_function = get_adc0;
 * 
 * 
 * 	}
 * }
 */
int get_pwm1();

void get_adc1()
{
	if(HAL_IS_BIT_SET(AdcHandle2.Instance->SR, ADC_FLAG_EOC))
	{
		__HAL_ADC_CLEAR_FLAG(&AdcHandle2, ADC_FLAG_EOC);
		/* Start ADC conversion on regular group with SW start */
    	SET_BIT(AdcHandle2.Instance->CR2, (ADC_CR2_SWSTART | ADC_CR2_EXTTRIG));

		adc.current2 += AdcHandle2.Instance->DR;
		adc.current2_count++;
		if(adc.current2_count > OVERSAMPLE)
		{
			adc.current2_avg = adc.current2 / adc.current2_count;





#ifdef TEST_MOTOR
TRACE
print_fixed(&uart, motor.phase);
//print_number(&uart, motor.pwm1);
//print_number(&uart, motor.pwm2);
//print_number(&uart, get_pwm1());
print_number(&uart, adc.current1_avg);
print_number(&uart, adc.current2_avg);
//print_number(&uart, motor.power);
//motor.phase += FRACTION;
//write_motor();
//if(motor.pwm1 < 1800)
//{
//	motor.pwm1 += 1;
//	write_motor2();
//}
#endif




			adc.current2_count = 0;
			adc.current2 = 0;
		}
		adc.current_function = get_adc0;
	}

}

#endif // 0


void get_adc0()
{
	if(HAL_IS_BIT_SET(AdcHandle1.Instance->SR, ADC_FLAG_EOC))
	{

		adc.current1 += AdcHandle1.Instance->DR;
		__HAL_ADC_CLEAR_FLAG(&AdcHandle1, ADC_FLAG_EOC);
		/* Start ADC conversion on regular group with SW start */
    	SET_BIT(AdcHandle1.Instance->CR2, (ADC_CR2_SWSTART | ADC_CR2_EXTTRIG));

		adc.current1_count++;
		if(adc.current1_count > OVERSAMPLE)
		{
			adc.current1_avg = adc.current1 / adc.current1_count;
			adc.current1_count = 0;
			adc.current1 = 0;
		}

		adc.current_function = get_adc1;
	}

}



// current sensor
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef          GPIO_InitStruct;
  RCC_PeriphCLKInitTypeDef  PeriphClkInit;
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable clock of GPIO associated to the peripheral channels */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  /* Enable clock of ADCx peripheral */
  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_ADC2_CLK_ENABLE();

  /* Configure ADCx clock prescaler */
  /* Caution: On STM32F1, ADC clock frequency max is 14MHz (refer to device   */
  /*          datasheet).                                                     */
  /*          Therefore, ADC clock prescaler must be configured in function   */
  /*          of ADC clock source frequency to remain below this maximum      */
  /*          frequency.                                                      */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* Configure GPIO pin of the selected ADC channel */
  GPIO_InitStruct.Pin = (1 << ADC_PIN0) | (1 << ADC_PIN1) | (1 << ADC_PIN2);
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADC_GPIO, &GPIO_InitStruct);
}




void init_adc()
{
// multiplexing doesn't work.  The best we can do is 2 channels on 2 ADCs
	/* Configuration of ADCx init structure: ADC parameters and regular group */
	AdcHandle1.Instance = ADC1;
	AdcHandle1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
	AdcHandle1.Init.ScanConvMode          = ADC_SCAN_ENABLE;              /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
	AdcHandle1.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 conversion at each conversion trig */
	AdcHandle1.Init.NbrOfConversion       = 1;                             /* Parameter discarded because sequencer is disabled */
	AdcHandle1.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
	AdcHandle1.Init.NbrOfDiscConversion   = 1;                             /* Parameter discarded because sequencer is disabled */
	AdcHandle1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
	HAL_ADC_Init(&AdcHandle1);

	/* Configuration of channel on ADCx regular group on sequencer rank 1 */
	/* Note: Considering IT occurring after each ADC conversion if ADC          */
	/*       conversion is out of the analog watchdog window selected (ADC IT   */
	/*       enabled), select sampling time and ADC clock with sufficient       */
	/*       duration to not create an overhead situation in IRQHandler.        */
	sConfig.Channel      = ADC_CHANNEL_0;
	sConfig.Rank         = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;

	HAL_ADC_ConfigChannel(&AdcHandle1, &sConfig);
	__HAL_ADC_ENABLE(&AdcHandle1);


	__HAL_ADC_CLEAR_FLAG(&AdcHandle1, ADC_FLAG_EOC);
	/* Start ADC conversion on regular group with SW start */
    SET_BIT(AdcHandle1.Instance->CR2, (ADC_CR2_SWSTART | ADC_CR2_EXTTRIG));

#if 1
	/* Configuration of ADCx init structure: ADC parameters and regular group */
	AdcHandle2.Instance = ADC2;
	AdcHandle2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
	AdcHandle2.Init.ScanConvMode          = ADC_SCAN_ENABLE;              /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
	AdcHandle2.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 conversion at each conversion trig */
	AdcHandle2.Init.NbrOfConversion       = 1;                             /* Parameter discarded because sequencer is disabled */
	AdcHandle2.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
	AdcHandle2.Init.NbrOfDiscConversion   = 1;                             /* Parameter discarded because sequencer is disabled */
	AdcHandle2.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
	HAL_ADC_Init(&AdcHandle2);

	/* Configuration of channel on ADCx regular group on sequencer rank 1 */
	/* Note: Considering IT occurring after each ADC conversion if ADC          */
	/*       conversion is out of the analog watchdog window selected (ADC IT   */
	/*       enabled), select sampling time and ADC clock with sufficient       */
	/*       duration to not create an overhead situation in IRQHandler.        */
	sConfig.Channel      = ADC_CHANNEL_1;
	sConfig.Rank         = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;

	HAL_ADC_ConfigChannel(&AdcHandle2, &sConfig);
	__HAL_ADC_ENABLE(&AdcHandle2);

	__HAL_ADC_CLEAR_FLAG(&AdcHandle2, ADC_FLAG_EOC);
	/* Start ADC conversion on regular group with SW start */
    SET_BIT(AdcHandle2.Instance->CR2, (ADC_CR2_SWSTART | ADC_CR2_EXTTRIG));
#endif // 0


	adc.current_function = get_adc0;
}

void handle_adc()
{
	adc.current_function();
}












