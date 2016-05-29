#include "feiyu_mane.h"
#include "feiyu_adc.h"
#include "feiyu_uart.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_hal_adc.h"


adc_t adc;
ADC_HandleTypeDef    AdcHandle;
ADC_ChannelConfTypeDef   sConfig;

#define ADC_PIN0 0
#define ADC_PIN1 1
#define ADC_GPIO GPIOA

void get_adc0();

void get_adc1()
{
	if(HAL_IS_BIT_SET(AdcHandle.Instance->SR, ADC_FLAG_EOC))
	{
		adc.value2 = AdcHandle.Instance->DR;
		sConfig.Channel      = ADC_CHANNEL_0;
		HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);
		/* Start ADC conversion on regular group with SW start */
    	SET_BIT(AdcHandle.Instance->CR2, (ADC_CR2_SWSTART | ADC_CR2_EXTTRIG));
		adc.current_function = get_adc0;
/*
 * TRACE
 * print_number(&uart, adc.value1);
 * print_number(&uart, adc.value2);
 */
	}
}



void get_adc0()
{
	if(HAL_IS_BIT_SET(AdcHandle.Instance->SR, ADC_FLAG_EOC))
	{
		adc.value1 = AdcHandle.Instance->DR;
		sConfig.Channel      = ADC_CHANNEL_1;
		HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);
		/* Start ADC conversion on regular group with SW start */
    	SET_BIT(AdcHandle.Instance->CR2, (ADC_CR2_SWSTART | ADC_CR2_EXTTRIG));
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
  GPIO_InitStruct.Pin = (1 << ADC_PIN0) | (1 << ADC_PIN1);
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADC_GPIO, &GPIO_InitStruct);
}




void init_adc()
{
	/* Configuration of ADCx init structure: ADC parameters and regular group */
	AdcHandle.Instance = ADC1;

	AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
	AdcHandle.Init.ScanConvMode          = ADC_SCAN_DISABLE;              /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
	AdcHandle.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 conversion at each conversion trig */
	AdcHandle.Init.NbrOfConversion       = 1;                             /* Parameter discarded because sequencer is disabled */
	AdcHandle.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
	AdcHandle.Init.NbrOfDiscConversion   = 1;                             /* Parameter discarded because sequencer is disabled */
	AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */

	HAL_ADC_Init(&AdcHandle);

	/* Configuration of channel on ADCx regular group on sequencer rank 1 */
	/* Note: Considering IT occurring after each ADC conversion if ADC          */
	/*       conversion is out of the analog watchdog window selected (ADC IT   */
	/*       enabled), select sampling time and ADC clock with sufficient       */
	/*       duration to not create an overhead situation in IRQHandler.        */
	sConfig.Channel      = ADC_CHANNEL_0;
	sConfig.Rank         = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;

	HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);
	__HAL_ADC_ENABLE(&AdcHandle);
	
	/* Start ADC conversion on regular group with SW start */
    SET_BIT(AdcHandle.Instance->CR2, (ADC_CR2_SWSTART | ADC_CR2_EXTTRIG));
	
	adc.current_function = get_adc0;
}

void handle_adc()
{
	adc.current_function();
}












