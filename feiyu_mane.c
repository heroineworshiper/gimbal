// bootloader for the feiyu stm32f103c8 
// STM32Cube_FW_F1_V1.3.0 only defines the stm32f103xb

// build with
// make


// program with 
// feiyu_program feiyu_mane.bin 
// feiyu_program -p 1 feiyu_mane.bin 

#include "feiyu_imu.h"
#include "feiyu_mane.h"
#include "feiyu_uart.h"
#include "arm_linux.h"
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_hal_rcc.h"

volatile int mane_time = 0;


#if 0
// the mane timer
TIM_HandleTypeDef    TimHandle;
void init_timer()
{
	__HAL_RCC_TIM3_CLK_ENABLE();


// auto reload value
	TIM3->ARR = 10000 - 1;
// prescaler
	TIM3->PSC = 720000 - 1;
// mode bits
	uint32_t cr1_value = TIM3->CR1;
	cr1_value &= ~(TIM_CR1_DIR | TIM_CR1_CMS | TIM_CR1_CKD);
	cr1_value |= TIM_COUNTERMODE_UP | TIM_CLOCKDIVISION_DIV1;
	TIM3->CR1 = cr1_value;

// enable interrupt
	TIM3->DIER |= TIM_IT_UPDATE;
// start timer
	TIM3->CR1 |= TIM_CR1_CEN;
	
	HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
}




void TIM3_IRQHandler()
{
}
#endif // 0



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
#endif

#ifdef BOARD1
	print_text(&uart, "Welcome to Feiyu BOARD1\n");
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
	
	
#endif // BOARD2


	init_adc();
	init_hall();
	init_motor();


	while(1)
	{
		handle_uart();
		handle_imu();
		handle_hall();
		handle_adc();
	}
}












