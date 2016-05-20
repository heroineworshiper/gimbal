// bootloader for the feiyu stm32f103c8 
// STM32Cube_FW_F1_V1.3.0 only defines the stm32f103xb

// build with
// make feiyu_bootloader.bin


// program with 
// /amazon/root/nordic/JLink_Linux_V510p_x86_64/JLinkExe
// connect
// STM32F103C8
// s
// loadbin feiyu_bootloader.bin 0
// r
// g



#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_hal_tim.h"

#include "feiyu_mane.h"
#include "feiyu_uart.h"
#include "arm_linux.h"

//#define READ_ONLY

int mane_time = 0;

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




void init_leds()
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef  GPIO_InitStruct;
	GPIO_InitStruct.Pin       = 1 << BLUE_LED_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(BLUE_LED_GPIO, &GPIO_InitStruct);
	CLEAR_PIN(BLUE_LED_GPIO, 1 << BLUE_LED_PIN)

	GPIO_InitStruct.Pin       = 1 << RED_LED_PIN;
	HAL_GPIO_Init(RED_LED_GPIO, &GPIO_InitStruct);
	SET_PIN(RED_LED_GPIO, 1 << RED_LED_PIN)

}


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


#ifndef READ_ONLY
static FLASH_EraseInitTypeDef EraseInitStruct;
#endif

void do_bootloader()
{
	int i;
	flush_uart(&uart);
	while(1)
	{
		send_uart(&uart, ">", 1);

		while(1)
		{
//TOGGLE_PIN(BLUE_LED_GPIO, 1 << BLUE_LED_PIN);
			handle_uart();
			if(uart_got_input(&uart))
			{
				char c = uart_get_input(&uart);

				switch(c)
				{
					case 'p':
	// passthrough
						print_text(&uart, "\nPassthrough mode\n");
						while(1)
						{
							handle_uart();
							if(uart_got_input(&uart2))
							{
								char c[4];
								c[0] = uart_get_input(&uart2);
								send_uart(&uart, c, 1);
							}

							if(uart_got_input(&uart))
							{
								char c[4];
								c[0] = uart_get_input(&uart);
//TOGGLE_PIN(BLUE_LED_GPIO, 1 << BLUE_LED_PIN);
								send_uart(&uart2, c, 1);
							}
						}
						break;

// write to flash
					case 'w':
					{
						unsigned char buffer[8];
						unsigned char write_buffer[FLASH_PAGE_SIZE];

						for(i = 0; i < 8; i++)
						{
							buffer[i] = read_char();
						}

						int address = *(int*)buffer;
						int bytes = *(int*)(buffer + 4);
						print_text(&uart, "\nWriting address 0x");
						print_hex(&uart, address);
						print_text(&uart, "bytes=");
						flush_uart(&uart);
						print_number(&uart, bytes);
						flush_uart(&uart);
						print_text(&uart, "\n>");
						flush_uart(&uart);
						
						for(i = 0; i < bytes; i++)
						{
							write_buffer[i] = read_char();
						}

#ifndef READ_ONLY
						HAL_FLASH_Unlock();
						
						EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
						EraseInitStruct.PageAddress = address;
						EraseInitStruct.NbPages     = 1;
						uint32_t error = 0;
						HAL_FLASHEx_Erase(&EraseInitStruct, &error);
						
						for(i = 0; i < bytes; i += 4)
						{
							HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 
								address + i, 
								*(uint32_t*)(write_buffer + i));
						}
						
						HAL_FLASH_Lock();
#endif
						
						print_text(&uart, "\n>");
						break;
					}

// read from flash
					case 'r':
					{
						unsigned char buffer[8];

						for(i = 0; i < 8; i++)
						{
							buffer[i] = read_char();
						}

						int address = *(int*)buffer;
						int bytes = *(int*)(buffer + 4);
						print_text(&uart, "\nReading address 0x");
						print_hex(&uart, address);
						print_text(&uart, "bytes=");
						print_number(&uart, bytes);
						print_text(&uart, "\n>");
						flush_uart(&uart);
						
						unsigned char *ptr = (unsigned char*)address;
						send_uart(&uart, ptr, bytes);
						
						print_text(&uart, "\n>");
						break;
					}

// start program
					case 'g':
						return;
						break;

					default:
						print_text(&uart, "\nUnknown command.\n");
						break;
				}
			}
		}
	}
}


void main()
{
// disable JTAG pins
  	__HAL_RCC_AFIO_CLK_ENABLE();
	__HAL_AFIO_REMAP_SWJ_NOJTAG();

// reset handler
	void (*user_main)(void) = (void (*)(void))(PROGRAM_START + 0x0168);
// calling main directly works.  Calling _mainCRTStartup doesn't
//	void (*user_main)(void) = (void (*)(void))(PROGRAM_START + 0x1ce5);

	init_linux();
	HAL_Init();
	SystemClock_Config();
	init_uart();


	init_leds();
	

// wait a while for a code to go into bootloader mode while printing the 
// start code
	while(mane_time < 3 * HZ)
	{
		handle_uart();
		if(UART_EMPTY(&uart))
		{
			print_text(&uart, "**** BOOT\n");
		}
		if(uart_got_input(&uart))
		{
			char c = uart_get_input(&uart);
			if(c == 'b')
			{
// go into bootloader
				do_bootloader();
				break;
			}
		}
	}

	
	print_text(&uart, "\nStarting mane program\n");
	flush_uart(&uart);


	__disable_irq();
	CLEAR_PIN(BLUE_LED_GPIO, 1 << BLUE_LED_PIN);
	CLEAR_PIN(RED_LED_GPIO, 1 << RED_LED_PIN);


	
	user_main();
}












