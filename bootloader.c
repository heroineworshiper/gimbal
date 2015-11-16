/*
 * STM32F4 Wifi flight controller
 * Copyright (C) 2012 Adam Williams <broadcast at earthling dot net>
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

// Useful commands to install it:

// src/openocd -f script

// telnet localhost 4444

// flash protect 0 0 11 off

// reset halt;flash write_image erase /amazon/root/gimbal/bootloader.bin 0x8000000; reset run

// verify_image /amazon/root/gimbal/bootloader.bin 0x8000000

// The bootloader should print out some information & crash, 
// since no program has been written.




#include "copter.h"
#include "stm32f4xx.h"
#include "stm32f4xx_flash.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "uart.h"


unsigned char write_buffer[65536];
//#define READ_ONLY

int main(void)
{
	int i;
	unsigned char code[4] = { 0, 0, 0, 0 };

// Address of actual executable
//	void (*user_main)(void) = (void (*)(void))PROGRAM_START + 1;
// reset handler
	void (*user_main)(void) = (void (*)(void))(PROGRAM_START + 0x0189);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |
			RCC_AHB1Periph_GPIOB |
			RCC_AHB1Periph_GPIOC, 
		ENABLE);


// kill motors
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | 
		GPIO_Pin_1 | 
		GPIO_Pin_2 | 
		GPIO_Pin_6 |
		GPIO_Pin_7 |
		GPIO_Pin_9 |
		GPIO_Pin_10 |
		GPIO_Pin_11;
	GPIO_ResetBits(GPIOA, GPIO_InitStructure.GPIO_Pin);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_ResetBits(GPIOB, GPIO_InitStructure.GPIO_Pin);
	GPIO_Init(GPIOB, &GPIO_InitStructure);



	init_uart();




	

// Show some information
	RCC_ClocksTypeDef RCC_ClocksStatus;
	RCC_GetClocksFreq(&RCC_ClocksStatus);
	print_text("SYSCLK_Frequency=");
	print_number(RCC_ClocksStatus.SYSCLK_Frequency);
	print_lf();
	print_text("SystemCoreClock=");
	print_number(SystemCoreClock);
	print_lf();
	print_text("HCLK_Frequency=");
	print_number(RCC_ClocksStatus.HCLK_Frequency);
	print_lf();
	print_text("PCLK1_Frequency=");
	print_number(RCC_ClocksStatus.PCLK1_Frequency);
	print_lf();
	print_text("PCLK2_Frequency=");
	print_number(RCC_ClocksStatus.PCLK2_Frequency);
	print_lf();

	flush_uart();

	
// Wait a few seconds for host to send data
 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  
   	TIM_Cmd(TIM2, DISABLE);
 	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 11999;
  	TIM_TimeBaseStructure.TIM_Prescaler = 5;
  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  	TIM_ARRPreloadConfig(TIM2, ENABLE);
/* TIM2 enable counter */
  	TIM_Cmd(TIM2, ENABLE);
	
	int delay = 1000;
	int seconds = 0;
	int do_bootloader = 0;
	while(seconds < 1)
	{

		if(TIM_GetFlagStatus(TIM2, TIM_FLAG_Update) != RESET)
		{
			TIM_ClearFlag(TIM2, TIM_FLAG_Update);
			delay--;
			if(!delay)
			{
				delay = 1000;
				seconds++;
//				print_number(seconds);
//				print_lf();
			}
		}

		handle_uart();
		
		
		if(got_input())
		{
			code[0] = code[1];
			code[1] = code[2];
			code[2] = get_input();

//			send_uart(code + 2, 1);
/*
 * TRACE
 * print_number(code[0]);
 * print_number(code[1]);
 * print_number(code[2]);
 */
			if(code[0] == 'X' &&
				code[1] == 'Y' &&
				code[2] == 'Z')
			{
				do_bootloader = 1;
				break;
			}
		}
	}
	
	if(!do_bootloader)
	{
		
		goto boot;
	}

// Trigger the programmer to continue
	print_text("done\n");
	while(1)
	{
		handle_uart();

		if(got_input())
		{
// Get command
			char c = get_input();
//			print_text("Command: ");
//			send_uart(&c, 1);
//			print_lf();
//			flush_uart();


			switch(c)
			{
// Erase flash sector
				case 'E':
				{
					int sector = read_char();

					print_text("Erasing flash sector ");
					unsigned char tmp[1];
					print_number(sector);
					print_lf();
					flush_uart();

// Don't erase bootloader
					if(sector >= 4)
					{
#ifndef READ_ONLY
						const int sectors[] = 
						{
							FLASH_Sector_0 ,
							FLASH_Sector_1 ,
							FLASH_Sector_2 ,
							FLASH_Sector_3 ,
							FLASH_Sector_4 ,
							FLASH_Sector_5 ,
							FLASH_Sector_6 ,
							FLASH_Sector_7 ,
							FLASH_Sector_8 ,
							FLASH_Sector_9 ,
							FLASH_Sector_10,
							FLASH_Sector_11
						};

						FLASH_Unlock();
						FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                						FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR); 
						int StartSector = sectors[sector];
    					if (FLASH_EraseSector(StartSector, VoltageRange_3) != FLASH_COMPLETE)
						{
							TRACE
							print_text("FLASH_EraseSector failed\n");
							flush_uart();
						}
						FLASH_Lock(); 
#endif

					}
					
					print_text("done\n");
					break;
				}

// Program flash
				case 'W':
				{
					unsigned char buffer[8];
					for(i = 0; i < 8; i++)
						buffer[i] = read_char();

					int address = *(int*)buffer;
					int bytes = *(int*)(buffer + 4);
					
					for(i = 0; i < bytes; i++)
					{
						write_buffer[i] = read_char();
					}
					
					
					print_text("Writing flash address ");
					print_hex(address);
					print_text(" bytes ");
					print_number(bytes);
					print_lf();
					flush_uart();

#ifndef READ_ONLY					
					FLASH_Unlock();

					/* Erase the user Flash area
    				  (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

					/* Clear pending flags (if any) */  
					FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                  		FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR); 

					for(i = 0; i < bytes; i += 4)
					{
					    if (FLASH_ProgramWord(address + i, 
							*(int*)(write_buffer + i)) != FLASH_COMPLETE)
						{
							print_text("FLASH_ProgramWord failed\n");
							flush_uart();
						}
					}
	  				FLASH_Lock(); 
#endif


					
					print_text("done\n");
					break;
				}

// Boot it
				case 'B':
					goto boot;
					break;
				
				default:
					print_text("done\n");
					break;
			}
		}
	}


boot:

	print_text("Jumping to 0x");
	print_hex((uint32_t)user_main);
	print_lf();
	print_lf();
	print_lf();
	flush_uart();


	user_main();
}


// signal handler
int raise(int sig)
{
	return 0;
}




