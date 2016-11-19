
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


#include "feiyu_mane.h"
#include "feiyu_motor.h"
#include "feiyu_hall.h"
#include "feiyu_uart.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_hal_spi.h"
#include "arm_linux.h"

/* SPI handler declaration */
SPI_HandleTypeDef SpiHandle;

hall_t hall;
volatile int temp = 0;

#define CS_PIN 4
#define MOSI_PIN 7
#define SPI_GPIO GPIOA

void send_hall_command();

void hall_sleep()
{
	if(mane_time - hall.time >= HZ / 1000 ||
		mane_time < hall.time)
	{
		hall.current_function = send_hall_command;
	}
}

void read_hall_result()
{
	if(__HAL_SPI_GET_FLAG(&SpiHandle, SPI_FLAG_BSY) == RESET)
	{
// change to output
		GPIO_InitTypeDef  GPIO_InitStruct;
    	GPIO_InitStruct.Pin = 1 << MOSI_PIN;
    	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    	GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
    	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
   		HAL_GPIO_Init(SPI_GPIO, &GPIO_InitStruct);

// raise CS
		SET_PIN(SPI_GPIO, 1 << CS_PIN);
		hall.value = SpiHandle.Instance->DR;
		hall.got_readout = 1;

#ifdef BOARD0
		if(hall.value > 27000)
		{
			hall.value -= 27000;
		}
		else
		{
			hall.value -= 16384;
			hall.value += (32767 - 27000);
		}
#endif // BOARD0

		hall.count++;
//TRACE
//print_number(&uart, hall.value);

		hall.current_function = hall_sleep;
	}
}

void wait_hall_command()
{
	if(__HAL_SPI_GET_FLAG(&SpiHandle, SPI_FLAG_BSY) == RESET)
	{
// change to input mode
		GPIO_InitTypeDef  GPIO_InitStruct;
    	GPIO_InitStruct.Pin = 1 << MOSI_PIN;
    	GPIO_InitStruct.Mode      = GPIO_MODE_AF_INPUT;
    	GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
    	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
   		HAL_GPIO_Init(SPI_GPIO, &GPIO_InitStruct);

		temp = SpiHandle.Instance->DR;

// read the result
		SpiHandle.Instance->DR = 0x0000;
		hall.current_function = read_hall_result;
	}
}

void send_hall_command()
{
	hall.time = mane_time;
// lower CS
	CLEAR_PIN(SPI_GPIO, 1 << CS_PIN);


// send the command
	SpiHandle.Instance->DR = 0x8020;
	hall.current_function = wait_hall_command;
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
	__HAL_RCC_SPI1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

    /*##-2- Configure peripheral GPIO ##########################################*/  
    /* SPI SCK GPIO pin configuration  */
    GPIO_InitStruct.Pin       = GPIO_PIN_5;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SPI_GPIO, &GPIO_InitStruct);

    /* SPI MISO GPIO pin configuration  */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_INPUT;
    HAL_GPIO_Init(SPI_GPIO, &GPIO_InitStruct);

    /* SPI MOSI GPIO pin configuration  */
    GPIO_InitStruct.Pin = 1 << MOSI_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(SPI_GPIO, &GPIO_InitStruct);
	
	/* CS pin */
	CLEAR_PIN(SPI_GPIO, 1 << CS_PIN);
    GPIO_InitStruct.Pin       = 1 << CS_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(SPI_GPIO, &GPIO_InitStruct);
	
}

void init_hall()
{
	/*##-1- Configure the SPI peripheral #######################################*/
	/* Set the SPI parameters */
	SpiHandle.Instance               = SPI1;
	SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
	SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
	SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
	SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
	SpiHandle.Init.DataSize          = SPI_DATASIZE_16BIT;
	SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
	SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
	SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
	SpiHandle.Init.CRCPolynomial     = 7;
	SpiHandle.Init.NSS               = SPI_NSS_SOFT;
	SpiHandle.Init.Mode              = SPI_MODE_MASTER;

	HAL_SPI_Init(&SpiHandle);

	/* SPI block is enabled prior calling SPI transmit/receive functions, in order to get CLK signal properly pulled down.
       Otherwise, SPI CLK signal is not clean on this board and leads to errors during transfer */
	__HAL_SPI_ENABLE(&SpiHandle);

	hall.current_function = send_hall_command;

	
}

void handle_hall()
{
	hall.current_function();
}





