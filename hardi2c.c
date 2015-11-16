/*
 * I2C driver
 * Copyright (C) 2012-2013 Adam Williams <broadcast at earthling dot net>
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



#include "copter.h"
#include "hardi2c.h"

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h"
#include "uart.h"
#include "misc.h"
#include "gimbal.h"

#define I2C_TIMEOUT_MAX 100000



// wait for the bits to be set
#define WAIT_STATUS_TRUE(verbose) \
{ \
	i2c->status1 |= i2c->regs->SR1; \
	i2c->status1 |= (i2c->regs->SR2 << 16); \
	if((i2c->status1 & i2c->want_event) != i2c->want_event) \
	{ \
  	  	if ((i2c->timeout--) > 0) \
		{ \
			return; \
		} \
		else \
		if(verbose) \
		{ \
			TRACE2 \
			print_text("regs="); \
			print_hex((uint32_t)i2c->regs); \
			print_text("current_function="); \
			print_hex((uint32_t)imu2.current_function); \
			print_text("WAIT_STATUS_TRUE flags="); \
			print_hex(i2c->status1); \
			init_hardi2c(i2c, i2c->regs); \
		} \
	} \
}

// Wait for the bits to not be set
#define WAIT_STATUS_FALSE(verbose) \
{ \
	i2c->status1 = i2c->regs->SR1; \
	i2c->status1 |= (i2c->regs->SR2 << 16); \
	if((i2c->status1 & i2c->want_event) == i2c->want_event) \
	{ \
  	  	if ((i2c->timeout--) > 0) \
		{ \
			return; \
		} \
		else \
		if(verbose) \
		{ \
			TRACE2 \
			print_hex((uint32_t)i2c->regs); \
			print_text("WAIT_STATUS_FALSE flags="); \
			print_hex(i2c->status1); \
			init_hardi2c(i2c, i2c->regs); \
		} \
	} \
}



void hardi2c_idle(void *i2c)
{
}

static void i2c_write_device5(void *ptr)
{
	hardi2c_t *i2c = (hardi2c_t*)ptr;
	WAIT_STATUS_FALSE(1)

	i2c->state = hardi2c_idle;
}


static void i2c_write_device4(void *ptr)
{
	hardi2c_t *i2c = (hardi2c_t*)ptr;


	WAIT_STATUS_TRUE(1)

  	I2C_GenerateSTOP(i2c->regs, ENABLE);

#ifdef I2C_INTERRUPTS
  	i2c->got_event = 0;
  	i2c->status1 = 0;
  	i2c->want_event = I2C_FLAG_BUSY;
  	i2c->timeout = I2C_TIMEOUT_MAX;
 	i2c->state = i2c_write_device5;
#else
	i2c->state = hardi2c_idle;
#endif
}


static void i2c_write_device3(void *ptr)
{
	hardi2c_t *i2c = (hardi2c_t*)ptr;


	WAIT_STATUS_TRUE(1)
  	
	
	i2c->got_event = 0;
	i2c->status1 = 0;
	i2c->want_event = I2C_EVENT_MASTER_BYTE_TRANSMITTED;
	i2c->timeout = I2C_TIMEOUT_MAX;
	i2c->state = i2c_write_device4;
	I2C_SendData(i2c->regs, i2c->value);
}


static void i2c_write_device2(void *ptr)
{
	hardi2c_t *i2c = (hardi2c_t*)ptr;
	WAIT_STATUS_TRUE(1)
	
	i2c->got_event = 0;
	i2c->status1 = 0;
	i2c->want_event = I2C_EVENT_MASTER_BYTE_TRANSMITTED;
	i2c->timeout = I2C_TIMEOUT_MAX;
	i2c->state = i2c_write_device3;
  	I2C_SendData(i2c->regs, i2c->reg_address);
}

static void i2c_write_device1(void *ptr)
{
	hardi2c_t *i2c = (hardi2c_t*)ptr;
	WAIT_STATUS_TRUE(1)
	
	i2c->got_event = 0;
	i2c->status1 = 0;
	i2c->want_event = I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED;
	i2c->timeout = I2C_TIMEOUT_MAX;
	i2c->state = i2c_write_device2;
  	I2C_Send7bitAddress(i2c->regs, i2c->dev_address, I2C_Direction_Transmitter);
}

static void i2c_write_device0(void *ptr)
{
	hardi2c_t *i2c = (hardi2c_t*)ptr;
	WAIT_STATUS_FALSE(1)
	
	i2c->got_event = 0;
	i2c->status1 = 0;
	i2c->timeout = I2C_TIMEOUT_MAX;
	i2c->state = i2c_write_device1;
	i2c->want_event = I2C_EVENT_MASTER_MODE_SELECT;
  	I2C_GenerateSTART(i2c->regs, ENABLE);
}


void hardi2c_write_device(hardi2c_t *i2c,
	unsigned char dev_address, 
	unsigned char reg_address,
	unsigned char value)
{
	i2c->dev_address = dev_address;
	i2c->reg_address = reg_address;
	i2c->value = value;
	i2c->timeout = I2C_TIMEOUT_MAX;
	i2c->got_event = 0;
	i2c->status1 = 0;

#ifdef I2C_INTERRUPTS
	i2c->want_event = I2C_EVENT_MASTER_MODE_SELECT;
  	I2C_GenerateSTART(i2c->regs, ENABLE);
	i2c->state = i2c_write_device1;
#else
	i2c->want_event = I2C_FLAG_BUSY;
	i2c->state = i2c_write_device0;
#endif
}











static void i2c_read_device7(void *ptr)
{
	hardi2c_t *i2c = (hardi2c_t*)ptr;
	WAIT_STATUS_FALSE(1)
	
	i2c->state = hardi2c_idle;
}



static void i2c_read_device6(void *ptr)
{
	hardi2c_t *i2c = (hardi2c_t*)ptr;
	WAIT_STATUS_TRUE(1)
	
	i2c->value = I2C_ReceiveData(i2c->regs);
	if(i2c->bytes > 1)
	{
		i2c->burst[i2c->bytes_read] = i2c->value;
		if(i2c->bytes_read < i2c->bytes - 1)
		{
			I2C_AcknowledgeConfig(i2c->regs, ENABLE);
		}
		else
		{
			I2C_AcknowledgeConfig(i2c->regs, DISABLE);
		}
	}
	i2c->bytes_read++;
	
	i2c->got_event = 0;
	i2c->status1 = 0;
	i2c->want_event = I2C_EVENT_MASTER_BYTE_RECEIVED;
	if(i2c->bytes_read >= i2c->bytes)
	{
		I2C_GenerateSTOP(i2c->regs, ENABLE);

#ifdef I2C_INTERRUPTS
  		i2c->got_event = 0;
  		i2c->status1 = 0;
		i2c->state = i2c_read_device7;
  		i2c->timeout = I2C_TIMEOUT_MAX;
		i2c->want_event = I2C_FLAG_BUSY;
#else

		i2c->state = hardi2c_idle;
#endif


	}

	i2c->timeout = I2C_TIMEOUT_MAX;
}


static void i2c_read_device5(void *ptr)
{
	hardi2c_t *i2c = (hardi2c_t*)ptr;
	WAIT_STATUS_TRUE(1)
	
	if(i2c->bytes > 1)
	{
		I2C_AcknowledgeConfig(i2c->regs, ENABLE);
	}
	else
	{
		I2C_AcknowledgeConfig(i2c->regs, DISABLE);
	}

	i2c->got_event = 0;
	i2c->status1 = 0;
	i2c->want_event = I2C_EVENT_MASTER_BYTE_RECEIVED;
	i2c->timeout = I2C_TIMEOUT_MAX;
	i2c->state = i2c_read_device6;
}


static void i2c_read_device4(void *ptr)
{
	hardi2c_t *i2c = (hardi2c_t*)ptr;
	WAIT_STATUS_TRUE(1)
	
	i2c->got_event = 0;
	i2c->status1 = 0;
	i2c->want_event = I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED;
	i2c->timeout = I2C_TIMEOUT_MAX;
	i2c->state = i2c_read_device5;
 	I2C_Send7bitAddress(i2c->regs, i2c->dev_address, I2C_Direction_Receiver);
}


static void i2c_read_device3(void *ptr)
{
	hardi2c_t *i2c = (hardi2c_t*)ptr;
	WAIT_STATUS_TRUE(2)
	
	i2c->got_event = 0;
	i2c->status1 = 0;
	i2c->want_event = I2C_EVENT_MASTER_MODE_SELECT;
 	i2c->regs->SR1 |= (uint16_t)0x0400;
	i2c->timeout = I2C_TIMEOUT_MAX;
	i2c->state = i2c_read_device4;
	I2C_GenerateSTART(i2c->regs, ENABLE);
}


static void i2c_read_device2(void *ptr)
{
	hardi2c_t *i2c = (hardi2c_t*)ptr;
	WAIT_STATUS_TRUE(2)
	
	i2c->got_event = 0;
	i2c->status1 = 0;
	i2c->want_event = I2C_EVENT_MASTER_BYTE_TRANSMITTED;
	i2c->timeout = I2C_TIMEOUT_MAX;
	i2c->state = i2c_read_device3;
  	I2C_SendData(i2c->regs, (uint8_t)i2c->reg_address);
}

static void i2c_read_device1(void *ptr)
{
	hardi2c_t *i2c = (hardi2c_t*)ptr;
	WAIT_STATUS_TRUE(1)
	
	i2c->got_event = 0;
	i2c->status1 = 0;
	i2c->want_event = I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED;
	i2c->timeout = I2C_TIMEOUT_MAX;
	i2c->state = i2c_read_device2;
  	I2C_Send7bitAddress(i2c->regs, i2c->dev_address, I2C_Direction_Transmitter);
}

static void i2c_read_device0(void *ptr)
{
	hardi2c_t *i2c = (hardi2c_t*)ptr;

	WAIT_STATUS_FALSE(1)

	i2c->state = i2c_read_device1;
	i2c->timeout = I2C_TIMEOUT_MAX;
	i2c->got_event = 0;
	i2c->status1 = 0;
	i2c->want_event = I2C_EVENT_MASTER_MODE_SELECT;
	I2C_GenerateSTART(i2c->regs, ENABLE);
}


void hardi2c_read_device(hardi2c_t *i2c,
	unsigned char dev_address, 
	unsigned char reg_address)
{
	i2c->dev_address = dev_address;
	i2c->reg_address = reg_address;
	i2c->value = 0;
	i2c->timeout = I2C_TIMEOUT_MAX;
  	i2c->bytes = 1;
	i2c->bytes_read = 0;
	i2c->got_event = 0;
	i2c->status1 = 0;
#ifdef I2C_INTERRUPTS
	i2c->state = i2c_read_device1;
	i2c->want_event = I2C_EVENT_MASTER_MODE_SELECT;
	I2C_GenerateSTART(i2c->regs, ENABLE);
#else
	i2c->want_event = I2C_FLAG_BUSY;
	i2c->state = i2c_read_device0;
#endif
}

void hardi2c_read_burst(hardi2c_t *i2c,
	unsigned char dev_address, 
	unsigned char reg_address,
	int bytes)
{
	i2c->dev_address = dev_address;
	i2c->reg_address = reg_address;
	i2c->value = 0;
	i2c->timeout = I2C_TIMEOUT_MAX;
	i2c->bytes = bytes;
  	i2c->bytes_read = 0;
	i2c->got_event = 0;
	i2c->status1 = 0;
	int i;
	for(i = 0; i < bytes; i++)
		i2c->burst[i] = 0;

#ifdef I2C_INTERRUPTS
	i2c->state = i2c_read_device1;
	i2c->want_event = I2C_EVENT_MASTER_MODE_SELECT;
	I2C_GenerateSTART(i2c->regs, ENABLE);

#else
	i2c->state = i2c_read_device0;
	i2c->want_event = I2C_FLAG_BUSY;
#endif
}

#ifdef I2C_INTERRUPTS
void i2c->regs_EV_IRQHandler()
{
//TRACE2
//print_hex(global_i2c->state);
//flush_uart();
	global_i2c->state(global_i2c);
}

#endif

void init_hardi2c(hardi2c_t *i2c, I2C_TypeDef *regs)
{
	I2C_InitTypeDef  I2C_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;

	i2c->regs = regs;
	
	if(regs == I2C2)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);  
	}
	else
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_I2C3);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_I2C3);  
	}

	I2C_Cmd(i2c->regs, DISABLE);
	I2C_DeInit(i2c->regs);
	i2c->regs->SR1 = 0;
	i2c->regs->SR2 = 0;
	I2C_Cmd(i2c->regs, ENABLE);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
//	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_16_9;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0xFE;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
//	I2C_InitStructure.I2C_ClockSpeed = 100000;
	I2C_InitStructure.I2C_ClockSpeed = 170000;
//	I2C_InitStructure.I2C_ClockSpeed = 400000;
	/* Apply i2c->regs configuration after enabling it */
	I2C_Init(i2c->regs, &I2C_InitStructure);
	i2c->state = hardi2c_idle;

#ifdef I2C_INTERRUPTS
 	NVIC_SetVectorTable(NVIC_VectTab_FLASH, PROGRAM_START - 0x08000000);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = i2c->regs_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	I2C_ITConfig(i2c->regs, I2C_IT_EVT, ENABLE);
#endif

}


