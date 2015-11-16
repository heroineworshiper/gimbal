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

#include "linux.h"
#include "uart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "math.h"

const char hex_table[] = 
{
	'0', '1', '2', '3', '4', '5', '6', '7', 
	'8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
};

int uart_offset;
int uart_size;
unsigned char uart_buffer[UART_BUFFER_SIZE];
// If next trace needs a linefeed
int need_lf;
int got_input_;
unsigned char input;


void init_uart()
{
	USART_InitTypeDef USART_InitStructure;
	uart_offset = 0;
	uart_size = 0;
	got_input_ = 0;

	need_lf = 1;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	
/* Connect PXx to USARTx_Tx*/
  	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);

/* Connect PXx to USARTx_Rx*/
  	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

/* Configure USART Tx as alternate function  */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  	GPIO_Init(GPIOC, &GPIO_InitStructure);

/* Configure USART Rx as alternate function  */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  	GPIO_Init(GPIOC, &GPIO_InitStructure);


	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
/* USART configuration */
  	USART_Init(USART6, &USART_InitStructure);
/* Enable USART */
  	USART_Cmd(USART6, ENABLE);
}

void send_uart(const unsigned char *text, int size)
{
	if(uart_offset >= uart_size)
	{
		uart_offset = 0;
		uart_size = 0;
	}

	int i;
	for(i = 0; i < size && uart_size < UART_BUFFER_SIZE && text[i] != 0; i++)
		uart_buffer[uart_size++] = text[i];

	if(uart_buffer[uart_size - 1] == '\n')
		need_lf = 0;
}

void print_text(const char *text)
{
	int i = 0;
	while(text[i] != 0 && i < UART_BUFFER_SIZE) i++;
	send_uart(text, i);
}

#define PRINT_DIGIT(x) \
if(number >= x || force) \
{ \
	force = 1; \
	if(ptr - dst < maxlen - 1) \
	{ \
		*ptr++ = '0' + number / x; \
	} \
	number %= x; \
}


int sprint_number(unsigned char *dst, int number, int maxlen)
{
	unsigned char *ptr = dst;
	int force = 0;

	if(number < 0)
	{
		if(ptr - dst < maxlen - 1)
		{
			*ptr++ = '-';
		}
		number = -number;
	}

	
	PRINT_DIGIT(1000000000)
	PRINT_DIGIT(100000000)
	PRINT_DIGIT(10000000)
	PRINT_DIGIT(1000000)
	PRINT_DIGIT(100000)
	PRINT_DIGIT(10000)
	PRINT_DIGIT(1000)
	PRINT_DIGIT(100)
	PRINT_DIGIT(10)
	
	if(ptr - dst < maxlen - 1)
	{
		*ptr++ = '0' + number % 10;
	}

	*ptr = 0;
	return ptr - dst;
}

void print_number_nospace(int number)
{
	char buffer[16];
	int len = sprint_number(buffer, number, sizeof(buffer));
	send_uart(buffer, len);
}

void print_number(int number)
{
	print_number_nospace(number);
	send_uart(" ", 1);
}

/*
 * void print_float(float number)
 * {
 * 	int whole = (int)number;
 * 	print_number_nospace(whole);
 * 
 * 	if(absf(number - whole) > 0)
 * 	{
 * 		print_text(".");
 * 		float remainder = absf(number - whole) * 1000000;
 * 		print_number(remainder);
 * 	}
 * 	print_text(" ");
 * }
 */

void print_fixed_nospace(int number)
{
	if(number < 0) 
	{
		print_text("-");
		number = -number;
	}
	
	print_number_nospace(number / 256);
	int fraction = abs_fixed(number % 256);
	char string[1];
	if(fraction)
	{
		send_uart(".", 1);
		fraction = fraction * 1000 / 256;
		string[0] = '0' + (fraction / 100);
		send_uart(string, 1);
		string[0] = '0' + ((fraction / 10) % 10);
		send_uart(string, 1);
		string[0] = '0' + (fraction % 10);
		send_uart(string, 1);
	}
}

void print_fixed(int number)
{
	print_fixed_nospace(number);
	send_uart(" ", 1);
}


void print_hex(uint32_t number)
{
	char buffer[10];
	int i;
	int force = 0;
	char *dst = buffer;
	
	for(i = 0; i < 8; i++)
	{
		uint8_t code = (number >> 28) & 0xf;
		
		if(code > 0 || force || i == 7)
		{
			force = 1;
			*dst++ = hex_table[code];
		}
		
		number <<= 4;
	}

	*dst++ = ' ';
	send_uart(buffer, dst - buffer);
}

void print_hex2(unsigned char number)
{
	char buffer[4];
	int i = 0;
	
//	buffer[i++] = '0';
//	buffer[i++] = 'x';
	buffer[i++] = hex_table[(number >> 4) & 0xf];
	buffer[i++] = hex_table[number & 0xf];
	send_uart(buffer, i);
}

void print_hex1(unsigned char number)
{
	char buffer[3];
	int i = 0;
	
	buffer[i++] = hex_table[number & 0xf];
	send_uart(buffer, i);
}

void print_buffer(const unsigned char *buf, int len)
{
	char buffer[UART_BUFFER_SIZE];
	int i;
	char *ptr = buffer;
	for(i = 0; i < len && ptr - buffer < UART_BUFFER_SIZE - 1; i++)
	{
		if(i > 0)
		{
			*ptr++ = ' ';
		}
		
		*ptr++ = hex_table[(buf[i] >> 4) & 0xf];
		*ptr++ = hex_table[buf[i] & 0xf];
		
		if(ptr - buffer > UART_BUFFER_SIZE / 2)
		{
			send_uart(buffer, ptr - buffer);
			ptr = buffer;
		}
	}
	
	*ptr = 0;
	send_uart(buffer, ptr - buffer);
	print_lf();
}

void print_buffer16(const uint16_t *buf, int len)
{
	char buffer[UART_BUFFER_SIZE];
	int i;
	char *ptr = buffer;
	for(i = 0; i < len && ptr - buffer < UART_BUFFER_SIZE - 1; i++)
	{
		if(i > 0)
		{
			*ptr++ = ' ';
		}
		
		*ptr++ = hex_table[(buf[i] >> 12) & 0xf];
		*ptr++ = hex_table[(buf[i] >> 8) & 0xf];
		*ptr++ = hex_table[(buf[i] >> 4) & 0xf];
		*ptr++ = hex_table[buf[i] & 0xf];
		
		if(ptr - buffer > UART_BUFFER_SIZE / 2)
		{
			send_uart(buffer, ptr - buffer);
			ptr = buffer;
		}
	}
	
	*ptr = 0;
	send_uart(buffer, ptr - buffer);
	print_lf();
}

void flush_uart()
{
	while(uart_offset < uart_size) handle_uart();
}

void print_lf()
{
	send_uart("\n", 1);
}

/*
 * void handle_uart()
 * {
 * 	if(USART_GetFlagStatus(USART6, USART_FLAG_TC) == SET &&
 * 		uart_offset < uart_size)
 * 	{
 * 		USART_SendData(USART6, uart_buffer[uart_offset++]);
 * 	}
 * 
 * 	if(USART_GetFlagStatus(USART6, USART_FLAG_RXNE) == SET)
 * 	{
 * 		got_input_ = 1;
 * 		input = USART_ReceiveData(USART6);
 * 	}
 * }
 */

int got_input()
{
	return got_input_;
}

unsigned char get_input()
{
	got_input_ = 0;
	return input;
}

unsigned char read_char()
{
	while(!got_input())
	{
		handle_uart();
	}
	return get_input();
}


void trace(const char *file, const char *function, int line, int flush_it)
{
	if(need_lf)
	{
		print_text("\n");
	}
	
	print_text(file);
	print_text(": ");
	print_text(function);
	print_text(" ");
	print_number_nospace(line);
	print_text(": ");
	if(flush_it) flush_uart();
	need_lf = 1;
}


