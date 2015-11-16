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







#ifndef UART_H
#define UART_H

#include <stdint.h>
#include "stm32f4xx_usart.h"

#define TRACE trace(__FILE__, __FUNCTION__, __LINE__, 1);
#define TRACE2 trace(__FILE__, __FUNCTION__, __LINE__, 0);
#define UART_BUFFER_SIZE 1024


extern const char hex_table[];

extern int uart_offset;
extern int uart_size;
extern unsigned char uart_buffer[UART_BUFFER_SIZE];
extern unsigned char input;
extern int need_lf;
extern int got_input_;

void init_uart();
void send_uart(const unsigned char *text, int size);
void print_text(const char *text);
int sprint_number(unsigned char *dst, int number, int maxlen);
void print_float(float number);
void print_fixed(int number);
void print_fixed_nospace(int number);
void print_number(int number);
void print_number_nospace(int number);
void print_hex(uint32_t number);
void print_hex1(unsigned char number);
void print_hex2(unsigned char number);
void print_buffer(const unsigned char *buf, int len);
void print_buffer16(const uint16_t *buf, int len);
void flush_uart();
void print_lf();

//void handle_uart();

#define handle_uart() \
{ \
	if((USART6->SR & USART_FLAG_TC) != 0 && \
		uart_offset < uart_size) \
	{ \
		USART6->DR = uart_buffer[uart_offset++]; \
	} \
 \
	if((USART6->SR & USART_FLAG_RXNE) != 0) \
	{ \
		got_input_ = 1; \
		input = USART6->DR & (uint16_t)0x01FF; \
	} \
}

#define UART_EMPTY \
	(uart_offset >= uart_size)

#define TRY_UART(x, result) \
	if((USART6->SR & USART_FLAG_TC) != 0) \
	{ \
		USART6->DR = (x); \
		result = 1; \
	} \
	else \
	{ \
		result = 0; \
	}



void trace(const char *file, const char *function, int line, int flush_it);
int got_input();
unsigned char get_input();
// Read byte with blocking
unsigned char read_char();



#endif




