
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

// debugging for AVR



#include "feiyu.h"





#define BAUD 115200L
#define UART_SIZE 1024
char uart_buffer[UART_SIZE];
int uart_used = 0;
int uart_write_ptr = 0;
int uart_read_ptr = 0;
int have_uart_in = 0;
char uart_in = 0;

void print_text(char *text)
{
	int i;
	for(i = 0; uart_used < UART_SIZE && text[i] != 0; i++)
	{
		uart_buffer[uart_write_ptr++] = text[i];
		uart_used++;
		if(uart_write_ptr >= UART_SIZE) uart_write_ptr = 0;
	}
}

void print_number(int number)
{
	char string[8];
	char *ptr = string;
	if(number < 0)
	{
		*ptr++ = '-';
		number = -number;
	}

	if(number > 10000) *ptr++ = '0' + (number / 10000);
	if(number > 1000) *ptr++ = '0' + ((number / 1000) % 10);
	if(number > 100) *ptr++ = '0' + ((number / 100) % 10);
	if(number > 10) *ptr++ = '0' + ((number / 10) % 10);
	*ptr++ = '0' + (number % 10);
	*ptr++ = ' ';
	*ptr = 0;
	print_text(string);
}

void print_number_unsigned(uint16_t number)
{
	char string[8];
	char *ptr = string;
	if(number > 10000) *ptr++ = '0' + (number / 10000);
	if(number > 1000) *ptr++ = '0' + ((number / 1000) % 10);
	if(number > 100) *ptr++ = '0' + ((number / 100) % 10);
	if(number > 10) *ptr++ = '0' + ((number / 10) % 10);
	*ptr++ = '0' + (number % 10);
	*ptr++ = ' ';
	*ptr = 0;
	print_text(string);
}

void flush_serial()
{
	while(1)
	{
		if(bitRead(UCSR0A, UDRE0)) 
		{
			if(uart_used) 
			{ 
				bitSet(UCSR0A, UDRE0); 
				UDR0 = uart_buffer[uart_read_ptr++]; 
				if(uart_read_ptr >= UART_SIZE) uart_read_ptr = 0; 
				uart_used--; 
			} 
			else
			{
				break;
			}
		}
	}
}

void handle_serial()
{
	if(bitRead(UCSR0A, UDRE0)) 
	{
		if(uart_used) 
		{ 
			bitSet(UCSR0A, UDRE0); 
			UDR0 = uart_buffer[uart_read_ptr++]; 
			if(uart_read_ptr >= UART_SIZE) uart_read_ptr = 0; 
			uart_used--; 
		} 
	}
}

void init_serial()
{
	uint16_t baud_setting = (F_CPU / 4 / BAUD - 1) / 2;
	UBRR0H = baud_setting >> 8;
	UBRR0L = baud_setting & 0xff;
	UCSR0A = (1 << U2X0);
	UCSR0C = (1 << UCSZ01) |
		(1 << UCSZ00);
	UCSR0B = (1 << RXCIE0) |
		(1 << RXEN0) |
		(1 << TXEN0);
}


ISR(USART_RX_vect)
{
	uart_in = UDR0;
	have_uart_in = 1;
}





