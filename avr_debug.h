
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

#ifndef AVR_DEBUG_H
#define AVR_DEBUG_H

extern int uart_used;
extern int have_uart_in;
extern char uart_in;

void print_text(char *text);
void print_number(int number);
void print_number_unsigned(uint16_t number);
void flush_serial();
void handle_serial();
void init_serial();
void print_byte(char c);





#endif





