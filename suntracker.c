/*
 * Motion tracking cam
 * Copyright (C) 2016  Adam Williams <broadcast at earthling dot net>
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



// ATMega328 servo controller for the sun tracker
// pulsed the pan/tilt H bridges directly



// compile with make suntracker
// program fuses: make suntracker_fuse
// program with make suntracker_isp



#include "suntracker.h"
#include "avr_debug.h"
#include <string.h> // memcpy


// pins
#define PITCH0_PIN 0
#define PITCH1_PIN 1
#define PITCH_PORT PORTC
#define PITCH_DDR DDRC

#define DEBUG_PIN 2
#define DEBUG_PORT PORTD
#define DEBUG_DDR DDRD

#define IDLE 0
#define CURSOR1 1
#define CURSOR2 2
#define CURSOR3 3
uint8_t input_state = IDLE;
uint8_t shift_down = 0;



void do_pulse(volatile uint8_t *port, uint8_t pin)
{
	bitSet(*port, pin);
	bitSet(DEBUG_PORT, DEBUG_PIN);
// set the delay
	TCNT1 = 0xc000;
// TOV1 can be cleared by writing a logic one to its bit location.
	bitSet(TIFR1, TOV1);
// wait for the timer overflow
	while(!bitRead(TIFR1, TOV1))
	{
		asm("wdr");
		handle_serial();
	}
	bitClear(*port, pin);
	bitClear(DEBUG_PORT, DEBUG_PIN);
}



int main()
{
// this doesn't work
	asm("wdr");
	WDTCSR |= 0x18;
	WDTCSR = 0;

// verify the prescaler is 0
	CLKPR = 0x80;
	CLKPR = 0x00;

	init_serial();
	print_text("Sun tracker\n");

// set pin to enable output
	bitSet(DEBUG_DDR, DEBUG_PIN);
	bitClear(DEBUG_PORT, DEBUG_PIN);

// pitch pins
	bitClear(PITCH_PORT, PITCH0_PIN);
	bitClear(PITCH_PORT, PITCH1_PIN);

// clear bit to enable input
	bitSet(PITCH_DDR, PITCH0_PIN);
	bitSet(PITCH_DDR, PITCH1_PIN);


// enable pulse timer
	TCCR1B = (1 << CS10);
// enable interrupt
//	TIMSK1 = (1 << TOIE1);




// enable interrupts
	sei();
	
	while(1)
	{
// reset the watchdog
		asm("wdr");
		handle_serial();
//		bitToggle(DEBUG_PORT, DEBUG_PIN);
		
		if(have_uart_in)
		{
			uint8_t need_status = 0;
			have_uart_in = 0;

/*
 * print_text("main 1 ");
 * print_number(shift_down);
 * print_number(uart_in);
 * print_text("\n");
 */
			
			switch(input_state)
			{
				case IDLE:
					if(uart_in == 27)
					{
						input_state++;
						shift_down = 0;
					}
					else
					if(uart_in == 59)
					{
						input_state++;
						shift_down = 1;
					}
					break;


				case CURSOR1:
					if(uart_in == 91 && !shift_down ||
						uart_in == 50 && shift_down)
					{
						input_state++;
					}
					else
					{
						input_state = IDLE;
						shift_down = 0;
					}
					break;


				case CURSOR2:
/*
 * print_text("main 2 ");
 * print_number(shift_down);
 * print_number(uart_in);
 * print_text("\n");
 */
					if(shift_down)
					{
						if(uart_in == 65)
						{
							// cursor up fast
							need_status = 1;
						}
						else
						if(uart_in == 66)
						{
							// cursor down fast
							need_status = 1;
						}
						else
						if(uart_in == 68)
						{
							// cursor left fast
							need_status = 1;
						}
						else
						if(uart_in == 67)
						{
							// cursor right fast
							need_status = 1;
						}
					}
					else
					{
						if(uart_in == 65)
						{
							// cursor up
							need_status = 1;
							print_text("UP\n");
							do_pulse(&PITCH_PORT, PITCH0_PIN);
						}
						else
						if(uart_in == 66)
						{
							// cursor down
							need_status = 1;
							print_text("DOWN\n");
							do_pulse(&PITCH_PORT, PITCH1_PIN);
						}
						else
						if(uart_in == 68)
						{
							// cursor left
							need_status = 1;
						}
						else
						if(uart_in == 67)
						{
							// cursor right
							need_status = 1;
						}
					}
					input_state = IDLE;
					shift_down = 0;
					break;
			}
			
			if(need_status)
			{
			}
			
//			print_number(uart_in);
//			print_text("\n");
		}
		
	}
	
}








