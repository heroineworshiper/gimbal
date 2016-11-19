
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

// controller for the Feiyu atmega328
// compile with make feiyu.hex
// program with make feiyu_isp


#include "feiyu.h"
#include "avr_debug.h"

// vehicle mounted board uses pure bluetooth input
//#define USE_BLUETOOTH

// use PWM to send pitch/yaw instead of UART
//#define USE_PWM

#define PWM_DISABLE_PIN 5
#define MODE_PIN 2
#define PITCH_PIN 3
#define YAW_PIN 4

#define TIMER_DEADBAND 256
#define STICK_DEADBAND 16384
#define STICK_CENTER 32768
#define SPEED_DEADBAND 256
#define MODE_ADC0 0
#define MODE_ADC1 21845
#define MODE_ADC2 43690
#define PWM_MIN 8192
#define PWM_MAX 16384
#define PWM_MID ((PWM_MIN + PWM_MAX) / 2)

// hard coded mode when USE_BLUETOOTH is off
// MIN -> heading changes based on PWM.  Pitch follows absolute PWM.
// MID -> heading follows handle.  Pitch is fixed.
// MAX -> heading follows handle.  Pitch follows absolute PWM.
#define FIXED_MODE PWM_MIN
#define FIXED_YAW_SPEED ((PWM_MAX - PWM_MIN) / 2)
#define FIXED_PITCH_SPEED ((PWM_MAX - PWM_MIN) / 800)

// ADC channel masks
#define SPEED_ADC 0
#define MODE_ADC 1
#define PITCH_ADC 6
#define YAW_ADC 7

#define ADC_COMMON_BITS ((1 << REFS0) | (1 << ADLAR))


uint8_t current_adc = 0;
#define SPEED_ADC_INDEX 0
#define MODE_ADC_INDEX  1
#define PITCH_ADC_INDEX 2
#define YAW_ADC_INDEX   3

uint16_t mode_analog = 0;
uint16_t speed_analog = 0;
uint16_t pitch_analog = 0;
uint16_t yaw_analog = 0;

uint16_t new_mode_pwm = PWM_MID;
uint16_t new_pitch_pwm = PWM_MID;
uint16_t new_yaw_pwm = PWM_MID;

uint16_t mode_pwm = PWM_MID;
uint16_t pitch_pwm = PWM_MID;
uint16_t yaw_pwm = PWM_MID;

uint8_t pwm_enabled = 0;
uint8_t pwm_cycles = 0;
uint16_t stick_counter = 0;
#define STICK_COUNT 10

// code to send in UART mode
uint8_t yaw_code = 'U';


#ifdef USE_PWM
// timer compare interrupt
ISR(TIMER1_COMPA_vect)
{
	uint16_t timer_value = TCNT1 + TIMER_DEADBAND;
	if(timer_value >= mode_pwm)
	{
		bitClear(PORTD, MODE_PIN);
	}
	
	if(timer_value >= pitch_pwm)
	{
		bitClear(PORTD, PITCH_PIN);
	}
	
	if(timer_value >= yaw_pwm)
	{
		bitClear(PORTD, YAW_PIN);
	}

// set compare to new lowest value
	uint16_t min_pwm = 0x7fff;
	if(mode_pwm > timer_value)
	{
		min_pwm = mode_pwm;
	}

	if(pitch_pwm > timer_value &&
		pitch_pwm < min_pwm)
	{
		min_pwm = pitch_pwm;
	}
	
	if(yaw_pwm > timer_value &&
		yaw_pwm < min_pwm)
	{
		min_pwm = yaw_pwm;
	}

	OCR1A = min_pwm;
}
#endif // USE_PWM



static void uart_delay()
{
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
}

// timer overflow
ISR(TIMER1_OVF_vect)
{
#ifdef USE_PWM
	pwm_cycles++;
	if(pwm_cycles >= 2)
	{
// turn pins on
		if(pwm_enabled)
		{
			bitSet(PORTD, MODE_PIN);
			bitSet(PORTD, PITCH_PIN);
			bitSet(PORTD, YAW_PIN);
		}
		pwm_cycles = 0;

// copy new values
		mode_pwm = new_mode_pwm;
		pitch_pwm = new_pitch_pwm;
		yaw_pwm = new_yaw_pwm;

// set compare to lowest value
		uint16_t min_pwm = mode_pwm;
		if(pitch_pwm < min_pwm)
		{
			min_pwm = pitch_pwm;
		}
		
		if(yaw_pwm < min_pwm)
		{
			min_pwm = yaw_pwm;
		}
		
		OCR1A = min_pwm;
	}
#else // USE_PWM

	if(pwm_enabled)
	{
		pwm_cycles++;
// slow down the repeat rate
		if(pwm_cycles >= 4)
		{
			pwm_cycles = 0;
// send RS232 byte
// start bit
			bitClear(PORTD, YAW_PIN);
			uart_delay();
// data bits
			uint8_t temp = yaw_code;
			int i;
			for(i = 0; i < 8; i++)
			{
				if(temp & 0x1)
				{
					bitSet(PORTD, YAW_PIN);
				}
				else
				{
					bitClear(PORTD, YAW_PIN);
				}
				uart_delay();
				temp >>= 1;
			}
			bitSet(PORTD, YAW_PIN);
		}
	}
#endif // !USE_PWM
}



ISR(ADC_vect)
{
	uint16_t value = ADC;

	switch(current_adc)
	{
		case SPEED_ADC_INDEX:
			speed_analog = value;
			ADMUX = ADC_COMMON_BITS | MODE_ADC;
			current_adc++;
			break;

		case MODE_ADC_INDEX:
			mode_analog = value;
			ADMUX = ADC_COMMON_BITS | PITCH_ADC;
			current_adc++;
			break;

		case PITCH_ADC_INDEX:
			pitch_analog = value;
			ADMUX = ADC_COMMON_BITS | YAW_ADC;
			current_adc++;
			break;

		case YAW_ADC_INDEX:
		default:
			yaw_analog = value;
			ADMUX = ADC_COMMON_BITS | SPEED_ADC;
			current_adc = SPEED_ADC_INDEX;


// convert user input into PWM
			if(mode_analog >= MODE_ADC2)
			{
				new_mode_pwm = PWM_MAX;
			}
			else
			if(mode_analog >= MODE_ADC1)
			{
				new_mode_pwm = PWM_MID;
			}
			else
			{
				new_mode_pwm = PWM_MIN;
			}
			
#ifndef USE_BLUETOOTH
			new_mode_pwm = FIXED_MODE;
#endif

			if(stick_counter == 0)
			{
				int16_t yaw_speed = (long)(speed_analog - SPEED_DEADBAND) * 
					PWM_MID /
					(0xfffff - SPEED_DEADBAND);

#ifndef USE_BLUETOOTH
				yaw_speed = FIXED_YAW_SPEED;
#endif


				if(yaw_speed  < 1)
				{
					yaw_speed = 1;
				}

/*
 * 				if(pitch_analog >= STICK_CENTER + STICK_DEADBAND)
 * 				{
 * 					new_pitch_pwm -= FIXED_PITCH_SPEED;
 * 					if(new_pitch_pwm < PWM_MIN)
 * 					{
 * 						new_pitch_pwm = PWM_MIN;
 * 					}
 * 				}
 * 				else
 * 				if(pitch_analog < STICK_CENTER - STICK_DEADBAND)
 * 				{
 * 					new_pitch_pwm += FIXED_PITCH_SPEED;
 * 					if(new_pitch_pwm > PWM_MAX)
 * 					{
 * 						new_pitch_pwm = PWM_MAX;
 * 					}
 * 				}
 */

// fix it
				new_pitch_pwm = PWM_MID;

				if(yaw_analog >= STICK_CENTER + STICK_DEADBAND)
				{
					new_yaw_pwm = PWM_MID + yaw_speed;
					yaw_code = '2';
				}
				else
				if(yaw_analog < STICK_CENTER - STICK_DEADBAND)
				{
					new_yaw_pwm = PWM_MID - yaw_speed;
					yaw_code = '1';
				}
				else
				{
					new_yaw_pwm = PWM_MID;
					yaw_code = 'U';
				}
			}

			stick_counter++;
			if(stick_counter >= STICK_COUNT)
			{
				stick_counter = 0;
			}

#if 0
			if(uart_used == 0)
			{
				print_text("ADC: ");
//				print_number_unsigned(mode_analog);
//				print_number_unsigned(speed_analog);
				print_number_unsigned(pitch_analog);
				print_number_unsigned(yaw_analog);
				print_text(" PWM: ");
//				print_number_unsigned(new_mode_pwm);
				print_number_unsigned(new_pitch_pwm);
				print_number_unsigned(new_yaw_pwm);
				print_text("\n");
			}
#endif
			break;
	}

// start conversion
	bitSet(ADCSRA, ADSC);
}


int main()
{
	WDTCSR = 0;


	init_serial();
	print_text("Feiyu controller\n");

// PWM disable pin
// clear bit to enable input
	bitClear(DDRB, PWM_DISABLE_PIN);
// enable pullup
	bitClear(MCUCR, PUD);
	bitSet(PORTB, PWM_DISABLE_PIN);

// mode pin
// set bit to enable output
	bitClear(DDRD, MODE_PIN);
	bitClear(PORTD, MODE_PIN);

// pitch pin
	bitClear(DDRD, PITCH_PIN);
	bitClear(PORTD, PITCH_PIN);

// yaw pin
	bitClear(DDRD, YAW_PIN);
	bitClear(PORTD, YAW_PIN);

// ADC
	ADMUX = ADC_COMMON_BITS | SPEED_ADC;

	ADCSRA = (0 << ADATE) |
		(1 << ADIE) |
		(1 << ADEN) |
		(1 << ADPS2) |
		(1 << ADPS1) |
		(1 << ADPS0);
	ADCSRB = 0;
// start conversion
	bitSet(ADCSRA, ADSC);



// enable PWM timer
	TCCR1B = (1 << CS10);
// duty cycle
	OCR1A = PWM_MID;
// enable interrupts
	TIMSK1 = (1 << TOIE1)
#ifdef USE_PWM
		 | (1 << OCIE1A);
#else
		;
#endif

	sei();

	while(1)
	{
		handle_serial();

		if(!bitRead(PINB, PWM_DISABLE_PIN) && pwm_enabled)
		{
			print_text("pwm disabled\n");
			cli();
			pwm_enabled = 0;
// disable outputs.  Must also keep the PORT bits 0 to disable the pullups
			bitClear(DDRD, MODE_PIN);
			bitClear(DDRD, PITCH_PIN);
			bitClear(DDRD, YAW_PIN);
			bitClear(PORTD, MODE_PIN);
			bitClear(PORTD, PITCH_PIN);
			bitClear(PORTD, YAW_PIN);
			sei();
		}
		else
		if(bitRead(PINB, PWM_DISABLE_PIN) && !pwm_enabled)
		{
			print_text("pwm enabled\n");
			pwm_enabled = 1;
// enable outputs
			bitSet(DDRD, MODE_PIN);
			bitSet(DDRD, PITCH_PIN);
			bitSet(DDRD, YAW_PIN);
		}
	}
}





