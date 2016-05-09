// controller for the Feiyu

#include "feiyu.h"

#define PWM_DISABLE_PIN 5
#define MODE_PIN 2
#define PITCH_PIN 3
#define YAW_PIN 4

#define TIMER_DEADBAND 256
#define STICK_DEADBAND 256
#define STICK_CENTER 32768
#define SPEED_DEADBAND 256
#define MODE_ADC0 0
#define MODE_ADC1 21845
#define MODE_ADC2 43690
#define PWM_MIN 1024
#define PWM_MAX 2048
#define PWM_MID ((PWM_MIN + PWM_MAX) / 2)

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

uint8_t pwm_cycles = 0;
uint16_t stick_counter = 0;
#define STICK_COUNT 10

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

// timer overflow
ISR(TIMER1_OVF_vect)
{
	pwm_cycles++;
	if(pwm_cycles >= 4)
	{
// turn pins on
		if(bitRead(PORTB, PWM_DISABLE_PIN))
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
			
			if(stick_counter == 0)
			{
				int16_t scaled_speed = (long)(speed_analog - SPEED_DEADBAND) * 
					PWM_MID /
					(0xfffff - SPEED_DEADBAND);
				if(scaled_speed  < 1)
				{
					scaled_speed = 1;
				}

				if(pitch_analog >= STICK_CENTER + STICK_DEADBAND)
				{
					new_pitch_pwm = PWM_MID + scaled_speed;
				}
				else
				if(pitch_analog < STICK_CENTER - STICK_DEADBAND)
				{
					new_pitch_pwm = PWM_MID - scaled_speed;
				}
				else
				{
					new_pitch_pwm = PWM_MID;
				}

				if(yaw_analog >= STICK_CENTER + STICK_DEADBAND)
				{
					new_yaw_pwm = PWM_MID + scaled_speed;
				}
				else
				if(yaw_analog < STICK_CENTER - STICK_DEADBAND)
				{
					new_yaw_pwm = PWM_MID - scaled_speed;
				}
				else
				{
					new_yaw_pwm = PWM_MID;
				}
			}

			stick_counter++;
			if(stick_counter >= STICK_COUNT)
			{
				stick_counter = 0;
			}

			print_text("ADC: ");
			print_number_unsigned(mode_analog);
			print_number_unsigned(speed_analog);
			print_number_unsigned(pitch_analog);
			print_number_unsigned(yaw_analog);
			print_text(" PWM: ");
			print_number_unsigned(new_mode_pwm);
			print_number_unsigned(new_pitch_pwm);
			print_number_unsigned(new_yaw_pwm);
			print_text("\n");
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
	bitSet(DDRD, MODE_PIN);
	bitClear(PORTD, MODE_PIN);

// pitch pin
	bitSet(DDRD, PITCH_PIN);
	bitClear(PORTD, PITCH_PIN);

// yaw pin
	bitSet(DDRD, YAW_PIN);
	bitClear(PORTD, YAW_PIN);

// ADC
	ADMUX = ADC_COMMON_BITS | SPEED_ADC;

	ADCSRA = (0 << ADATE) |
		(0 << ADIE) |
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
	TIMSK1 = (1 << TOIE1) |
		(1 << OCIE1A);

	while(1)
	{
		if(!bitRead(PORTB, PWM_DISABLE_PIN))
		{
// disable outputs.  Must also keep the PORT bits 0 to disable the pullups
			bitClear(DDRD, MODE_PIN);
			bitClear(DDRD, PITCH_PIN);
			bitClear(DDRD, YAW_PIN);
			bitClear(PORTD, MODE_PIN);
			bitClear(PORTD, PITCH_PIN);
			bitClear(PORTD, YAW_PIN);
		}
		else
		{
// enable outputs
			bitSet(DDRD, MODE_PIN);
			bitSet(DDRD, PITCH_PIN);
			bitSet(DDRD, YAW_PIN);
		}
	}
}





