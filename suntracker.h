#ifndef SUNTRACKER_H
#define SUNTRACKER_H









#define F_CPU 16000000L

#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>


#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitToggle(value, bit) ((value) ^= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))


#define CLAMP(x, y, z) ((x) = ((x) < (y) ? (y) : ((x) > (z) ? (z) : (x))))




// the table played back by the interrupt handler
typedef struct
{
// members have to be volatile to be accessed by an interrupt handler
// time until the next interrupt
	volatile uint16_t time;
// value of the PORT register
	volatile uint8_t value;
} pwm_table_t;








#endif







