#ifndef HARDI2C_H
#define HARDI2C_H


#include <stdint.h>
#include "stm32f4xx_i2c.h"

// doesn't work because I2C_FLAG_BUSY is polled
//#define I2C_INTERRUPTS

typedef struct 
{
	void (*state)(void *);
	
	I2C_TypeDef *regs;
	uint32_t dev_address;
	uint32_t reg_address;
// Value user reads & writes
	uint32_t value;
// burst output
	unsigned char burst[16];
// bytes to read in burst
	int bytes;
	int bytes_read;

	uint32_t want_event;
	int got_event;
// copy of SR1
	uint32_t status1;
	int timeout;
} hardi2c_t;



#define hardi2c_ready(i2c) ((i2c)->state == hardi2c_idle)

#ifndef I2C_INTERRUPTS
#define handle_hardi2c(i2c) \
{ \
	(i2c)->state(i2c); \
}
#else
#define handle_hardi2c(i2c) \
{ \
}
#endif



void hardi2c_write_device(hardi2c_t *i2c,
	unsigned char dev_address, 
	unsigned char reg_address,
	unsigned char value);

void hardi2c_read_device(hardi2c_t *i2c,
	unsigned char dev_address, 
	unsigned char reg_address);

void hardi2c_read_burst(hardi2c_t *i2c,
	unsigned char dev_address, 
	unsigned char reg_address,
	int bytes);


void init_hardi2c(hardi2c_t *i2c, I2C_TypeDef *regs);
void hardi2c_idle(void *i2c);




#endif


