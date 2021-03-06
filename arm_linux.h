
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

#ifndef ARM_LINUX_H
#define ARM_LINUX_H



#define PROGRAM_START 0x8004000



#define BIT_IS_SET(byte, bit) ((byte & (1 << bit)) ? 1 : 0)
#define BIT_IS_CLEAR(byte, bit) ((byte & (1 << bit)) ? 0 : 1)
#define SET_BIT_(byte, bit) byte |= (1 << bit);
#define CLEAR_BIT_(byte, bit) byte &= ~(1 << bit);
#define COPY_BIT(dst_byte, dst_bit, src_byte, src_bit) \
	if((src_byte & (1 << src_bit)) != 0) \
		dst_byte |= (1 << dst_bit); \
	else \
		dst_byte &= ~(1 << dst_bit);
#define CLIP(x, y, z) (((x) < (y) ? (y) : ((x) > (z) ? (z) : (x))))
#define CLAMP(x, y, z) ((x) = ((x) < (y) ? (y) : ((x) > (z) ? (z) : (x))))
#define MIN(a, b)	   (((a) < (b)) ? (a) : (b))
#define MAX(a, b)	   (((a) > (b)) ? (a) : (b))
#define ABS(a)	   (((a) > 0) ? (a) : (-(a)))

#define SET_PIN(gpio, x) gpio->BSRR = (x);
#define CLEAR_PIN(gpio, x) gpio->BSRR = ((x) << 16);

// only tests 1 pin
#define PIN_IS_SET(gpio, x) ((gpio->IDR & (x)) ? 1 : 0)
#define PIN_IS_CLEAR(gpio, x) ((gpio->IDR & (x)) ? 0 : 1)
#define TOGGLE_PIN(gpio, x) (gpio->ODR ^= (x))
#define SET_COMPARE(tim, number, value) { tim->number = value; }


#define READ_UINT16(buffer, offset) \
({ \
	offset += 2; \
	(uint16_t)(buffer[offset - 2] | (buffer[offset - 1] << 8)); \
})

#define READ_INT16(buffer, offset) \
({ \
	offset += 2; \
	(int16_t)(buffer[offset - 2] | (buffer[offset - 1] << 8)); \
})

#define READ_INT16BE(buffer, offset) \
({ \
	offset += 2; \
	(int16_t)(buffer[offset - 1] | (buffer[offset - 2] << 8)); \
})

#define SQR(x) ((x) * (x))

#ifndef M_PI
#define M_PI  3.14159f
#endif

#ifdef STM32F103xB
void bzero(void *ptr, int size);
void init_linux();
#endif

void udelay(int usec);
void mdelay(int msec);


#endif





