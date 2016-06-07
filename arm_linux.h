#ifndef LINUX_H
#define LINUX_H



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


#endif





