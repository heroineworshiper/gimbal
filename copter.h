#ifndef COPTER_H
#define COPTER_H



#define PROGRAM_START 0x80c0000


#define SET_PIN(gpio, x) gpio->BSRRL = x;
#define CLEAR_PIN(gpio, x) gpio->BSRRH = x;
// only tests 1 pin
#define PIN_IS_SET(gpio, x) ((gpio->IDR & (x)) ? 1 : 0)
#define PIN_IS_CLEAR(gpio, x) ((gpio->IDR & (x)) ? 0 : 1)
#define TOGGLE_PIN(gpio, x) (gpio->ODR ^= (x))
#define SET_COMPARE(tim, number, value) { tim->number = value; }

#define READ_UINT16(buffer, offset) \
({ \
	offset += 2; \
	buffer[offset - 2] | (buffer[offset - 1] << 8); \
})

#define READ_INT16(buffer, offset) \
({ \
	offset += 2; \
	(int16_t)(buffer[offset - 2] | (buffer[offset - 1] << 8)); \
})

#define READ_INT32(buffer, offset) \
({ \
	offset += 4; \
	buffer[offset - 4] | (buffer[offset - 3] << 8) | (buffer[offset - 2] << 16) | (buffer[offset - 1] << 24); \
})

#define WRITE_INT32(buffer, offset, value) \
	(buffer)[(offset)++] = (value) & 0xff; \
	(buffer)[(offset)++] = ((value) >> 8) & 0xff; \
	(buffer)[(offset)++] = ((value) >> 16) & 0xff; \
	(buffer)[(offset)++] = ((value) >> 24) & 0xff;

#define WRITE_INT16(buffer, offset, value) \
	(buffer)[(offset)++] = (value) & 0xff; \
	(buffer)[(offset)++] = ((value) >> 8) & 0xff;

#endif



