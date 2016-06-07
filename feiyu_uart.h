#ifndef UART_H
#define UART_H

#include "feiyu_mane.h"
#include <stdint.h>
#include "stm32f1xx_hal_usart.h"

#define TRACE trace(__FILE__, __FUNCTION__, __LINE__, 1);
#define TRACE2 trace(__FILE__, __FUNCTION__, __LINE__, 0);
// must be large enough for a bootloader read/write command
#define UART_BUFFER_SIZE 1032
#define UART_INPUT_SIZE 1032


extern const char hex_table[];

typedef struct
{
	volatile int uart_offset;
	volatile int uart_size;
	volatile unsigned char uart_buffer[UART_BUFFER_SIZE];

	volatile int input_offset1;
	volatile int input_offset2;
	volatile int input_size;
	volatile unsigned char input_buffer[UART_INPUT_SIZE];

	volatile int need_lf;
} uart_t;

// connected to debugger
extern uart_t uart;

#ifndef BOARD2

// connected to next chip
extern uart_t uart2;
#endif


void init_uart();
int sprint_number(unsigned char *dst, int number, int maxlen);
void print_float(float number);
void print_fixed(uart_t *uart, int number);
void print_fixed_nospace(uart_t *uart, int number);
void print_hex8(uart_t *uart, uint32_t number);
void print_hex2(uart_t *uart, uint32_t number);
void print_hex(uart_t *uart, uint32_t number);
void print_number(uart_t *uart, int number);
void print_number_nospace(uart_t *uart, int number);
void print_buffer(uart_t *uart, unsigned char *buffer, int size);
void print_buffer16(const uint16_t *buf, int len);
void flush_uart();
void print_lf(uart_t *uart);
void trace(const char *file, const char *function, int line, int flush_it);
void send_uart(uart_t *uart, const unsigned char *text, int size);
void print_text(uart_t *uart, const char *text);
void flush_uart(uart_t *uart);
// blocking read from debug port for the bootloader
unsigned char read_char();

unsigned char uart_get_input(uart_t *uart);

//void handle_uart();


#ifndef BOARD2

	#define handle_uart() \
	{ \
		if((DEBUG_UART->SR & USART_FLAG_TC) != 0 && \
			uart.uart_offset < uart.uart_size) \
		{ \
			TOGGLE_PIN(RED_LED_GPIO, 1 << RED_LED_PIN); \
			DEBUG_UART->DR = uart.uart_buffer[uart.uart_offset++]; \
		} \
	 \
		if((PASS_UART->SR & USART_FLAG_TC) != 0 && \
			uart2.uart_offset < uart2.uart_size) \
		{ \
			PASS_UART->DR = uart2.uart_buffer[uart2.uart_offset++]; \
		} \
	}

#else // !BOARD2


	#define handle_uart() \
	{ \
		if((DEBUG_UART->SR & USART_FLAG_TC) != 0 && \
			uart.uart_offset < uart.uart_size) \
		{ \
			DEBUG_UART->DR = uart.uart_buffer[uart.uart_offset++]; \
		} \
	}

#endif // BOARD2




#define UART_EMPTY(uart) \
	((uart)->uart_offset >= (uart)->uart_size)

#define TRY_UART(x, result) \
	if((DEBUG_UART->SR & USART_FLAG_TC) != 0) \
	{ \
		DEBUG_UART->DR = (x); \
		result = 1; \
	} \
	else \
	{ \
		result = 0; \
	}


#define uart_got_input(uart) ((uart)->input_size > 0)








#endif





