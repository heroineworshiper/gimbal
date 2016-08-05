#include "feiyu_mane.h"
#include "feiyu_uart.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_hal_rcc.h"
#include "arm_linux.h"

uart_t uart;
uart_t uart2;
UART_HandleTypeDef UartHandle;

#ifndef BOARD2
UART_HandleTypeDef UartHandle2;
#endif // !BOARD2


const char hex_table[] = 
{
	'0', '1', '2', '3', '4', '5', '6', '7', 
	'8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
};


void init_uart(int baud1, int baud2)
{
 	UartHandle.Instance        = DEBUG_UART;
	UartHandle.Init.BaudRate   = baud1;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits   = UART_STOPBITS_1;
	UartHandle.Init.Parity     = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode       = UART_MODE_TX_RX;
	
	HAL_UART_Init(&UartHandle);

    /* Enable the UART Data Register not empty Interrupt */
    __HAL_UART_ENABLE_IT(&UartHandle, UART_IT_RXNE);
	HAL_NVIC_EnableIRQ(USART1_IRQn);

#ifndef BOARD2

	UartHandle2.Instance        = PASS_UART;
	UartHandle2.Init.BaudRate   = baud2;
	UartHandle2.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle2.Init.StopBits   = UART_STOPBITS_1;
	UartHandle2.Init.Parity     = UART_PARITY_NONE;
	UartHandle2.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	UartHandle2.Init.Mode       = UART_MODE_TX_RX;
	
	HAL_UART_Init(&UartHandle2);


    /* Enable the UART Data Register not empty Interrupt */
    __HAL_UART_ENABLE_IT(&UartHandle2, UART_IT_RXNE);
	HAL_NVIC_EnableIRQ(USART3_IRQn);


#endif // !BOARD2
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef  GPIO_InitStruct;


  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  __HAL_RCC_AFIO_CLK_ENABLE();
// Remap UART1.  Trap for young players.
  AFIO->MAPR |= AFIO_MAPR_USART1_REMAP;

  /* Enable USARTx clock */
  __HAL_RCC_USART1_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = 1 << DEBUG_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init(DEBUG_TX_GPIO_PORT, &GPIO_InitStruct);

  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = 1 << DEBUG_RX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;

  HAL_GPIO_Init(DEBUG_RX_GPIO_PORT, &GPIO_InitStruct);




#ifndef BOARD2


// Don't remap UART3.  Trap for young players.
//  AFIO->MAPR |= AFIO_MAPR_USART3_REMAP;
  __HAL_RCC_USART3_CLK_ENABLE();

  GPIO_InitStruct.Pin = 1 << PASS_UART_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  HAL_GPIO_Init(PASS_UART_TX_GPIO, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = 1 << PASS_UART_RX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_INPUT;
  HAL_GPIO_Init(PASS_UART_RX_GPIO, &GPIO_InitStruct);
#endif // !BOARD2

}



void USART1_IRQHandler()
{
	TOGGLE_PIN(BLUE_LED_GPIO, 1 << BLUE_LED_PIN);
	if((DEBUG_UART->SR & USART_FLAG_RXNE) != 0)
	{
		uart.input_buffer[uart.input_offset1] = DEBUG_UART->DR & (uint16_t)0x01FF;
		uart.input_offset1++;
		if(uart.input_offset1 >= UART_INPUT_SIZE)
		{
			uart.input_offset1 = 0;
		}
		uart.input_size++;
	}
}

#ifndef BOARD2

void USART3_IRQHandler()
{
//	TOGGLE_PIN(BLUE_LED_GPIO, 1 << BLUE_LED_PIN);
	if((PASS_UART->SR & USART_FLAG_RXNE) != 0)
	{
		uart2.input_buffer[uart2.input_offset1] = PASS_UART->DR & (uint16_t)0x01FF;
		uart2.input_offset1++;
		if(uart2.input_offset1 >= UART_INPUT_SIZE)
		{
			uart2.input_offset1 = 0;
		}
		uart2.input_size++;
	}
}
#endif // !BOARD2



unsigned char uart_get_input(uart_t *uart)
{
	unsigned char c = uart->input_buffer[uart->input_offset2];
	uart->input_offset2++;
	if(uart->input_offset2 >= UART_INPUT_SIZE)
	{
		uart->input_offset2 = 0;
	}
	
	__disable_irq();
	uart->input_size--;
	__enable_irq();
	
	return c;
}

// blocking read from debug port for the bootloader
unsigned char read_char()
{
	while(!uart_got_input(&uart))
	{
		;
	}
	return uart_get_input(&uart);
}


void send_uart(uart_t *uart, const unsigned char *text, int size)
{
//TOGGLE_PIN(BLUE_LED_GPIO, 1 << BLUE_LED_PIN);
	if(uart->uart_offset >= uart->uart_size)
	{
		uart->uart_offset = 0;
		uart->uart_size = 0;
	}

	int i;
	for(i = 0; i < size && uart->uart_size < UART_BUFFER_SIZE; i++)
		uart->uart_buffer[uart->uart_size++] = text[i];

	if(uart->uart_buffer[uart->uart_size - 1] == '\n')
		uart->need_lf = 0;
}

void print_text(uart_t *uart, const char *text)
{
	int i = 0;
	while(text[i] != 0 && i < UART_BUFFER_SIZE) i++;
	send_uart(uart, text, i);
}

void flush_uart(uart_t *uart_ptr)
{
	while(uart_ptr->uart_offset < uart_ptr->uart_size) handle_uart();
}


void print_digit(int *number, 
	int *force, 
	unsigned char **ptr, 
	unsigned char *dst, 
	int maxlen, 
	int x)
{
	if(*number >= x || *force) 
	{ 
		*force = 1; 
		if(*ptr - dst < maxlen - 1) 
		{ 
			*(*ptr)++ = '0' + *number / x; 
		} 
		(*number) %= x; 
	}
}

int sprint_number(unsigned char *dst, int number, int maxlen)
{
	unsigned char *ptr = dst;
	int force = 0;

	if(number < 0)
	{
		if(ptr - dst < maxlen - 1)
		{
			*ptr++ = '-';
		}
		number = -number;
	}

	
	print_digit(&number, &force, &ptr, dst, maxlen, 1000000000);
	print_digit(&number, &force, &ptr, dst, maxlen, 100000000);
	print_digit(&number, &force, &ptr, dst, maxlen, 10000000);
	print_digit(&number, &force, &ptr, dst, maxlen, 1000000);
	print_digit(&number, &force, &ptr, dst, maxlen, 100000);
	print_digit(&number, &force, &ptr, dst, maxlen, 10000);
	print_digit(&number, &force, &ptr, dst, maxlen, 1000);
	print_digit(&number, &force, &ptr, dst, maxlen, 100);
	print_digit(&number, &force, &ptr, dst, maxlen, 10);
	
	if(ptr - dst < maxlen - 1)
	{
		*ptr++ = '0' + number % 10;
	}

	*ptr = 0;
	return ptr - dst;
}

void print_number_nospace(uart_t *uart, int number)
{
	char buffer[16];
	int len = sprint_number(buffer, number, sizeof(buffer));
	send_uart(uart, buffer, len);
}

void print_number(uart_t *uart, int number)
{
	print_number_nospace(uart, number);
	send_uart(uart, " ", 1);
}

void print_hex2(uart_t *uart, uint32_t number)
{
	char buffer[8];
	buffer[0] = hex_table[(number & 0xf0) >> 4];
	buffer[1] = hex_table[(number & 0xf)];
	buffer[2] = ' ';
	send_uart(uart, buffer, 3);
}

void print_fixed_nospace(uart_t *uart, int number)
{
	if(number < 0) 
	{
		print_text(uart, "-");
		number = -number;
	}
	
	print_number_nospace(uart, number / 256);
	int fraction = ABS(number % 256);
	char string[1];
	if(fraction)
	{
		send_uart(uart, ".", 1);
		fraction = fraction * 1000 / 256;
		string[0] = '0' + (fraction / 100);
		send_uart(uart, string, 1);
		string[0] = '0' + ((fraction / 10) % 10);
		send_uart(uart, string, 1);
		string[0] = '0' + (fraction % 10);
		send_uart(uart, string, 1);
	}
}

void print_fixed(uart_t *uart, int number)
{
	print_fixed_nospace(uart, number);
	send_uart(uart, " ", 1);
}



void print_lf(uart_t *uart)
{
	send_uart(uart, "\n", 1);
}

void print_buffer(uart_t *uart, unsigned char *buffer, int size)
{
	int i;
	for(i = 0; i < size; i++)
	{
		print_hex2(uart, buffer[i]);
	}
}

void print_hex8(uart_t *uart, uint32_t number)
{
	char buffer[16];
	buffer[0] = hex_table[(number >> 28) & 0xf];
	buffer[1] = hex_table[(number >> 24) & 0xf];
	buffer[2] = hex_table[(number >> 20) & 0xf];
	buffer[3] = hex_table[(number >> 16) & 0xf];
	buffer[4] = hex_table[(number >> 12) & 0xf];
	buffer[5] = hex_table[(number >> 8) & 0xf];
	buffer[6] = hex_table[(number >> 4) & 0xf];
	buffer[7] = hex_table[(number & 0xf)];
	buffer[8] = ' ';
	send_uart(uart, buffer, 9);
}

void print_hex(uart_t *uart, uint32_t number)
{
	char buffer[16];
	int i;
	int force = 0;
	char *dst = buffer;
	
	for(i = 0; i < 8; i++)
	{
		uint8_t code = (number >> 28) & 0xf;
		
		if(code > 0 || force || i == 7)
		{
			force = 1;
			*dst++ = hex_table[code];
		}
		
		number <<= 4;
	}

	*dst++ = ' ';
	send_uart(uart, buffer, dst - buffer);
}

void trace(const char *file, const char *function, int line, int flush_it)
{
	if(uart.need_lf)
	{
		print_text(&uart, "\n");
	}

	print_text(&uart, file);
	print_text(&uart, ": ");
	print_text(&uart, function);
	print_text(&uart, " ");
	print_number_nospace(&uart, line);
	print_text(&uart, ": ");
	if(flush_it) flush_uart(&uart);
	uart.need_lf = 1;
}














