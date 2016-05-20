// bootloader for the feiyu stm32f103c8 
// STM32Cube_FW_F1_V1.3.0 only defines the stm32f103xb

// build with
// make feiyu_mane.bin


// program with 
// feiyu_program feiyu_mane.bin 

#include "feiyu_mane.h"
#include "arm_linux.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_hal_rcc.h"




void main()
{
	while(1)
	{
	}

// relocate interrupt vector table
	SCB->VTOR = PROGRAM_START;

	init_linux();
	init_uart();

	__enable_irq();


	CLEAR_PIN(BLUE_LED_GPIO, 1 << BLUE_LED_PIN)
	SET_PIN(RED_LED_GPIO, 1 << RED_LED_PIN)


	while(1)
	{
		print_text("Welcome to The Feiyu\n");
		flush_uart();
		TOGGLE_PIN(BLUE_LED_GPIO, 1 << BLUE_LED_PIN);

	}
}












