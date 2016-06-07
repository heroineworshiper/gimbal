#ifndef FEIYU_MANE_H
#define FEIYU_MANE_H



// pin 10/ADC12_IN0 -> RC filter connected to LMV358 OUT A
// pin 11/ADC12_IN1 -> RC filter connected to LMV358 OUT B
// pin 12/ADC12_IN2 -> voltage divider between battery V+ & GND to sense battery voltage
// pin 14/ADC12_IN4 -> hall chip select
// pin 15/ADC12_IN5 -> hall clock/561 khz
// pin 16/ADC12_IN6 -> hall data/bridged to pin 17
// pin 21/UART3_TX/I2C2_SCL/TIM2_CH3 -> to next board UART1_RX
// pin 22/UART3_RX/I2C2_SDA/TIM2_CH4 -> to next board UART1_TX
// pin 26/PB13 -> 100R to N MOSFET 1 pulled to ground by 2k
// pin 27/PB14 -> 100R to N MOSFET 2 pulled to ground by 2k
// pin 28/PB15 -> 100R to N MOSFET 3 pulled to ground by 2k
// pin 29/PA8 -> 100R to P MOSFET 1 pulled to ground by 2k
// pin 30/PA9 -> 100R to P MOSFET 2 pulled to ground by 2k
// pin 31/PA10 -> 100R to P MOSFET 3 pulled to ground by 2k
// pin 34 -> SWD IO
// pin 37 -> SWD CLK
// pin 40/PB4 -> LED
// pin 41/PB5/TIM3_CH2 -> 0R on yaw board to mode PWM
// pin 42/UART1_TX/I2C1_SCL/TIM4_CH2 -> TX/pitch PWM via 100R
// pin 43/UART1_RX/I2C1_SDA/TIM4_CH1 -> RX/yaw PWM via 100R
// pin 46/PB9 -> LED




// enable a board to build the bootloader or mane program
// board 2 needs a bootloader without UART3.  
// A better way is to only enable UART3 after passthrough mode is enabled.
// yaw board
//#define BOARD0
// roll board
//#define BOARD1
// pitch board
#define BOARD2


/* Definition for USARTx Pins */
#define DEBUG_UART						USART1
#define DEBUG_TX_PIN					6
#define DEBUG_TX_GPIO_PORT  			GPIOB
#define DEBUG_RX_PIN					7
#define DEBUG_RX_GPIO_PORT  			GPIOB

#define PASS_UART						USART3
#define PASS_UART_TX_PIN                10
#define PASS_UART_RX_PIN                11
#define PASS_UART_TX_GPIO               GPIOB
#define PASS_UART_RX_GPIO               GPIOB


#define BLUE_LED_PIN 9
#define BLUE_LED_GPIO GPIOB
#define RED_LED_PIN 4
#define RED_LED_GPIO GPIOB

/* Definition for TIMx clock resources */
#define TIMx                           TIM3
#define TIMx_CLK_ENABLE()              __HAL_RCC_TIM3_CLK_ENABLE()


/* Definition for TIMx's NVIC */
#define TIMx_IRQn                      TIM3_IRQn
#define TIMx_IRQHandler                TIM3_IRQHandler

// frequency of the mane timer
#define HZ 1000
// frequency of the IMU, determined by sample rate divider
#define IMU_HZ 400



#define SYNC_CODE 0xe5

extern volatile int mane_time;

typedef struct
{
	int test_step;
	int test_period;

	int hall0;
	int hall1;
	int hall2;

	int gyro_count;
	int accel_x;
	int accel_y;
	int accel_z;
	int gyro_x;
	int gyro_y;
	int gyro_z;
	int gyro_x_accum;
	int gyro_y_accum;
	int gyro_z_accum;
	int gyro_x_min;
	int gyro_y_min;
	int gyro_z_min;
	int gyro_x_max;
	int gyro_y_max;
	int gyro_z_max;
	int prev_gyro_x_center;
	int prev_gyro_y_center;
	int prev_gyro_z_center;
	int gyro_x_center;
	int gyro_y_center;
	int gyro_z_center;
	int total_gyro;
	int calibrate_imu;
	int blend_counter;
	
	int abs_roll;
	int abs_pitch;
	int gyro_roll;
	int gyro_pitch;
	int gyro_heading;
	int current_roll;
	int current_pitch;
	int current_heading;
} feiyu_t;

extern feiyu_t fei;



#endif






