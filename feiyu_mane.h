#ifndef FEIYU_MANE_H
#define FEIYU_MANE_H

#include "arm_math.h"

// pin 10/ADC12_IN0 -> RC filter connected to LMV358 OUT A. 1612 when off
// pin 11/ADC12_IN1 -> RC filter connected to LMV358 OUT B. 1639 when off
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


// program with 
// make clean;make;feiyu_program feiyu_mane.bin 
// make clean;make;feiyu_program -p 1 feiyu_mane.bin 
// make clean;make;feiyu_program -p 2 feiyu_mane.bin 


// enable a board to build the bootloader or mane program
// board 2 needs a bootloader without UART3.  
// A better way is to only enable UART3 after passthrough mode is enabled.
// yaw board
#define BOARD0
// roll board
//#define BOARD1
// pitch board
//#define BOARD2

// mane anticogging table
//#define CALIBRATE_MOTOR
//#define ANTICOGGING
//#define TEST_MOTOR

// derivative length
#define ROLL_D_SIZE 8
#define PITCH_D_SIZE 4
#define HEADING_D_SIZE 8


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

#define SLOW_BAUD 115200
#define FAST_BAUD 2000000


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
#define IMU_HZ 2000



#define SYNC_CODE 0xe5
#define MOTOR_PACKET_SIZE 10
#define IMU_PACKET_SIZE 20

extern volatile int mane_time;


// set of PID constants, but not accumulating the I
typedef struct
{
// absolute angle
	int i;
// angle rate
	int p;
// rate of rate
	int d;
	int error_limit;
	int rate_limit;
} ipd_t;





typedef struct
{
	int test_step;
	int test_period;

	int hall0; // yaw motor
	int hall1; // roll motor
	int hall2; // pitch motor


#ifdef BOARD0

// for dowmsampling the accel readings
	int gyro_count;
// for calibrating the IMU
	int total_gyro;
// total IMU packets for testing
	int imu_count;
	int debug_time;
	int accel_x;
	int accel_y;
	int accel_z;
// the raw gyro readings
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
// the centers * FRACTION
	int gyro_x_center;
	int gyro_y_center;
	int gyro_z_center;

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

	int target_roll;
	int target_pitch;
	int target_heading;

// feedback based on IMU
// heading motor on top
// effect of roll motor on roll
	ipd_t top_x;
// effect of pitch motor on pitch
	ipd_t top_y;
// effect of yaw motor on yaw
	ipd_t top_z;

// heading motor behind camera
	ipd_t back_x;
	ipd_t back_z;

// feedback based on hall effect sensors
	ipd_t top_y2;
	ipd_t top_x2;
	ipd_t top_z2;
	ipd_t back_x2;
	ipd_t back_z2;
	
	
// the phases for all 3 motors.
	int x_phase;
	int y_phase;
	int z_phase;

	derivative_t roll_accel;
	int roll_accel_data[ROLL_D_SIZE];
	derivative_t pitch_accel;
	int pitch_accel_data[PITCH_D_SIZE];
	derivative_t heading_accel;
	int heading_accel_data[HEADING_D_SIZE];
#endif // BOARD0


} feiyu_t;

extern feiyu_t fei;



#endif






