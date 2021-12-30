
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


// Short DISABLE PWM & GROUND pins to access the serial port.
// then run feiyu_program

// program with 
// make clean;make;./feiyu_program feiyu_mane.bin 
// make clean;make;./feiyu_program -p 1 feiyu_mane.bin 
// make clean;make;./feiyu_program -p 2 feiyu_mane.bin 




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
// don't provide any feedback
//#define TEST_KINEMATICS
//#define TEST_PID
// disable motors for testing.
//#define DISABLE_MOTORS
// read keyboard input instead of atmega codes
//#define USE_KEYBOARD

// derivative length
#define ROLL_D_SIZE 4
#define PITCH_D_SIZE 4
#define HEADING_D_SIZE 4


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

// software I2C
#define SOFT_I2C

// frequency of the mane timer
#define HZ 1000
// frequency of the IMU, determined by sample rate divider & CPU load
#ifdef SOFT_I2C
    #define IMU_HZ 966
#else
    #define IMU_HZ 1955
#endif
// UART packet rate
#define UART_HZ 15

// degrees per second for manual slewing
#define MANUAL_SPEED FIXED(16)

#define SYNC_CODE 0xe5
#define MOTOR_PACKET_SIZE 12
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



#define ORDER 2
typedef struct
{
	int bandwidth;
	int prev_output[ORDER];
	int prev_input[ORDER];
	int result;
} filter_t;




typedef struct
{
	int test_step;
	int test_period;
	int test_scale;

	int hall0; // yaw motor
	int hall1; // roll motor
	int hall2; // pitch motor


#ifdef BOARD0

	int debug_time;
// for dowmsampling the accel readings
	int gyro_count;
// for calibrating the IMU
	int total_gyro;
// total IMU packets for testing
	int imu_count;

// bits from the UART
// 1 if following handle
    int follow_mode;
// code from the stick
    int stick_code;
// amount of yaw change if following handle
    int yaw_command;
// if we're upside down
    int flip;
// timer for flipping
    int flip_counter;
#define FLIP_THRESHOLD 10000
#define FLIP_COUNT (IMU_HZ / 2)

// accel readings * FRACTION after lowpass filtering
	int accel_x;
	int accel_y;
	int accel_z;
// the raw gyro readings from the chip
	int gyro_x;
	int gyro_y;
	int gyro_z;
// centered, flipped raw gyro readings
	int gyro_x2;
	int gyro_y2;
	int gyro_z2;

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
// IMU angles
	int current_roll;
	int current_pitch;
	int current_heading;

// absolute angles of the motors detected by hall effect sensor
	int current_roll2;
	int current_pitch2;
    int current_yaw2;
	int handle_angle;
	int yaw_roll_fraction;
    int pitch_heading_fraction;

	int target_roll;
	int target_pitch;
	int target_heading;

// feedback based on IMU
	ipd_t roll_ipd;
	ipd_t pitch_ipd;
	ipd_t heading_ipd;

// handle at 45 deg
//	ipd_t roll_ipd45;
//	ipd_t heading_ipd45;

// handle at 90 deg
//	ipd_t roll_ipd90;
//	ipd_t heading_ipd90;

// feedback based on hall effect sensors
	ipd_t hall_pitch_ipd;
	ipd_t hall_roll_ipd;
	ipd_t hall_heading_ipd;

//	ipd_t hall_roll_ipd90;
//	ipd_t hall_heading_ipd90;


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

	filter_t roll_highpass;
	filter_t pitch_highpass;
	filter_t heading_highpass;

//	matrix_t rotation_matrix;
//	vector_t rotation_vector;
//	vector_t rotation_result;

#endif // BOARD0


} feiyu_t;

extern feiyu_t fei;



#endif






