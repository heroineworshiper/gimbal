/*
 * STM32F4 Wifi flight controller
 * Copyright (C) 2012-2013 Adam Williams <broadcast at earthling dot net>
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
 
 
// MPU9150 driver for ARM

#include "settings.h"

#if defined(PIC_USE_MPU9150) && defined(COPTER_MODE) && !defined(USE_UART_IMU)

#include "linux.h"
#include "gimbal.h"
#include "math.h"
#include "mpu9150.h"
#include "uart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"

#define MAG_ADDRESS (0xc << 1)
#define IMU_ADDRESS (0x68 << 1)
// gyro readings for each accel & mag probe
#define GYRO_RATIO 8
#define MAG_TIMEOUT_MAX 100
#define DEBUG_DOWNSAMPLE 128


static int debug_counter = 0;
// copy & paste from calibration output
static const int mag_defaults[] =
{
//	-300, -300, -300, 300, 300, 300
// motors off
//        -105, -117, -363, 146, 127, -22 
// motors on
        -112, -110, -346, 149, 162, -20 
};

static void test_status(void *ptr);


static void send_results(void *ptr)
{
	imu_t *imu = (imu_t*)ptr;
// 1080 Hz
// DEBUG
//	TOGGLE_PIN(GPIOB, GPIO_Pin_4);

	
	
	imu_send_results(imu);
	imu->current_function = test_status;

/*
 * debug_counter++;
 * if(!(debug_counter % DEBUG_DOWNSAMPLE))
 * {
 * TRACE2
 * print_fixed_nospace(imu->current_roll);
 * send_uart("\t", 1);
 * print_fixed_nospace(imu->current_pitch);
 * send_uart("\t", 1);
 * print_fixed_nospace(imu->current_heading);
 * send_uart("\t", 1);
 * print_fixed_nospace(imu->abs_roll);
 * send_uart("\t", 1);
 * print_fixed_nospace(imu->abs_pitch);
 * send_uart("\t", 1);
 * print_fixed(imu->abs_heading);
 * }
 */

}



static void mag_read(void *ptr)
{
	imu_t *imu = (imu_t*)ptr;
	int x_offset = imu->mag_x_axis * 2;
	int y_offset = imu->mag_y_axis * 2;
	int z_offset = imu->mag_z_axis * 2;


	int mag_x = imu->mag_x_sign * (int16_t)((imu->i2c.burst[x_offset + 1] << 8) | (imu->i2c.burst[x_offset]));
	int mag_y = imu->mag_y_sign * (int16_t)((imu->i2c.burst[y_offset + 1] << 8) | (imu->i2c.burst[y_offset]));
	int mag_z = imu->mag_z_sign * (int16_t)((imu->i2c.burst[z_offset + 1] << 8) | (imu->i2c.burst[z_offset]));

	imu_update_mag(imu, mag_x, mag_y, mag_z);
	

// start next conversion
	hardi2c_write_device(&imu->i2c, MAG_ADDRESS, 0x0a, 0x1);
	imu->current_function = send_results;
}

static void test_mag2(void *ptr)
{
	imu_t *imu = (imu_t*)ptr;
	if((imu->i2c.value & 0x01) || imu->mag_timeout >= MAG_TIMEOUT_MAX)
	{
// debug pin
//		TOGGLE_PIN(GPIOA, GPIO_Pin_12);
		if(imu->mag_timeout >= MAG_TIMEOUT_MAX)
		{
			TRACE2
			print_text("mag timeout");
		}
		hardi2c_read_burst(&imu->i2c, MAG_ADDRESS, 0x03, 6);
		imu->current_function = mag_read;
	}
	else
	{
//TRACE
//print_hex(imu->i2c.value);
		send_results(imu);
	}
}

static void test_mag1(void *ptr)
{
	int offset = 0;
	int x_offset;
	int y_offset;
	int z_offset;
	int gyro_x;
	int gyro_y;
	int gyro_z;

	imu_t *imu = (imu_t*)ptr;
	if(imu->gyro_count == 0)
	{
		x_offset = imu->accel_x_axis * 2;
		y_offset = imu->accel_y_axis * 2;
		z_offset = imu->accel_z_axis * 2;
		int accel_x = imu->accel_x_sign * (int16_t)((imu->i2c.burst[x_offset] << 8) | (imu->i2c.burst[x_offset + 1]));
		int accel_y = imu->accel_y_sign * (int16_t)((imu->i2c.burst[y_offset] << 8) | (imu->i2c.burst[y_offset + 1]));
		int accel_z = imu->accel_z_sign * (int16_t)((imu->i2c.burst[z_offset] << 8) | (imu->i2c.burst[z_offset + 1]));

		imu_update_accel(imu, accel_x, accel_y, accel_z);

		offset = 8;
	}

	x_offset = offset + imu->gyro_x_axis * 2;
	y_offset = offset + imu->gyro_y_axis * 2;
	z_offset = offset + imu->gyro_z_axis * 2;
	gyro_x = imu->gyro_x_sign * (int16_t)((imu->i2c.burst[x_offset] << 8) | (imu->i2c.burst[x_offset + 1]));
	gyro_y = imu->gyro_y_sign * (int16_t)((imu->i2c.burst[y_offset] << 8) | (imu->i2c.burst[y_offset + 1]));
	gyro_z = imu->gyro_z_sign * (int16_t)((imu->i2c.burst[z_offset] << 8) | (imu->i2c.burst[z_offset + 1]));

	imu_update_gyro(imu, gyro_x, gyro_y, gyro_z);

	if(imu->gyro_count == 0)
	{
		imu->mag_timeout++;
		hardi2c_read_device(&imu->i2c, MAG_ADDRESS, 0x02);
		imu->current_function = test_mag2;
	}
	else
	{
		send_results(imu);
	}

/*
 * debug_counter++;
 * if(!(debug_counter % 100))
 * {
 * TRACE2
 * print_number(gyro_x);
 * print_number(gyro_y);
 * print_number(gyro_z);
 * }
 */

	imu->gyro_count++;
	if(imu->gyro_count >= GYRO_RATIO)
		imu->gyro_count = 0;
}

static void test_status2(void *ptr)
{
	imu_t *imu = (imu_t*)ptr;
	if(imu->i2c.value & 0x01)
	{
// debug pin
//		TOGGLE_PIN(GPIOA, GPIO_Pin_3);

		if(imu->gyro_count == 0)
		{
			hardi2c_read_burst(&imu->i2c,
				IMU_ADDRESS, 
				0x3b,
				14);
		}
		else
		{
			hardi2c_read_burst(&imu->i2c,
				IMU_ADDRESS, 
				0x43,
				6);
		}

		imu->current_function = test_mag1;
	}
	else
	{
		test_status(imu);
	}
}

static void test_status(void *ptr)
{
	imu_t *imu = (imu_t*)ptr;
	hardi2c_read_device(&imu->i2c, IMU_ADDRESS, 0x3a);
	imu->current_function = test_status2;
}


static void mag_start_conversion(void *ptr)
{
// start magnetometer read
	imu_t *imu = (imu_t*)ptr;
	hardi2c_write_device(&imu->i2c, MAG_ADDRESS, 0x0a, 0x01);
	imu->current_function = test_status;
}

static void imu_config5(void *ptr)
{
// sample rate divider
// high enough to keep i2c from dropping samples
	imu_t *imu = (imu_t*)ptr;
	hardi2c_write_device(&imu->i2c, IMU_ADDRESS, 0x19, 0x6);
	imu->current_function = mag_start_conversion;
}

static void imu_config4(void *ptr)
{
// accel config
	imu_t *imu = (imu_t*)ptr;
	hardi2c_write_device(&imu->i2c, IMU_ADDRESS, 0x1c, 0x00);
	imu->current_function = imu_config5;
}



static void imu_config3(void *ptr)
{
// gyro config
// 2000 deg/sec
	imu_t *imu = (imu_t*)ptr;
	hardi2c_write_device(&imu->i2c, IMU_ADDRESS, 0x1b, 0x18);
	imu->current_function = imu_config4;
}

static void imu_config2a(void *ptr)
{
// I2C master disable
	imu_t *imu = (imu_t*)ptr;
	hardi2c_write_device(&imu->i2c, IMU_ADDRESS, 0x6a, 0x00);
	imu->current_function = imu_config3;
}

static void imu_config2(void *ptr)
{
// passthrough mode
	imu_t *imu = (imu_t*)ptr;
	hardi2c_write_device(&imu->i2c, IMU_ADDRESS, 0x37, 0x02);
	imu->current_function = imu_config2a;
}

static void imu_config(void *ptr)
{
// sleep mode & clock
	imu_t *imu = (imu_t*)ptr;
	hardi2c_write_device(&imu->i2c, IMU_ADDRESS, 0x6b, 0x01);
	imu->current_function = imu_config2;
}


void init_imu1()
{
//	TRACE2
	init_imu(&imu1);

	init_hardi2c(&imu1.i2c, I2C2);

	imu1.current_function = imu_config;
	imu1.is_target = 1;


	imu1.mag_x_axis = 0;
	imu1.mag_y_axis = 1;
	imu1.mag_z_axis = 2;
	imu1.mag_x_sign = -1;
	imu1.mag_y_sign = -1;
	imu1.mag_z_sign = 1;
	imu1.mag_x_min = mag_defaults[0];
	imu1.mag_y_min = mag_defaults[1];
	imu1.mag_z_min = mag_defaults[2];
	imu1.mag_x_max = mag_defaults[3];
	imu1.mag_y_max = mag_defaults[4];
	imu1.mag_z_max = mag_defaults[5];
	imu1.mag_bandwidth = 16;
	
	if(imu1.calibrate_mag)
	{
		imu1.mag_x_max = -0x7fffffff;
		imu1.mag_y_max = -0x7fffffff;
		imu1.mag_z_max = -0x7fffffff;
		imu1.mag_x_min = 0x7fffffff;
		imu1.mag_y_min = 0x7fffffff;
		imu1.mag_z_min = 0x7fffffff;
	}

	imu1.accel_x_axis = 1;
	imu1.accel_y_axis = 0;
	imu1.accel_z_axis = 2;
	imu1.accel_x_sign = 1;
	imu1.accel_y_sign = 1;
	imu1.accel_z_sign = -1;
	imu1.accel_bandwidth = 16;
		
	imu1.gyro_x_axis = 0;
	imu1.gyro_y_axis = 1;
	imu1.gyro_z_axis = 2;
	imu1.gyro_x_sign = 1;
	imu1.gyro_y_sign = 1;
	imu1.gyro_z_sign = 1;
	imu1.gyro_bandwidth = 256;
	
	
	imu1.compass_sign = 1;
	imu1.attitude_blend = 64;
	imu1.angle_to_gyro = 50;
	imu1.gyro_center_max = 100 * FRACTION;
	imu1.gyro_x_min = 65535;
	imu1.gyro_y_min = 65535;
	imu1.gyro_z_min = 65535;
	imu1.gyro_x_max = -65535;
	imu1.gyro_y_max = -65535;
	imu1.gyro_z_max = -65535;


// debug pins
// 	GPIO_InitTypeDef  GPIO_InitStructure;
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_12;
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
// 	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
// 	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
// 	GPIO_Init(GPIOA, &GPIO_InitStructure);

}




#endif // defined(PIC_USE_MPU9150) && defined(COPTER_MODE) && !defined(USE_UART_IMU)







