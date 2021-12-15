
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




#include "settings.h"




// IDG3200 + KXTF9 driver for ARM



#ifndef USE_UART_IMU

#include "idg3200.h"
#include "linux.h"
#include "gimbal.h"
#include "math.h"
#include "uart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"


#define GYRO_ADDRESS (0x69 << 1)
#define ACCEL_ADDRESS (0xf << 1)

// gyro readings for each accel probe
#define GYRO_RATIO 8
#define DEBUG_DOWNSAMPLE 128



static int debug_counter = 0;

static void imu_gyro_status1(void *ptr);

static void send_results(void *ptr)
{
	imu_t *imu = (imu_t*)ptr;
// 1100 Hz
// DEBUG
//	TOGGLE_PIN(GPIOB, GPIO_Pin_4);

	imu_send_results(imu);
	imu->current_function = imu_gyro_status1;


/*
 * debug_counter++;
 * if(!(debug_counter % DEBUG_DOWNSAMPLE))
 * {
 * TRACE2
 * print_fixed_nospace(imu->current_roll);
 * send_uart("\t", 1);
 * print_fixed_nospace(imu->current_pitch);
 * send_uart("\t", 1);
 * print_fixed_nospace(imu->abs_roll);
 * send_uart("\t", 1);
 * print_fixed_nospace(imu->abs_pitch);
 * }
 */

}



static void imu_read_accel(void *ptr)
{
	imu_t *imu = (imu_t*)ptr;
	int offset_x = imu->accel_x_axis * 2;
	int offset_y = imu->accel_y_axis * 2;
	int offset_z = imu->accel_z_axis * 2;
	int accel_x = imu->accel_x_sign * (int16_t)((imu->i2c.burst[offset_x + 1] << 8) | (imu->i2c.burst[offset_x]));
	int accel_y = imu->accel_y_sign * (int16_t)((imu->i2c.burst[offset_y + 1] << 8) | (imu->i2c.burst[offset_y]));
	int accel_z = imu->accel_z_sign * (int16_t)((imu->i2c.burst[offset_z + 1] << 8) | (imu->i2c.burst[offset_z]));

	imu_update_accel(imu, accel_x, accel_y, accel_z);
	
	send_results(ptr);
}

static void imu_accel_status2(void *ptr)
{
	imu_t *imu = (imu_t*)ptr;
	if(imu->i2c.value & 0x10)
	{
		hardi2c_read_burst(&imu->i2c,
			ACCEL_ADDRESS, 
			0x6,
			6);
		imu->current_function = imu_read_accel;

	}
 	else
	{
		send_results(ptr);
	}
}


static void imu_accel_status1(void *ptr)
{
	imu_t *imu = (imu_t*)ptr;
	int offset_x = imu->gyro_x_axis * 2;
	int offset_y = imu->gyro_y_axis * 2;
	int offset_z = imu->gyro_z_axis * 2;
	int gyro_x = imu->gyro_x_sign * (int16_t)((imu->i2c.burst[offset_x] << 8) | (imu->i2c.burst[offset_x + 1]));
	int gyro_y = imu->gyro_y_sign * (int16_t)((imu->i2c.burst[offset_y] << 8) | (imu->i2c.burst[offset_y + 1]));
	int gyro_z = imu->gyro_z_sign * (int16_t)((imu->i2c.burst[offset_z] << 8) | (imu->i2c.burst[offset_z + 1]));

	imu_update_gyro(imu, gyro_x, gyro_y, gyro_z);


// accel status
	if(imu->gyro_count == 0)
	{
		hardi2c_read_device(&imu->i2c, ACCEL_ADDRESS, 0x18);
		imu->current_function = imu_accel_status2;
	}
	else
	{
		send_results(ptr);
	}

	imu->gyro_count++;
	if(imu->gyro_count >= GYRO_RATIO)
		imu->gyro_count = 0;
}



static void imu_gyro_status2(void *ptr)
{
	imu_t *imu = (imu_t*)ptr;
	if(imu->i2c.value & 0x01)
	{
		hardi2c_read_burst(&imu->i2c,
			GYRO_ADDRESS, 
			0x1d,
			6);
		imu->current_function = imu_accel_status1;
	}
 	else
	{
		imu_gyro_status1(ptr);
	}
}

static void imu_gyro_status1(void *ptr)
{
// gyro status
	imu_t *imu = (imu_t*)ptr;
	hardi2c_read_device(&imu->i2c, GYRO_ADDRESS, 0x1a);
	imu->current_function = imu_gyro_status2;
}

static void imu_config4(void *ptr)
{
// accel CTRL_REG1
	imu_t *imu = (imu_t*)ptr;
	hardi2c_write_device(&imu->i2c, ACCEL_ADDRESS, 0x1b, 0xe0);
	imu->current_function = imu_gyro_status1;

}

static void imu_config3b(void *ptr)
{
// DATA_CTRL_REG 
	imu_t *imu = (imu_t*)ptr;
	hardi2c_write_device(&imu->i2c, ACCEL_ADDRESS, 0x21, 0x03);
	imu->current_function = imu_config4;
}

static void imu_config3(void *ptr)
{
// accel CTRL_REG1
	imu_t *imu = (imu_t*)ptr;
	hardi2c_write_device(&imu->i2c, ACCEL_ADDRESS, 0x1b, 0x00);
	imu->current_function = imu_config3b;
	
}

static void imu_config2(void *ptr)
{
// DLPF
	imu_t *imu = (imu_t*)ptr;
	hardi2c_write_device(&imu->i2c, GYRO_ADDRESS, 0x16, 0x18);
	imu->current_function = imu_config3;
	
}

static void imu_config1(void *ptr)
{
// sample rate divider
// high enough to keep i2c from dropping samples
	imu_t *imu = (imu_t*)ptr;
	hardi2c_write_device(&imu->i2c, GYRO_ADDRESS, 0x15, 0x6);
	imu->current_function = imu_config2;
	
}


// yaw IMU
void init_imu2()
{
	init_imu(&imu2);
	init_hardi2c(&imu2.i2c, I2C3);
	imu2.current_function = imu_config1;

	imu2.accel_x_axis = 1;
	imu2.accel_y_axis = 0;
	imu2.accel_z_axis = 2;
	imu2.accel_x_sign = -1;
	imu2.accel_y_sign = 1;
	imu2.accel_z_sign = -1;
	imu2.accel_bandwidth = 16;
		
	imu2.gyro_x_axis = 0;
	imu2.gyro_y_axis = 1;
	imu2.gyro_z_axis = 2;
	imu2.gyro_x_sign = 1;
	imu2.gyro_y_sign = 1;
	imu2.gyro_z_sign = 1;
	imu2.gyro_bandwidth = 256;
	
	
	imu2.compass_sign = 1;
	imu2.attitude_blend = 64;
	imu2.angle_to_gyro = 60;
	imu2.gyro_center_max = 100 * FRACTION;
	imu2.gyro_x_min = 65535;
	imu2.gyro_y_min = 65535;
	imu2.gyro_z_min = 65535;
	imu2.gyro_x_max = -65535;
	imu2.gyro_y_max = -65535;
	imu2.gyro_z_max = -65535;

}





#ifndef PIC_USE_MPU9150


imu_t imu1;




void init_imu1()
{
	init_imu(&imu1);
	init_hardi2c(&imu1.i2c, I2C2);
	imu1.current_function = imu_config1;

	imu1.accel_x_axis = 0;
	imu1.accel_y_axis = 1;
	imu1.accel_z_axis = 2;
	imu1.accel_x_sign = -1;
	imu1.accel_y_sign = 1;
	imu1.accel_z_sign = -1;
	imu1.accel_bandwidth = 16;
		
	imu1.gyro_x_axis = 1;
	imu1.gyro_y_axis = 0;
	imu1.gyro_z_axis = 2;
	imu1.gyro_x_sign = 1;
	imu1.gyro_y_sign = -1;
	imu1.gyro_z_sign = 1;
	imu1.gyro_bandwidth = 256;
	
	
	imu1.compass_sign = 1;
	imu1.attitude_blend = 1024;
	imu1.angle_to_gyro = 60;
	imu1.gyro_center_max = 100 * FRACTION;
	imu1.gyro_x_min = 65535;
	imu1.gyro_y_min = 65535;
	imu1.gyro_z_min = 65535;
	imu1.gyro_x_max = -65535;
	imu1.gyro_y_max = -65535;
	imu1.gyro_z_max = -65535;

}





#endif


#endif //  !USE_UART_IMU





