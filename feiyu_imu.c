
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


// MPU6050 driver for the Feiyu

#include "feiyu_mane.h"
#include "feiyu_imu.h"
#include "feiyu_hall.h"
#include "arm_linux.h"
#include "feiyu_uart.h"
#include "stm32f1xx_hal_i2c.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_rcc.h"
#include "arm_math.h"


imu_t imu;

#ifdef BOARD0

// must be level so it doesn't tilt when panning
// really need variable roll/pitch 
#define ROLL_OFFSET FIXED(5)
#define PITCH_OFFSET FIXED(2.5)


#define GYRO_RATIO (IMU_HZ / 50)
#define ACCEL_BANDWIDTH 16
#define GYRO_CENTER_MAX 200
#define GYRO_CENTER_TOTAL IMU_HZ
#define MAX_GYRO_DRIFT 256

#define BLEND_DOWNSAMPLE (IMU_HZ / 32)
// make it infinity to test the gyros
//#define BLEND_DOWNSAMPLE 0x7fffffff

// normal blending
#define ATTITUDE_BLEND 256
// when the gyros saturate
#define ATTITUDE_BLEND2 64
// time to use ATTITUDE_BLEND2
#define RECOVER_TIME (5 * IMU_HZ / BLEND_DOWNSAMPLE)

#define ANGLE_TO_GYRO (500 * IMU_HZ / 1000)


static int fix_gyro_angle(int gyro_angle)
{
	if(gyro_angle > 180 * ANGLE_TO_GYRO * FRACTION)
		gyro_angle -= 360 * ANGLE_TO_GYRO * FRACTION;
	else
	if(gyro_angle < -180 * ANGLE_TO_GYRO * FRACTION)
		gyro_angle += 360 * ANGLE_TO_GYRO * FRACTION;
	return gyro_angle;
}


void do_ahrs(unsigned char *imu_buffer)
{
/*
* TRACE
* print_number(&uart, fei.hall0);		
* print_number(&uart, fei.hall1);		
* print_number(&uart, fei.hall2);		
*/
	static int blink_counter = 0;

	fei.gyro_count++;
	if(fei.gyro_count >= GYRO_RATIO)
	{
		fei.gyro_count = 0;
		int accel_x = -(int16_t)((imu_buffer[6] << 8) | imu_buffer[7]);
		int accel_y = -(int16_t)((imu_buffer[2] << 8) | imu_buffer[3]);
		int accel_z = (int16_t)((imu_buffer[4] << 8) | imu_buffer[5]);

		fei.accel_x = (fei.accel_x * (FRACTION - ACCEL_BANDWIDTH) +
			accel_x * FRACTION * ACCEL_BANDWIDTH) / FRACTION;
		fei.accel_y = (fei.accel_y * (FRACTION - ACCEL_BANDWIDTH) +
			accel_y * FRACTION * ACCEL_BANDWIDTH) / FRACTION;
		fei.accel_z = (fei.accel_z * (FRACTION - ACCEL_BANDWIDTH) +
			accel_z * FRACTION * ACCEL_BANDWIDTH) / FRACTION;

// absolute angles
		if(abs_fixed(fei.accel_z) < 256)
		{
			fei.abs_roll = 0;
			fei.abs_pitch = 0;
		}
		else
		{
			fei.abs_roll = -atan2_fixed(fei.accel_x / FRACTION, fei.accel_z / FRACTION);
			fei.abs_pitch = -atan2_fixed(-fei.accel_y / FRACTION, fei.accel_z / FRACTION);
			fei.abs_roll += ROLL_OFFSET;
			fei.abs_pitch += PITCH_OFFSET;
			fei.abs_roll = fix_angle(fei.abs_roll);
			fei.abs_pitch = fix_angle(fei.abs_pitch);
		}
		
		
		
	}

	fei.gyro_x = -(int16_t)((imu_buffer[10] << 8) | imu_buffer[11]);
	fei.gyro_y = -(int16_t)((imu_buffer[14] << 8) | imu_buffer[15]);
	fei.gyro_z = -(int16_t)((imu_buffer[12] << 8) | imu_buffer[13]);

	if(fei.calibrate_imu)
	{
// blink the LED
		blink_counter++;
		if(blink_counter >= IMU_HZ / 10)
		{
			blink_counter = 0;
			send_uart(&uart, ".", 1);
		}

		fei.gyro_x_accum += fei.gyro_x;
		fei.gyro_y_accum += fei.gyro_y;
		fei.gyro_z_accum += fei.gyro_z;
		if(fei.total_gyro == 0)
		{
			fei.gyro_x_min = fei.gyro_x;
			fei.gyro_y_min = fei.gyro_y;
			fei.gyro_z_min = fei.gyro_z;
			fei.gyro_x_max = fei.gyro_x;
			fei.gyro_y_max = fei.gyro_y;
			fei.gyro_z_max = fei.gyro_z;
		}
		else
		{
			fei.gyro_x_min = MIN(fei.gyro_x, fei.gyro_x_min);
			fei.gyro_y_min = MIN(fei.gyro_y, fei.gyro_y_min);
			fei.gyro_z_min = MIN(fei.gyro_z, fei.gyro_z_min);
			fei.gyro_x_max = MAX(fei.gyro_x, fei.gyro_x_max);
			fei.gyro_y_max = MAX(fei.gyro_y, fei.gyro_y_max);
			fei.gyro_z_max = MAX(fei.gyro_z, fei.gyro_z_max);
		}
		
		fei.total_gyro++;

		if(ABS(fei.gyro_x_max - fei.gyro_x_min) > GYRO_CENTER_MAX ||
			ABS(fei.gyro_y_max - fei.gyro_y_min) > GYRO_CENTER_MAX ||
			ABS(fei.gyro_z_max - fei.gyro_z_min) > GYRO_CENTER_MAX)
		{
/*
 * 			TRACE2
 * 			print_text(&uart, "center too big ");
 * 			print_number(&uart, ABS(fei.gyro_x_max - fei.gyro_x_min));
 * 			print_number(&uart, ABS(fei.gyro_y_max - fei.gyro_y_min));
 * 			print_number(&uart, ABS(fei.gyro_z_max - fei.gyro_z_min));
 */

			fei.total_gyro = 0;
			fei.gyro_x_accum = 0;
			fei.gyro_y_accum = 0;
			fei.gyro_z_accum = 0;


			fei.gyro_x_min = 0;
			fei.gyro_y_min = 0;
			fei.gyro_z_min = 0;
			fei.gyro_x_max = 0;
			fei.gyro_y_max = 0;
			fei.gyro_z_max = 0;
		}
		else
		if(fei.total_gyro >= GYRO_CENTER_TOTAL)
		{
			fei.prev_gyro_x_center = fei.gyro_x_center;
			fei.prev_gyro_y_center = fei.gyro_y_center;
			fei.prev_gyro_z_center = fei.gyro_z_center;

 			fei.gyro_x_center = fei.gyro_x_accum * FRACTION / fei.total_gyro;
			fei.gyro_y_center = fei.gyro_y_accum * FRACTION / fei.total_gyro;
			fei.gyro_z_center = fei.gyro_z_accum * FRACTION / fei.total_gyro;

// test if calculation didn't drift
			print_lf(&uart);
			TRACE2
			print_text(&uart, "spread=");
			print_number(&uart, fei.gyro_x_max - fei.gyro_x_min);
			print_number(&uart, fei.gyro_y_max - fei.gyro_y_min);
			print_number(&uart, fei.gyro_z_max - fei.gyro_z_min);
			print_text(&uart, "center=");
			print_number(&uart, fei.gyro_x_center);
			print_number(&uart, fei.gyro_y_center);
			print_number(&uart, fei.gyro_z_center);
			print_text(&uart, "drift=");
			print_number(&uart, fei.prev_gyro_x_center - fei.gyro_x_center);
			print_number(&uart, fei.prev_gyro_y_center - fei.gyro_y_center);
			print_number(&uart, fei.prev_gyro_z_center - fei.gyro_z_center);


			if(ABS(fei.prev_gyro_x_center) > 0 && 
				ABS(fei.prev_gyro_y_center) > 0 && 
				ABS(fei.prev_gyro_z_center) > 0 && 
				ABS(fei.prev_gyro_x_center - fei.gyro_x_center) < MAX_GYRO_DRIFT &&
				ABS(fei.prev_gyro_y_center - fei.gyro_y_center) < MAX_GYRO_DRIFT &&
				ABS(fei.prev_gyro_z_center - fei.gyro_z_center) < MAX_GYRO_DRIFT)
			{
				TRACE2
				print_text(&uart, "got center ");
				print_number(&uart, fei.gyro_x_center);
				print_number(&uart, fei.gyro_y_center);
				print_number(&uart, fei.gyro_z_center);
				fei.calibrate_imu = 0;
			}
			else


// try again
			{
				print_lf(&uart);
				fei.total_gyro = 0;
				fei.gyro_x_accum = 0;
				fei.gyro_y_accum = 0;
				fei.gyro_z_accum = 0;


				fei.gyro_x_min = 65535;
				fei.gyro_y_min = 65535;
				fei.gyro_z_min = 65535;
				fei.gyro_x_max = -65535;
				fei.gyro_y_max = -65535;
				fei.gyro_z_max = -65535;
			}
		}

// predict gyro accumulation
		fei.current_roll = fei.abs_roll;
		fei.current_pitch = fei.abs_pitch;
		fei.current_heading = 0;
		fei.gyro_roll = fei.abs_roll * ANGLE_TO_GYRO;
		fei.gyro_pitch = fei.abs_pitch * ANGLE_TO_GYRO;
		fei.gyro_heading = 0;
	}
	else
// not calibrating
	{
// only use integer part to avoid saturation
		fei.gyro_roll += (fei.gyro_x * FRACTION - fei.gyro_x_center) / FRACTION;
		fei.gyro_pitch += (fei.gyro_y * FRACTION - fei.gyro_y_center) / FRACTION;
		fei.gyro_heading += (fei.gyro_z * FRACTION - fei.gyro_z_center) / FRACTION;

		fei.gyro_roll = fix_gyro_angle(fei.gyro_roll);
		fei.gyro_pitch = fix_gyro_angle(fei.gyro_pitch);
		fei.gyro_heading = fix_gyro_angle(fei.gyro_heading);
		
		
// detect overload
		if(ABS(fei.gyro_x) >= 32767 ||
			ABS(fei.gyro_y) >= 32767 ||
			ABS(fei.gyro_z) >= 32767)
		{
			imu.recover_count = RECOVER_TIME;
		}

		fei.blend_counter++;
		if(fei.blend_counter >= BLEND_DOWNSAMPLE)
		{
			int blend_factor = ATTITUDE_BLEND;
			if(imu.recover_count > 0)
			{
				blend_factor = ATTITUDE_BLEND2;
				imu.recover_count--;
			}

 			fei.blend_counter = 0;
			int error = get_angle_change_fixed(fei.gyro_roll / ANGLE_TO_GYRO, 
				fei.abs_roll);
			fei.gyro_roll += error * ANGLE_TO_GYRO / blend_factor;

			error = get_angle_change_fixed(fei.gyro_pitch / ANGLE_TO_GYRO, 
				fei.abs_pitch);
			fei.gyro_pitch += error * ANGLE_TO_GYRO / blend_factor;
		}


		fei.current_roll = fei.gyro_roll / ANGLE_TO_GYRO;
		fei.current_pitch = fei.gyro_pitch / ANGLE_TO_GYRO;
		fei.current_heading = fei.gyro_heading / ANGLE_TO_GYRO;
		fei.current_roll = fix_angle(fei.current_roll);
		fei.current_pitch = fix_angle(fei.current_pitch);
		fei.current_heading = fix_angle(fei.current_heading);

// debug
	fei.imu_count++;

//	if(mane_time - fei.debug_time > HZ / 10)
//	if(mane_time - fei.debug_time >= HZ)
//	{
//		TRACE
//		print_number(&uart, fei.gyro_x);
//		print_number(&uart, fei.gyro_y);
//		print_number(&uart, fei.gyro_z);

//		print_fixed(&uart, fei.accel_x);
//		print_fixed(&uart, fei.accel_y);
//		print_fixed(&uart, fei.accel_z);

//		print_fixed(&uart, fei.abs_roll);
//		print_fixed(&uart, fei.abs_pitch);

//		print_fixed(&uart, fei.current_roll);
//	    print_fixed(&uart, fei.current_pitch);
//		print_fixed(&uart, fei.current_heading);

//		print_fixed(&uart, fei.current_roll - fei.abs_roll);
//	    print_fixed(&uart, fei.current_pitch - fei.abs_pitch);

//		print_text(&uart, "fei.imu_count=");
//	    print_number(&uart, fei.imu_count);

//		fei.debug_time = mane_time;
//		fei.imu_count = 0;
//	}
	}

}



#endif // BOARD0







#ifdef BOARD2


#define I2C_ADDRESS (0x68 << 1)

I2C_HandleTypeDef I2cHandle;









// returns 1 when the address flag is set
int wait_address_flag()
{
// address sent
	if(__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_ADDR))
	{
//TRACE
		  /* Clear ADDR flag */
	  	__HAL_I2C_CLEAR_ADDRFLAG(&I2cHandle);

		return 1;
	}
	else
	{
// ACK failure
		if(__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_AF))
		{
TRACE
print_text(&uart, "ACK failure");
    		/* Generate Stop */
    		SET_BIT(I2cHandle.Instance->CR1, I2C_CR1_STOP);

    		/* Clear AF Flag */
    		__HAL_I2C_CLEAR_FLAG(&I2cHandle, I2C_FLAG_AF);
			imu.error = 1;
			return 1;
		}
	}
	
	return 0;
}







void i2c_read_device(unsigned char reg, int bytes)
{
	int i;
	for(i = 0; i < bytes; i++)
	{
		imu.i2c_buffer[i] = 0xff;
	}

	imu.error = 0;
	imu.reg = reg;
	imu.bytes = bytes;
	imu.current_byte = 0;

	while(__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_BUSY))
	{
	}
	
  	/* Disable Pos */
 	CLEAR_BIT(I2cHandle.Instance->CR1, I2C_CR1_POS);

	/* Enable Acknowledge */
	SET_BIT(I2cHandle.Instance->CR1, I2C_CR1_ACK);

	/* Generate Start */
	SET_BIT(I2cHandle.Instance->CR1, I2C_CR1_START);

	while(!__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_SB))
	{
	}
	
	/* Send slave address */
 	I2cHandle.Instance->DR = I2C_7BIT_ADD_WRITE(I2C_ADDRESS);
	
	while(!wait_address_flag())
	{
	}
	
	if(imu.error) return;
	
	while(!__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_TXE))
	{
	}
	
    /* Send Memory Address */
    I2cHandle.Instance->DR = I2C_MEM_ADD_LSB(imu.reg);
	
	while(!__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_TXE))
	{
	}
	
  	/* Generate Restart */
  	SET_BIT(I2cHandle.Instance->CR1, I2C_CR1_START);
	
	while(!__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_SB))
	{
	}
	
	/* Send slave address */
  	I2cHandle.Instance->DR = I2C_7BIT_ADD_READ(I2C_ADDRESS);

	while(!wait_address_flag())
	{
	}
	
	if(imu.error) return;
		
	if(imu.bytes == 1)
	{
    	/* Disable Acknowledge */
    	CLEAR_BIT(I2cHandle.Instance->CR1, I2C_CR1_ACK);

    	/* Clear ADDR flag */
    	__HAL_I2C_CLEAR_ADDRFLAG(&I2cHandle);

    	/* Generate Stop */
    	SET_BIT(I2cHandle.Instance->CR1, I2C_CR1_STOP);

	}
	else
	{
    	/* Enable Acknowledge */
    	SET_BIT(I2cHandle.Instance->CR1, I2C_CR1_ACK);

    	/* Clear ADDR flag */
    	__HAL_I2C_CLEAR_ADDRFLAG(&I2cHandle);
	}

	while(imu.current_byte < imu.bytes)
	{
		// wait for the byte
		while(!__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_RXNE))
		{
		}
		
		imu.i2c_buffer[imu.current_byte] = I2cHandle.Instance->DR;
		imu.current_byte++;
		if(imu.current_byte < imu.bytes)
		{
			/* Enable Acknowledge */
			SET_BIT(I2cHandle.Instance->CR1, I2C_CR1_ACK);
		}
		else
		{
    		/* Disable Acknowledge */
    		CLEAR_BIT(I2cHandle.Instance->CR1, I2C_CR1_ACK);
			/* Generate Stop */
    		SET_BIT(I2cHandle.Instance->CR1, I2C_CR1_STOP);
		}
	}

}







void i2c_write_device(unsigned char reg, unsigned char value)
{
	imu.error = 0;
	imu.reg = reg;
	imu.bytes = 1;
	imu.current_byte = 0;
	imu.i2c_buffer[0] = value;

	while(__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_BUSY))
	{
	}

//TRACE
	/* Disable Pos */
    CLEAR_BIT(I2cHandle.Instance->CR1, I2C_CR1_POS);
 	/* Generate Start */
 	SET_BIT(I2cHandle.Instance->CR1, I2C_CR1_START);

	while(!__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_SB))
	{
	}

//TRACE
	/* Send slave address */
  	I2cHandle.Instance->DR = I2C_7BIT_ADD_WRITE(I2C_ADDRESS);

	while(!wait_address_flag())
	{
	}
	
	if(imu.error) return;

	while(!__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_TXE))
	{
	}

    /* Send Memory Address */
    I2cHandle.Instance->DR = I2C_MEM_ADD_LSB(imu.reg);

	while(!__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_TXE))
	{
	}

	/* Write data to DR */
    I2cHandle.Instance->DR = imu.i2c_buffer[imu.current_byte];
	
	while(!__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_TXE))
	{
	}
	
	/* Generate Stop */
    SET_BIT(I2cHandle.Instance->CR1, I2C_CR1_STOP);
}




void imu_idle()
{
}

void test_status();
void send_imu_result()
{
	int i;
//TRACE

// send data to BOARD1
	int size = IMU_PACKET_SIZE;
	unsigned char buffer[size];
	buffer[0] = 0xff;
	buffer[1] = SYNC_CODE;
	memcpy(buffer + 2, imu.i2c_buffer, 14);
	buffer[16] = hall.value & 0xff;
	buffer[17] = (hall.value >> 8) & 0xff;
	buffer[18] = 0;
	buffer[19] = 0;
	send_uart(&uart, buffer, size);
		
//		TRACE
/*
 * 		for(i = 0; i < 14; i++)
 * 		{
 * 			print_hex2(&uart, imu.i2c_buffer[i]);
 * 		}
 */
/*
 *  		int offset = 0;
 *  		print_number(&uart, READ_INT16BE(imu.i2c_buffer, offset));
 *  		print_number(&uart, READ_INT16BE(imu.i2c_buffer, offset));
 *  		print_number(&uart, READ_INT16BE(imu.i2c_buffer, offset));
 * 		offset += 2;
 *  		print_number(&uart, READ_INT16BE(imu.i2c_buffer, offset));
 *  		print_number(&uart, READ_INT16BE(imu.i2c_buffer, offset));
 *  		print_number(&uart, READ_INT16BE(imu.i2c_buffer, offset));
 */



	imu.count++;
	if(mane_time - imu.time2 >= HZ)
	{
//		TRACE
//		print_text(&uart, "imu.count=");
//		print_number(&uart, imu.count);
//		print_text(&uart, "hall.count=");
//		print_number(&uart, hall.count);
//		print_text(&uart, "imu.count2=");
//		print_number(&uart, imu.count2);
		imu.count = 0;
		imu.count2 = 0;
		hall.count = 0;
		imu.time2 = mane_time;
	}
	
	
//	imu.got_readout = 1;
//	test_status();
}


#if 0
void test_status3()
{
	i2c_read_device(0x3b, 14);
	send_imu_result();
}

void test_status2()
{
//TRACE
//print_hex2(&uart, imu.i2c_buffer[0]);
	if((imu.i2c_buffer[0] & 0x1))
	{
		imu.time = mane_time;
		imu.imu_function = test_status3;
	}
	else
	{
		test_status();
	}
}

void test_status1()
{
	i2c_read_device(0x3a, 1);
	imu.imu_function = test_status2;
}

// set the function pointer, but don't read I2C
void test_status()
{
//TRACE
	imu.time = mane_time;
	imu.imu_function = test_status1;
}

#endif // 0


// get 1 readout
void do_imu()
{
	while(1)
	{
		i2c_read_device(0x3a, 1);
		if((imu.i2c_buffer[0] & 0x1))
		{
			i2c_read_device(0x3b, 14);
			send_imu_result();
			break;
		}
	}
}

void imu_init3()
{
//TRACE
// sample rate divider
// 2khz is as fast as the UART pipeline can go
//	i2c_write_device(0x19, 3);
// 1khz
//	i2c_write_device(0x19, 6);
// minimum latency
	i2c_write_device(0x19, 0);
//	imu.imu_function = test_status;
	imu.initialized = 1;
}

void imu_init2()
{
//TRACE
// accel config accel scale
	i2c_write_device(0x1c, 0x00);
	imu.imu_function = imu_init3;
}

void imu_init1()
{
//TRACE
// gyro config gyro scale
	i2c_write_device(0x1b, 0x0);
	imu.imu_function = imu_init2;
}

void imu_init0()
{
// wait for the other boards to start
	if(mane_time - imu.time > HZ / 10)
	{
//TRACE
// sleep mode & clock source
		i2c_write_device(0x6b, 1);
		imu.imu_function = imu_init1;
	}
}



// callback from HAL_I2C_Init
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

/*##-1- Enable peripherals and GPIO Clocks #################################*/
/* Enable GPIO TX/RX clock */
	__HAL_RCC_GPIOB_CLK_ENABLE();
/* Enable I2Cx clock */
	__HAL_RCC_I2C2_CLK_ENABLE();

/*##-2- Configure peripheral GPIO ##########################################*/  
/* I2C CLK GPIO pin configuration  */
	GPIO_InitStruct.Pin       = GPIO_PIN_10;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
//	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
//	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* I2C DATA GPIO pin configuration  */
	GPIO_InitStruct.Pin       = GPIO_PIN_11;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/*##-3- Configure the NVIC for I2C ########################################*/   
/* NVIC for I2Cx */
// 	HAL_NVIC_SetPriority(I2C2_ER_IRQn, 0, 1);
// 	HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
// 	HAL_NVIC_SetPriority(I2C2_EV_IRQn, 0, 2);
// 	HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
}

void init_imu()
{
	I2cHandle.Instance             = I2C2;
// dutycycle 2 can do 923076Hz or 1Mhz
	I2cHandle.Init.ClockSpeed      = 1000000;
	I2cHandle.Init.DutyCycle       = I2C_DUTYCYCLE_2;
// dutycycle 16:9 can do 720khz or 1440000Hz
//	I2cHandle.Init.DutyCycle       = I2C_DUTYCYCLE_16_9;
	I2cHandle.Init.OwnAddress1     = I2C_ADDRESS;
	I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
	I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	I2cHandle.Init.OwnAddress2     = 0xFF;
// determine if address 0x00 is ACKed
	I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
// not used in master mode
	I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;  

	HAL_I2C_Init(&I2cHandle);

// read 0x68
//	i2c_read_device(0x75, 1);
	imu.time = mane_time;
	imu.time2 = mane_time;
	imu.imu_function = imu_init0;
//	imu.imu_function = imu_idle;
}





#else // BOARD2

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
}

#endif // !BOARD2




