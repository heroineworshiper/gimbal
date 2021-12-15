
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
#include "linux.h"
#include "imu.h"
#include "math.h"
#include "uart.h"
#include "gimbal.h"
#include "stm32f4xx_gpio.h"


#define GYRO_CENTER_TOTAL 4096
#define BLEND_DOWNSAMPLE (imu->hz / 10)
#define CALIBRATE_MAG_DOWNSAMPLE 16
#define CALIBRATE_IMU_DOWNSAMPLE (imu->hz / 10)

static int debug_counter;


int compass_heading(imu_t *imu, int mag_x, int mag_y, int mag_z)
{
	int mag_x_center = (imu->mag_x_max + imu->mag_x_min) / 2;
	int mag_y_center = (imu->mag_y_max + imu->mag_y_min) / 2;
	int mag_z_center = (imu->mag_z_max + imu->mag_z_min) / 2;
	int mag_x_scale = (imu->mag_x_max - imu->mag_x_min) / 2;
	int mag_y_scale = (imu->mag_y_max - imu->mag_y_min) / 2;
	int mag_z_scale = (imu->mag_z_max - imu->mag_z_min) / 2;

	NEW_VECTOR(mag_input, 3);
	VECTOR_ENTRY(mag_input, 0) = mag_x;
	VECTOR_ENTRY(mag_input, 1) = mag_y;
	VECTOR_ENTRY(mag_input, 2) = mag_z;

	NEW_VECTOR(mag_vector, 3);
	VECTOR_ENTRY(mag_vector, 0) = 
		(mag_input.x - mag_x_center) * 
		256 / mag_x_scale;
	VECTOR_ENTRY(mag_vector, 1) = 
		(mag_input.y - mag_y_center) * 
		256 / mag_y_scale;
	VECTOR_ENTRY(mag_vector, 2) = 
		(mag_input.z - mag_z_center) * 
		256 / mag_z_scale;
// fudge factors
	VECTOR_ENTRY(mag_vector, 0) *= -1;
	VECTOR_ENTRY(mag_vector, 2) *= -1;



/*
 * TRACE2
 * print_number(VECTOR_ENTRY(mag_input, imu->mag_y_axis));
 * print_number(mag_y_center);
 * print_number(mag_y_scale);
 * print_fixed(VECTOR_ENTRY(mag_vector, 1));
 */


	
	NEW_MATRIX(dcm, 3, 3);
	NEW_VECTOR(euler, 3);
//	VECTOR_ENTRY(euler, 0) = imu->current_roll;
//	VECTOR_ENTRY(euler, 1) = imu->current_pitch;
	VECTOR_ENTRY(euler, 0) = imu->abs_roll;
	VECTOR_ENTRY(euler, 1) = imu->abs_pitch;
	VECTOR_ENTRY(euler, 2) = 0;
	NEW_MATRIX(dcm_transpose, 3, 3);
	NEW_VECTOR(mag_ned, 3);
	euler_dc(&dcm, &euler);

	transpose_matrix(&dcm_transpose, &dcm);
//TRACE
//print_matrix(&dcm_transpose);


//TRACE2
//print_vector(&mag_vector);

	multiply_matrix_vector(&mag_ned, &dcm_transpose, &mag_vector);
//TRACE2
//print_fixed(VECTOR_ENTRY(mag_ned, 1));
//print_fixed(VECTOR_ENTRY(mag_ned, 0));

	int result = -atan2_fixed(VECTOR_ENTRY(mag_ned, 1), VECTOR_ENTRY(mag_ned, 0));
	result *= imu->compass_sign;
	result += imu->compass_offset * 256;
//TRACE2
//print_fixed(result);
	return result;
}


static int fix_gyro_angle(imu_t *imu, int gyro_angle)
{
	if(gyro_angle > 180 * imu->angle_to_gyro * FRACTION)
		gyro_angle -= 360 * imu->angle_to_gyro * FRACTION;
	else
	if(gyro_angle < -180 * imu->angle_to_gyro * FRACTION)
		gyro_angle += 360 * imu->angle_to_gyro * FRACTION;
	return gyro_angle;
}


void imu_send_results(imu_t *imu)
{




	if(imu->got_accel)
	{
		imu->got_accel = 0;
		imu->total_accel++;

		if(abs_fixed(imu->accel_z) < 256)
		{
			imu->abs_roll = 0;
			imu->abs_pitch = 0;
		}
		else
		{
			imu->abs_roll = -atan2_fixed(imu->accel_x / 256, imu->accel_z / 256);
			imu->abs_pitch = -atan2_fixed(-imu->accel_y / 256, imu->accel_z / 256);
			imu->abs_roll = fix_angle(imu->abs_roll);
			imu->abs_pitch = fix_angle(imu->abs_pitch);
		}
	}


	if(imu->got_mag)
	{
		imu->got_mag = 0;
		imu->total_mag++;
		
		imu->abs_heading = compass_heading(imu, 
			imu->mag_x / 256,
			imu->mag_y / 256, 
			imu->mag_z / 256);
		imu->abs_heading = fix_angle(imu->abs_heading);
	}




	if(!imu->have_gyro_center && imu->gyro_center_count >= GYRO_CENTER_TOTAL)
	{

/*
 * if(!imu->have_gyro_center)
 * {
 * TRACE2
 * print_text("magnitude ");
 * print_fixed(abs_fixed(imu->gyro_x_max - imu->gyro_x_min));
 * print_fixed(abs_fixed(imu->gyro_y_max - imu->gyro_y_min));
 * print_fixed(abs_fixed(imu->gyro_z_max - imu->gyro_z_min));
 * }
 */

		if(abs_fixed(imu->gyro_x_max - imu->gyro_x_min) < imu->gyro_center_max &&
			abs_fixed(imu->gyro_y_max - imu->gyro_y_min) < imu->gyro_center_max &&
			abs_fixed(imu->gyro_z_max - imu->gyro_z_min) < imu->gyro_center_max)
		{
			imu->gyro_x_center = imu->gyro_x_accum * FRACTION / imu->gyro_center_count;
			imu->gyro_y_center = imu->gyro_y_accum * FRACTION / imu->gyro_center_count;
			imu->gyro_z_center = imu->gyro_z_accum * FRACTION / imu->gyro_center_count;
			imu->temp_center = imu->temp_accum / imu->total_temp * FRACTION;

//if(imu == &imu1)
{
TRACE2
print_text("center ");
print_fixed(imu->gyro_x_center);
print_fixed(imu->gyro_y_center);
print_fixed(imu->gyro_z_center);
print_fixed(imu->temp_center);
}


			imu->have_gyro_center++;
			
			if(imu->is_target) gimbal.target_heading = imu->abs_heading;
		}
		
		
		imu->total_temp = 0;
		imu->temp_accum = 0;

		imu->gyro_center_count = 0;
		imu->gyro_x_accum = 0;
		imu->gyro_y_accum = 0;
		imu->gyro_z_accum = 0;
		

		imu->gyro_x_min = 65535;
		imu->gyro_y_min = 65535;
		imu->gyro_z_min = 65535;
		imu->gyro_x_max = -65535;
		imu->gyro_y_max = -65535;
		imu->gyro_z_max = -65535;
		
	}

	if(imu->have_gyro_center)
	{
// offset of gyro_z * FRACTION due to temperature change
		int z_temp_offset = 0;
		
		if(imu->temp_factor_z)
		{
			z_temp_offset = (imu->temp - 
				imu->temp_center) / 
				imu->temp_factor_z;
		}
	
// only use integer part to avoid saturation
		imu->gyro_roll += (imu->gyro_x - imu->gyro_x_center) / FRACTION;
		imu->gyro_pitch += (imu->gyro_y - imu->gyro_y_center) / FRACTION;
		imu->gyro_heading += (imu->gyro_z - (imu->gyro_z_center + z_temp_offset)) / FRACTION;

		imu->gyro_roll = fix_gyro_angle(imu, imu->gyro_roll);
		imu->gyro_pitch = fix_gyro_angle(imu, imu->gyro_pitch);
		imu->gyro_heading = fix_gyro_angle(imu, imu->gyro_heading);

		imu->blend_counter++;
		if(imu->blend_counter >= BLEND_DOWNSAMPLE)
		{
 			imu->blend_counter = 0;
			int error = get_angle_change_fixed(imu->gyro_roll / imu->angle_to_gyro, 
				imu->abs_roll);
			imu->gyro_roll += error * imu->angle_to_gyro / imu->attitude_blend;

			error = get_angle_change_fixed(imu->gyro_pitch / imu->angle_to_gyro, 
				imu->abs_pitch);
			imu->gyro_pitch += error * imu->angle_to_gyro / imu->attitude_blend;

//			error = get_angle_change_fixed(imu->gyro_heading / imu->angle_to_gyro, 
//				imu->abs_heading);
//			imu->gyro_heading += error * imu->angle_to_gyro / imu->attitude_blend;
		}
	

		imu->current_roll = imu->gyro_roll / imu->angle_to_gyro;
		imu->current_pitch = imu->gyro_pitch / imu->angle_to_gyro;
		imu->current_heading = imu->gyro_heading / imu->angle_to_gyro;
		imu->current_roll = fix_angle(imu->current_roll);
		imu->current_pitch = fix_angle(imu->current_pitch);
		imu->current_heading = fix_angle(imu->current_heading);

		imu->got_ahrs = 1;
	}
	else
	{
// predict gyro accumulation
		imu->current_roll = imu->abs_roll;
		imu->current_pitch = imu->abs_pitch;
//		imu->current_heading = imu->abs_heading;
		imu->current_heading = 0;
		imu->gyro_roll = imu->current_roll * imu->angle_to_gyro;
		imu->gyro_pitch = imu->current_pitch * imu->angle_to_gyro;
		imu->gyro_heading = imu->current_heading * imu->angle_to_gyro;
	}

	
	if(imu->calibrate_imu)
	{
		imu->calibrate_counter++;
		if(imu->calibrate_counter > CALIBRATE_IMU_DOWNSAMPLE)
		{
			imu->calibrate_counter = 0;
			TRACE2
//			print_buffer(imu->packet, 14);
//			print_number_nospace(imu->temp_center / FRACTION);
//			send_uart("\t", 1);
//			print_number_nospace(imu->temp / FRACTION);
//			send_uart("\t", 1);
//			print_number_nospace(imu->mag_x / FRACTION);
//			send_uart("\t", 1);
//			print_number_nospace(imu->mag_y / FRACTION);
//			send_uart("\t", 1);
//			print_number_nospace(imu->mag_z / FRACTION);
//			send_uart("\t", 1);
			print_number_nospace(imu->accel_x / FRACTION);
			send_uart("\t", 1);
			print_number_nospace(imu->accel_y / FRACTION);
			send_uart("\t", 1);
			print_number_nospace(imu->accel_z / FRACTION);
			send_uart("\t", 1);
			print_number_nospace(imu->gyro_x / FRACTION);
			send_uart("\t", 1);
			print_number_nospace(imu->gyro_y / FRACTION);
			send_uart("\t", 1);
			print_number(imu->gyro_z / FRACTION);
		}
	}

	if(imu->dump_theta)
	{

		debug_counter++;
		if(!(debug_counter % CALIBRATE_IMU_DOWNSAMPLE))
		{
			TRACE2
			print_fixed_nospace(imu->current_roll);
			send_uart("\t", 1);
			print_fixed_nospace(imu->current_pitch);
			send_uart("\t", 1);
			print_fixed_nospace(imu->current_heading);
			send_uart("\t", 1);
			print_fixed_nospace(imu->abs_roll);
			send_uart("\t", 1);
			print_fixed_nospace(imu->abs_pitch);
			send_uart("\t", 1);
			print_fixed(imu->abs_heading);
		}
	}

}



void imu_update_mag(imu_t *imu, int mag_x, int mag_y, int mag_z)
{
	imu->mag_x = (imu->mag_x * (256 - imu->mag_bandwidth) + mag_x * 256 * imu->mag_bandwidth) / 256;
	imu->mag_y = (imu->mag_y * (256 - imu->mag_bandwidth) + mag_y * 256 * imu->mag_bandwidth) / 256;
	imu->mag_z = (imu->mag_z * (256 - imu->mag_bandwidth) + mag_z * 256 * imu->mag_bandwidth) / 256;
	imu->got_mag = 1;
	imu->mag_timeout = 0;

	if(imu->calibrate_mag)
	{
		if(imu->mag_x / 256 > imu->mag_x_max) imu->mag_x_max = imu->mag_x / 256;
		if(imu->mag_y / 256 > imu->mag_y_max) imu->mag_y_max = imu->mag_y / 256;
		if(imu->mag_z / 256 > imu->mag_z_max) imu->mag_z_max = imu->mag_z / 256;
		if(imu->mag_x / 256 < imu->mag_x_min) imu->mag_x_min = imu->mag_x / 256;
		if(imu->mag_y / 256 < imu->mag_y_min) imu->mag_y_min = imu->mag_y / 256;
		if(imu->mag_z / 256 < imu->mag_z_min) imu->mag_z_min = imu->mag_z / 256;
		
		
		imu->calibrate_counter++;
		if(imu->calibrate_counter > CALIBRATE_MAG_DOWNSAMPLE)
		{
			imu->calibrate_counter = 0;
			TRACE2
			print_text("\n\t");
			print_number_nospace(imu->mag_x_min);
			print_text(", ");
			print_number_nospace(imu->mag_y_min);
			print_text(", ");
			print_number_nospace(imu->mag_z_min);
			print_text(", ");
			print_number_nospace(imu->mag_x_max);
			print_text(", ");
			print_number_nospace(imu->mag_y_max);
			print_text(", ");
			print_number(imu->mag_z_max);
		}
	}
}

void imu_update_temp(imu_t *imu, int temp)
{
	imu->temp = (imu->temp * (256 - imu->temp_bandwidth) + temp * 256 * imu->temp_bandwidth) / 256;
	imu->total_temp++;
	imu->temp_accum += temp;
}

void imu_update_accel(imu_t *imu, int accel_x, int accel_y, int accel_z)
{
	imu->accel_x = (imu->accel_x * (256 - imu->accel_bandwidth) + accel_x * 256 * imu->accel_bandwidth) / 256;
	imu->accel_y = (imu->accel_y * (256 - imu->accel_bandwidth) + accel_y * 256 * imu->accel_bandwidth) / 256;
	imu->accel_z = (imu->accel_z * (256 - imu->accel_bandwidth) + accel_z * 256 * imu->accel_bandwidth) / 256;
	imu->got_accel = 1;
}

void imu_update_gyro(imu_t *imu, int gyro_x, int gyro_y, int gyro_z)
{
	if(gyro_x >= 32767) gyro_x = 0;
	if(gyro_y >= 32767) gyro_y = 0;
	if(gyro_z >= 32767) gyro_z = 0;

	imu->gyro_x = (imu->gyro_x * (256 - imu->gyro_bandwidth) + gyro_x * 256 * imu->gyro_bandwidth) / 256;
	imu->gyro_y = (imu->gyro_y * (256 - imu->gyro_bandwidth) + gyro_y * 256 * imu->gyro_bandwidth) / 256;
	imu->gyro_z = (imu->gyro_z * (256 - imu->gyro_bandwidth) + gyro_z * 256 * imu->gyro_bandwidth) / 256;

	imu->total_gyro++;
	imu->gyro_x_accum += gyro_x;
	imu->gyro_y_accum += gyro_y;
	imu->gyro_z_accum += gyro_z;
	imu->gyro_x_min = MIN(gyro_x, imu->gyro_x_min);
	imu->gyro_y_min = MIN(gyro_y, imu->gyro_y_min);
	imu->gyro_z_min = MIN(gyro_z, imu->gyro_z_min);
	imu->gyro_x_max = MAX(gyro_x, imu->gyro_x_max);
	imu->gyro_y_max = MAX(gyro_y, imu->gyro_y_max);
	imu->gyro_z_max = MAX(gyro_z, imu->gyro_z_max);
	imu->gyro_center_count++;

}

void imu_read_packet(imu_t *imu)
{
	int offset = 0;
	imu->have_packet = 0;

// DEBUG
	if(imu == &imu1) 
		TOGGLE_PIN(GPIOB, GPIO_Pin_4);
	else
	if(imu == &imu2) 
		TOGGLE_PIN(GPIOB, GPIO_Pin_3);

	if(imu == &imu1)
	{
		offset = 6;
	}
	else
	{
		offset = 0;
	}

	int temp = (int16_t)((imu->packet[offset] << 8) | (imu->packet[offset + 1]));
	imu_update_temp(imu, temp);

	if(imu == &imu1)
	{
		offset = 8;
	}
	else
	{
		offset = 2;
	}
	
	
	int x_offset = offset + imu->gyro_x_axis * 2;
	int y_offset = offset + imu->gyro_y_axis * 2;
	int z_offset = offset + imu->gyro_z_axis * 2;
	
	int gyro_x = imu->gyro_x_sign * (int16_t)((imu->packet[x_offset] << 8) | (imu->packet[x_offset + 1]));
	int gyro_y = imu->gyro_y_sign * (int16_t)((imu->packet[y_offset] << 8) | (imu->packet[y_offset + 1]));
	int gyro_z = imu->gyro_z_sign * (int16_t)((imu->packet[z_offset] << 8) | (imu->packet[z_offset + 1]));

	imu_update_gyro(imu, gyro_x, gyro_y, gyro_z);


	int accel_x;
	int accel_y;
	int accel_z;
	if(imu == &imu1)
	{
		offset = 0;
		x_offset = offset + imu->accel_x_axis * 2;
		y_offset = offset + imu->accel_y_axis * 2;
		z_offset = offset + imu->accel_z_axis * 2;
		accel_x = imu->accel_x_sign * (int16_t)((imu->packet[x_offset] << 8) | (imu->packet[x_offset + 1]));
		accel_y = imu->accel_y_sign * (int16_t)((imu->packet[y_offset] << 8) | (imu->packet[y_offset + 1]));
		accel_z = imu->accel_z_sign * (int16_t)((imu->packet[z_offset] << 8) | (imu->packet[z_offset + 1]));
	}
	else
	{
		offset = 8;
		x_offset = offset + imu->accel_x_axis * 2;
		y_offset = offset + imu->accel_y_axis * 2;
		z_offset = offset + imu->accel_z_axis * 2;
		accel_x = imu->accel_x_sign * (int16_t)((imu->packet[x_offset + 1] << 8) | (imu->packet[x_offset]));
		accel_y = imu->accel_y_sign * (int16_t)((imu->packet[y_offset + 1] << 8) | (imu->packet[y_offset]));
		accel_z = imu->accel_z_sign * (int16_t)((imu->packet[z_offset + 1] << 8) | (imu->packet[z_offset]));
	}
	

	imu_update_accel(imu, accel_x, accel_y, accel_z);

	imu_send_results(imu);
}

static void sync_code1(void *ptr);

static void get_data(void *ptr)
{
	imu_t *imu = (imu_t*)ptr;
	imu->packet2[imu->offset++] = imu->data;
	if(imu->offset >= PACKET_SIZE)
	{
	 	imu->have_packet = 1;
		memcpy(imu->packet, imu->packet2, PACKET_SIZE);
		imu->current_function = sync_code1;
 	}
}


static void sync_code2(void *ptr)
{
	imu_t *imu = (imu_t*)ptr;
	if(imu->data == SYNC_CODE)
	{
		imu->current_function = get_data;
		imu->offset = 0;
	}
	else
	{
		imu->current_function = sync_code1;
	}
}

static void sync_code1(void *ptr)
{
	imu_t *imu = (imu_t*)ptr;
	if(imu->data == 0xff)
	{
		imu->current_function = sync_code2;
	}
}



void init_imu(imu_t *imu)
{
	bzero(imu, sizeof(imu_t));
	imu->gyro_x_min = 65535;
	imu->gyro_y_min = 65535;
	imu->gyro_z_min = 65535;
	imu->gyro_x_max = -65535;
	imu->gyro_y_max = -65535;
	imu->gyro_z_max = -65535;
	imu->current_function = sync_code1;
	debug_counter = 0;
}


















