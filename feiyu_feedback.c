
/*
 * Feiyu gimbal hack
 *
 * Copyright (C) 2016-2018 Adam Williams <broadcast at earthling dot net>
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

#include "feiyu_feedback.h"
#include "feiyu_imu.h"
#include "feiyu_mane.h"
#include "feiyu_hall.h"
#include "feiyu_motor.h"
#include "feiyu_uart.h"
#include "arm_linux.h"
#include "arm_math.h"





#ifdef BOARD0

static const unsigned char handle_table[] = {
    5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, // roll 0
    7, 11, 16, 21, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, // roll 5
    11, 14, 18, 22, 27, 31, 36, 41, 46, 51, 56, 61, 65, 70, 75, 80, 85, // roll 10
    16, 18, 21, 25, 29, 33, 38, 42, 47, 52, 56, 61, 66, 71, 76, 80, 85, // roll 15
    21, 22, 25, 28, 32, 36, 40, 44, 48, 53, 57, 62, 67, 71, 76, 81, 85, // roll 20
    25, 27, 29, 32, 35, 38, 42, 46, 50, 54, 59, 63, 67, 72, 76, 81, 85, // roll 25
    30, 31, 33, 36, 38, 41, 45, 48, 52, 56, 60, 64, 69, 73, 77, 81, 86, // roll 30
    35, 36, 38, 40, 42, 45, 48, 51, 55, 58, 62, 66, 70, 74, 78, 82, 86, // roll 35
    40, 41, 42, 44, 46, 48, 51, 54, 57, 61, 64, 67, 71, 75, 79, 82, 86, // roll 40
    45, 46, 47, 48, 50, 52, 55, 57, 60, 63, 66, 69, 73, 76, 79, 83, 86 // roll 45

};

//int16_t rotation_table[4 * 19];


int get_handle_angle()
{
	int table_row = fei.current_roll2 / 5 / FRACTION;
	if(table_row < 0) table_row *= -1;
	
	int clamped_pitch = fei.current_pitch2;
// punt
	if(clamped_pitch < FIXED(-80))
	{
		return FIXED(-80);
	}
	else
	if(clamped_pitch < 0)
	{
		clamped_pitch = -clamped_pitch;
	}
	else
	if(clamped_pitch > FIXED(90))
	{
		clamped_pitch = FIXED(180) - clamped_pitch;
	}
	
	int table_column = clamped_pitch / 5 / FRACTION - 1;

	if(table_column < 0)
	{
		return fei.current_roll2;
	}
	else
	if(table_column > 15)
	{
		return 90 * FRACTION;
	}
	
	int result = handle_table[table_row * 17 + table_column] * FRACTION;
	if(fei.current_pitch2 > FIXED(90))
	{
		result = FIXED(180) - result;
	}
	else
	if(fei.current_pitch2 < 0)
	{
		result = -result;
	}
	
	return result;
}

static void init_ipd(ipd_t *ptr, 
	int i, 
	int p, 
	int d, 
	int error_limit, 
	int rate_limit)
{
	ptr->i = i;
	ptr->p = p;
	ptr->d = d;
	ptr->error_limit = error_limit;
	ptr->rate_limit = rate_limit;
}

static void zero_ipd(ipd_t *ptr)
{
	ptr->i = 0;
	ptr->p = 0;
	ptr->d = 0;
}


static void blend_ipd(ipd_t *out, 
	ipd_t *arg1,
	ipd_t *arg2,
	int fraction)
{
	int inv_fraction = FRACTION - fraction;
	out->i = (arg1->i * inv_fraction + arg2->i * fraction) / FRACTION;
	out->p = (arg1->p * inv_fraction + arg2->p * fraction) / FRACTION;
	out->d = (arg1->d * inv_fraction + arg2->d * fraction) / FRACTION;
	out->error_limit = (arg1->error_limit * inv_fraction + arg2->error_limit * fraction) / FRACTION;
	out->rate_limit = (arg1->rate_limit * inv_fraction + arg2->rate_limit * fraction) / FRACTION;
}


static int get_step(ipd_t *table, int error, int rate, int derivative)
{
//	int limit = 0x7fffffff / table->i;
//	CLAMP(error, -limit, limit);

	int i_result = error * table->i / FRACTION;
	CLAMP(i_result, -table->error_limit, table->error_limit);

//	limit = 0x7fffffff / table->p;
//	CLAMP(rate, -limit, limit);

	int p_result = rate * table->p / FRACTION;
	CLAMP(p_result, -table->rate_limit, table->rate_limit);
	
//	limit = 0x7fffffff / table->d;
//	CLAMP(derivative, -limit, limit);

	int d_result = derivative * table->d / FRACTION;
	CLAMP(d_result, -table->rate_limit, table->rate_limit);

	int result = i_result + p_result + d_result;
	CLAMP(result, -table->rate_limit, table->rate_limit);
	
//	result = result * fei.test_scale / FRACTION;
	return result;
}


void init_filter(filter_t *ptr, int value, int bandwidth)
{
	int i;
	ptr->bandwidth = bandwidth;
	for(i = 0; i < ORDER; i++)
	{
		ptr->prev_output[i] = value;
		ptr->prev_input[i] = value;
	}
}

int do_highpass(filter_t *ptr, int value)
{
	int i;
	int result = value;
	for(i = 0; i < ORDER; i++)
	{
		result = ptr->bandwidth * 
			(ptr->prev_output[i] + value - ptr->prev_input[i]) / 
			FRACTION;
		ptr->prev_input[i] = value;
		ptr->prev_output[i] = result;
		value = result;
	}
	
	return result;
}



// motor feedback is on board 0 only
void init_feedback()
{
	init_derivative(&fei.roll_accel, ROLL_D_SIZE);
	init_derivative(&fei.pitch_accel, PITCH_D_SIZE);
	init_derivative(&fei.heading_accel, HEADING_D_SIZE);
	
	
	init_filter(&fei.roll_highpass, 0, FIXED(0.90));
	init_filter(&fei.pitch_highpass, 0, FIXED(0.90));
	init_filter(&fei.heading_highpass, 0, FIXED(0.90));
	
	
// init rotation table
	int i;
// 	for(i = 0; i <= 90; i += 5)
// 	{
// 		int index = i / 5;
// 		index *= 4;
// 		int cos_angle = cos_fixed(i * FRACTION);
// 		int sin_angle = sin_fixed(i * FRACTION);
// 		
// 		rotation_table[index + 0] = cos_angle;
// 		rotation_table[index + 1] = sin_angle;
// 		rotation_table[index + 2] = -sin_angle;
// 		rotation_table[index + 3] = cos_angle;
// /*
//  * 		TRACE
//  * 		print_fixed(&uart, rotation_table[index * 4 + 0]);
//  * 		print_fixed(&uart, rotation_table[index * 4 + 1]);
//  * 		print_fixed(&uart, rotation_table[index * 4 + 2]);
//  * 		print_fixed(&uart, rotation_table[index * 4 + 3]);
//  */
// 	}
	
// 	INIT_MATRIX(fei.rotation_matrix, 3, 3);
// 	MATRIX_ENTRY(fei.rotation_matrix, 0, 0) = FIXED(1);
// 	MATRIX_ENTRY(fei.rotation_matrix, 0, 1) = 0;
// 	MATRIX_ENTRY(fei.rotation_matrix, 0, 2) = 0;
// 	MATRIX_ENTRY(fei.rotation_matrix, 1, 0) = 0;
// 	MATRIX_ENTRY(fei.rotation_matrix, 2, 0) = 0;
	
//	INIT_VECTOR(fei.rotation_vector, 3);
//	INIT_VECTOR(fei.rotation_result, 3);
	
// feedback using IMU
	init_ipd(&fei.pitch_ipd,  
		FIXED(1),   // I as high as possible
		FIXED(0.082),   // P as high as possible
		FIXED(0.250),   // D/highpass as high as possible
		FIXED(255),   // error limit
		FIXED(255));  // rate limit
	


	init_ipd(&fei.roll_ipd,  
		FIXED(1),   // I
		FIXED(0.25),   // P
		FIXED(1.0),   // D/highpass
		FIXED(255), 
		FIXED(255));
	init_ipd(&fei.heading_ipd, 
		FIXED(1),  // I
		FIXED(0.5),  // P
		FIXED(1.0),  // D/highpass
		FIXED(255),  // error limit
		FIXED(255)); // rate limit





// feedback using hall sensors
	init_ipd(&fei.hall_pitch_ipd, 
		FIXED(10),  // I
		FIXED(2),  // P
		0,  // D
		FIXED(60),  // error limit
		FIXED(60)); // rate limit
	init_ipd(&fei.hall_roll_ipd, 
		FIXED(10),  // I
		FIXED(2),  // P
		0,  // D
		FIXED(60),  // error limit
		FIXED(60)); // rate limit
	init_ipd(&fei.hall_heading_ipd, 
		FIXED(10),  // I
		FIXED(2),  // P
		0,  // D
		FIXED(60),  // error limit
		FIXED(60)); // rate limit
	

// DEBUG
// lock all motors except the one under test
//zero_ipd(&fei.hall_heading_ipd);
//zero_ipd(&fei.hall_roll_ipd);
//zero_ipd(&fei.heading_ipd);
//zero_ipd(&fei.roll_ipd);
}


void mix_motors(int *x_step, 
	int *z_step, 
	int fraction, 
	int roll_step, 
	int heading_step)
{
	int inv_fraction = FRACTION - fei.yaw_roll_fraction;
	if(fei.handle_angle > FIXED(90))
	{
		*x_step = (-inv_fraction * roll_step + fraction * heading_step) / FRACTION;
		*z_step = (-inv_fraction * heading_step + -fraction * roll_step) / FRACTION;
	}
	else
	if(fei.handle_angle < 0)
	{
		*x_step = (inv_fraction * roll_step + -fraction * heading_step) / FRACTION;
		*z_step = (inv_fraction * heading_step + fraction * roll_step) / FRACTION;
	}
	else
	{
		*x_step = (inv_fraction * roll_step + fraction * heading_step) / FRACTION;
		*z_step = (inv_fraction * heading_step + -fraction * roll_step) / FRACTION;
	}
}

// hall sensor feedback needs different signs, for some reason
void mix_motors2(int *x_step, 
	int *z_step, 
	int fraction, 
	int roll_step, 
	int heading_step)
{
	int inv_fraction = FRACTION - fei.yaw_roll_fraction;

	if(fei.handle_angle > FIXED(90))
	{
		*x_step = (inv_fraction * roll_step + -fraction * heading_step) / FRACTION;
		*z_step = (-inv_fraction * heading_step + -fraction * roll_step) / FRACTION;
	}
	else
	if(fei.handle_angle < 0)
	{
		*x_step = (-inv_fraction * roll_step + fraction * heading_step) / FRACTION;
		*z_step = (inv_fraction * heading_step + fraction * roll_step) / FRACTION;
	}
	else
	{
		*x_step = (-inv_fraction * roll_step + -fraction * heading_step) / FRACTION;
		*z_step = (inv_fraction * heading_step + -fraction * roll_step) / FRACTION;
	}

}

void do_feedback()
{
#ifndef TEST_MOTOR
	if(!fei.calibrate_imu)
	{

// angle errors at the camera
		int x_error = get_angle_change_fixed(fei.current_roll, fei.target_roll);
		int y_error = get_angle_change_fixed(fei.current_pitch, fei.target_pitch);
		int z_error = get_angle_change_fixed(fei.current_heading, fei.target_heading);


#ifndef TEST_KINEMATICS
// calculate power by size of error
		int power_range = 1 * FRACTION;
		int min_power = MAX_POWER / 4;
		int max_power = MAX_POWER / 2;
		if(abs_fixed(x_error) < power_range &&
			abs_fixed(y_error) < power_range &&
			abs_fixed(z_error) < power_range)
		{
			int max_error = MAX(abs_fixed(x_error), abs_fixed(y_error));
			max_error = MAX(max_error, abs_fixed(z_error));
			motor.power = min_power + 
				max_error * (max_power - min_power) / 
				power_range;
		}
		else
		{
// full power
			motor.power = max_power;
		}
#endif // !TEST_KINEMATICS



// centered gyro values
		int gyro_x = (fei.gyro_x * FRACTION - fei.gyro_x_center) / FRACTION;
		int gyro_y = (fei.gyro_y * FRACTION - fei.gyro_y_center) / FRACTION;
		int gyro_z = (fei.gyro_z * FRACTION - fei.gyro_z_center) / FRACTION;

// angle rates of rates
		update_derivative(&fei.roll_accel, gyro_x);
		update_derivative(&fei.pitch_accel, gyro_y);
		update_derivative(&fei.heading_accel, gyro_z);


// update FIRs
// this didn't work as well as derivatives
//		int gyro_x_highpass = do_highpass(&fei.roll_highpass, gyro_x);
//		int gyro_y_highpass = do_highpass(&fei.pitch_highpass, gyro_y);
//		int gyro_z_highpass = do_highpass(&fei.heading_highpass, gyro_z);

#define IMU_LIMIT (10 * FRACTION)

// effect of pitch motor on pitch
// use hall sensor feedback
		if(abs_fixed(y_error) > IMU_LIMIT)
		{
			
// hall  phase
// 23846 -360
// 26184 0
// 28483 360
// predicted motor position based on hall effect sensor
			int test_y_phase = ((fei.hall2 - 26184) % 2319) * 
				360 * FRACTION / 
				2319;
// required change in phase
			int pitch_step2 = get_step(&fei.hall_pitch_ipd, 
				y_error, 
				-gyro_y, 
				0);
#ifndef TEST_KINEMATICS
			fei.y_phase = test_y_phase - pitch_step2;
#endif
		}
		else
// use gyro feedback
		{
			int pitch_step = get_step(&fei.pitch_ipd, 
				-y_error, 
				gyro_y, 
//				gyro_y_highpass);
				get_derivative(&fei.pitch_accel));
#ifndef TEST_KINEMATICS
			fei.y_phase += pitch_step;
#endif
		}


		FIX_PHASE(fei.y_phase);


// ranges for the pitch sensor
#define PITCH_VERTICAL 25000
#define PITCH_UP 20900

#define ROLL_VERTICAL 29400
#define ROLL_MIN 27280

// pitch servo in degrees
		fei.current_pitch2 = (fei.hall2 - PITCH_VERTICAL) * 90 * FRACTION / 
			(PITCH_UP - PITCH_VERTICAL);
// roll servo in degrees
		fei.current_roll2 = -(ROLL_VERTICAL - fei.hall1) * 45 * FRACTION / 
			(ROLL_VERTICAL - ROLL_MIN);



		fei.handle_angle = get_handle_angle();

// DEBUG
//fei.handle_angle = 45 * FRACTION;

// Amount yaw motor contributes to roll
		fei.yaw_roll_fraction = 1 * FRACTION - 
			(cos_fixed(fei.handle_angle * 2) + 1 * FRACTION) / 2;



// DEBUG
//yaw_roll_fraction = FRACTION * 2 / 3;

// if either Z or X motor is off, use hall sensor feedback for both
		if(abs_fixed(x_error) > IMU_LIMIT ||
			abs_fixed(z_error) > IMU_LIMIT)
		{
// predicted motor position based on hall effect sensor
// Z motor position
// 5875 -360
// 8218 0
// 10548 360
			int test_z_phase = ((fei.hall0 - 8218) % 2336) * 
				360 * FRACTION / 
				2336;


// X motor position
// 31650 0
// 29326 -360
			int test_x_phase = ((fei.hall1 - 31650) % 2324) * 
				360 * FRACTION / 
				2324;


			int heading_step = get_step(&fei.hall_heading_ipd, 
				-z_error, 
				gyro_z, 
				0);


			int roll_step = get_step(&fei.hall_roll_ipd, 
				-x_error, 
				gyro_x, 
				0);

			int x_step, z_step;
			mix_motors2(&x_step, 
				&z_step, 
				fei.yaw_roll_fraction, 
				roll_step, 
				heading_step);

#ifndef TEST_KINEMATICS

			fei.z_phase = test_z_phase + z_step;
			fei.x_phase = test_x_phase + x_step;
#endif


		}
		else
		{
// use IMU feedback


// get motor steps
			int roll_step = -get_step(&fei.roll_ipd, 
				x_error, 
				-gyro_x, 
//                -gyro_x_highpass);
				-get_derivative(&fei.roll_accel));
			int heading_step = get_step(&fei.heading_ipd, 
				-z_error, 
				gyro_z, 
//                gyro_z_highpass);
				get_derivative(&fei.heading_accel));

			int x_step, z_step;
			mix_motors(&x_step, 
				&z_step, 
				fei.yaw_roll_fraction, 
				roll_step, 
				heading_step);



#ifndef TEST_KINEMATICS
			fei.x_phase += x_step;
			fei.z_phase += z_step;
#endif




		}

if(mane_time - fei.debug_time >= HZ)
{
fei.debug_time = mane_time;
// blink the LED
send_uart(&uart, ".", 1);

//TRACE
//print_text(&uart, "recover_count=");
//print_number(&uart, imu.recover_count);
//print_text(&uart, "HZ=");
//print_number(&uart, fei.imu_count);
fei.imu_count = 0;

//print_fixed(&uart, fei.current_roll);
//print_fixed(&uart, fei.current_roll2);
//print_fixed(&uart, fei.current_pitch);
//print_fixed(&uart, fei.current_pitch2);
//print_fixed(&uart, fei.handle_angle);

//print_fixed(&uart, fei.x_phase);
//print_number(&uart, fei.hall1); // roll motor
//print_fixed(&uart, fei.yaw_roll_fraction);
//print_fixed(&uart, fei.current_heading);
//print_fixed(&uart, fei.y_phase);
//print_number(&uart, fei.hall2); // pitch motor
//print_fixed(&uart, test_phase);
//print_fixed(&uart, y_error);
//print_number(&uart, fei.hall0);
}

		FIX_PHASE(fei.x_phase);
		FIX_PHASE(fei.z_phase);



// write the yaw motor directly from board 0
		motor.phase = fei.z_phase;
		write_motor();
		
		
	}
#endif // !TEST_MOTOR
}



#endif // BOARD0


