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


int get_handle_angle()
{
	int table_row = fei.current_roll2 / 5 / FRACTION;
	if(table_row < 0) table_row *= -1;
	int table_column = fei.current_pitch2 / 5 / FRACTION - 1;

	if(table_column < 0)
	{
		return fei.current_roll2;
	}
	else
	if(table_column > 15)
	{
		return 90 * FRACTION;
	}
	
	return handle_table[table_row * 17 + table_column] * FRACTION;
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




// motor feedback is on board 0 only
void init_feedback()
{
	init_derivative(&fei.roll_accel, ROLL_D_SIZE);
	init_derivative(&fei.pitch_accel, PITCH_D_SIZE);
	init_derivative(&fei.heading_accel, HEADING_D_SIZE);
	
	
	
// effect of pitch motor on pitch
	init_ipd(&fei.top_y,  
		FIXED(1),   // I as high as possible
		FIXED(0.0625),   // P as high as possible
		FIXED(0.125),   // D as low as possible
		FIXED(255),   // error limit
		FIXED(255));  // rate limit
	


// handle at 0 deg

// effect of roll motor on roll
	init_ipd(&fei.top_x,  
		FIXED(1),   // I
		FIXED(0.25),   // P
		FIXED(1),   // D
		FIXED(255), 
		FIXED(255));
// effect of yaw motor on yaw
	init_ipd(&fei.top_z, 
		FIXED(1),  // I
		FIXED(0.25),  // P
		FIXED(0.5),  // D
		FIXED(255),  // error limit
		FIXED(255)); // rate limit




// handle at 45 deg
// effect of roll motor on roll
	init_ipd(&fei.top_x45,  
		FIXED(0.5),   // I
//		FIXED(1),  // I
		FIXED(0.25),   // P
		FIXED(2),   // D
		FIXED(255), 
		FIXED(255));
// effect of yaw motor on yaw
	init_ipd(&fei.top_z45, 
		FIXED(0.5),  // I
//		FIXED(1),  // I
		FIXED(0.25),  // P
		FIXED(2),  // D
		FIXED(255),  // error limit
		FIXED(255)); // rate limit
	
	
	
// effect of roll motor on yaw
	init_ipd(&fei.back_x45,  
		FIXED(0.5),  // I
//		FIXED(1),  // I
		FIXED(0.25),  // P
		FIXED(2),  // D 
		FIXED(1), 
		FIXED(1));
// effect of yaw motor on roll
	init_ipd(&fei.back_z45, 
		FIXED(0.5),  // I
//		FIXED(1),  // I
		FIXED(0.25),  // P
		FIXED(2),  // D
		FIXED(255),  // error limit
		FIXED(255)); // rate limit


// handle at 90 deg

// effect of roll motor on yaw
	init_ipd(&fei.back_x,  
		FIXED(0.5),  // I
//		FIXED(1),  // I
		FIXED(0.25),  // P
		FIXED(0.5),  // D 
		FIXED(1), 
		FIXED(1));
// effect of yaw motor on roll
	init_ipd(&fei.back_z, 
		FIXED(1),  // I
		FIXED(0.25),  // P
		FIXED(1),  // D
		FIXED(255),  // error limit
		FIXED(255)); // rate limit



// feedback using hall sensors
	init_ipd(&fei.top_y2, 
		FIXED(10),  // I
		FIXED(1),  // P
		0,  // D
		FIXED(60),  // error limit
		FIXED(60)); // rate limit
	init_ipd(&fei.top_x2, 
		FIXED(10),  // I
		FIXED(1),  // P
		0,  // D
		FIXED(60),  // error limit
		FIXED(60)); // rate limit
	init_ipd(&fei.top_z2, 
		FIXED(10),  // I
		FIXED(1),  // P
		0,  // D
		FIXED(60),  // error limit
		FIXED(60)); // rate limit
	init_ipd(&fei.back_x2, 
		FIXED(10),  // I
		FIXED(1),  // P
		0,  // D
		FIXED(60),  // error limit
		FIXED(60)); // rate limit
	init_ipd(&fei.back_z2, 
		FIXED(10),  // I
		FIXED(1),  // P
		0,  // D
		FIXED(60),  // error limit
		FIXED(60)); // rate limit
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

#define IMU_LIMIT (10 * FRACTION)

// effect of pitch motor on pitch
// use hall sensor feedback
		if(abs_fixed(y_error) > IMU_LIMIT)
		{
			
// hall  phase
// 23860 -360
// 26191 0
// 28507 360
// predicted phase based on hall effect sensor
			int test_y_phase = ((fei.hall2 - 26191) % 2323) * 
				360 * FRACTION / 
				2323;
// required change in phase
			int top_y_step2 = get_step(&fei.top_y2, 
				y_error, 
				-gyro_y, 
				0);
			fei.y_phase = test_y_phase + top_y_step2;
		}
		else
// use gyro feedback
		{
			int top_y_step = get_step(&fei.top_y, 
				-y_error, 
				gyro_y, 
				get_derivative(&fei.pitch_accel));
			fei.y_phase -= top_y_step;
		}


		FIX_PHASE(fei.y_phase);


// ranges for the pitch sensor
// use top values
#define PITCH_VERTICAL 25100
// use back values
#define PITCH_UP 29255
// not implemented
#define PITCH_DOWN 21000

#define ROLL_VERTICAL 21000
#define ROLL_MIN 19000
#define ROLL_MAX 23600

// pitch servo in degrees
		fei.current_pitch2 = (fei.hall2 - PITCH_VERTICAL) * 90 * FRACTION / 
			(PITCH_UP - PITCH_VERTICAL);
		CLAMP(fei.current_pitch2, 0, 90 * FRACTION);
// roll servo in degrees
		fei.current_roll2 = (ROLL_VERTICAL - fei.hall1) * 50 * FRACTION / 
			(ROLL_VERTICAL - ROLL_MIN);



		fei.handle_angle = get_handle_angle();


// DEBUG
//fei.handle_angle = 45 * FRACTION;

// Amount yaw motor contributes to roll
//		fei.yaw_roll_fraction = 1 * FRACTION - 
//			(cos_fixed(fei.handle_angle * 2) + 1 * FRACTION) / 2;
		fei.yaw_roll_fraction = fei.handle_angle * FRACTION / (90 * FRACTION);
// copy for debugging
		int yaw_roll_fraction1 = fei.yaw_roll_fraction;

// fudge the number
		int division1 = FRACTION / 5;
		int division2 = FRACTION / 3;
		int halfway = FRACTION / 2;
		int division4 = FRACTION * 2 / 3;
		int division5 = FRACTION * 3 / 4;
		if(fei.yaw_roll_fraction < division1)
		{
			fei.yaw_roll_fraction = 0;
		}
		else
		if(fei.yaw_roll_fraction < division2)
		{
			// scale it to 0 ... halfway
			fei.yaw_roll_fraction = (fei.yaw_roll_fraction - division1) * 
				(halfway - 0) /
				(division2 - division1) +
				0;
		}
		else
		if(fei.yaw_roll_fraction < division4)
		{
			// always halfway
			fei.yaw_roll_fraction = halfway;
		}
		else
		if(fei.yaw_roll_fraction < division5)
		{
			// scale it to halfway ... FRACTION
			fei.yaw_roll_fraction = (fei.yaw_roll_fraction - division4) * 
				(FRACTION - halfway) /
				(division5 - division4) +
				halfway;
		}
		else
		{
			fei.yaw_roll_fraction = FRACTION;
		}

// DEBUG
//yaw_roll_fraction = FRACTION * 2 / 3;

// if either Z or X motor is off, use hall sensor feedback for both
		if(abs_fixed(x_error) > IMU_LIMIT ||
			abs_fixed(z_error) > IMU_LIMIT)
		{
// Z hall phase
// 5875 -360
// 8218 0
// 10548 360
			int test_z_phase = ((fei.hall0 - 8218) % 2336) * 
				360 * FRACTION / 
				2336;


// X hall phase
// 19925 0
// 22309 360
			int test_x_phase = ((fei.hall1 - 19925) % 2384) * 
				360 * FRACTION / 
				2384;




// effect of yaw motor on yaw
			int top_z_step2 = get_step(&fei.top_z2, 
				-z_error, 
				gyro_z, 
				0);


// effect of roll motor on roll
			int top_x_step2 = get_step(&fei.top_x2, 
				-x_error, 
				gyro_x, 
				0);


// effect of roll motor on yaw
			int back_x_step2 = get_step(&fei.back_x2, 
				z_error, 
				-gyro_z, 
				0);

// effect of yaw motor on roll
			int back_z_step2 = get_step(&fei.back_z2, 
				-x_error, 
				gyro_x, 
				0);

			int total_x_step2 = ((FRACTION - fei.yaw_roll_fraction) * top_x_step2 +
				fei.yaw_roll_fraction * back_x_step2) / FRACTION;
			int total_z_step2 = ((FRACTION - fei.yaw_roll_fraction) * top_z_step2 +
				fei.yaw_roll_fraction * back_z_step2) / FRACTION;;

#ifndef TEST_KINEMATICS

			fei.z_phase = test_z_phase + total_z_step2;
			fei.x_phase = test_x_phase + total_x_step2;
#endif


		}
		else
		{
// use IMU feedback
// schedule gains based on handle angle
			ipd_t top_x;
			ipd_t top_z;
			ipd_t back_x;
			ipd_t back_z;
			int fraction45 = abs_fixed(fei.yaw_roll_fraction - FRACTION / 2) * 2;
			

			blend_ipd(&top_x, 
				&fei.top_x45,
				&fei.top_x,
				fraction45);
			blend_ipd(&top_z, 
				&fei.top_z45,
				&fei.top_z,
				fraction45);
			blend_ipd(&back_x, 
				&fei.back_x45,
				&fei.back_x,
				fraction45);
			blend_ipd(&back_z, 
				&fei.back_z45,
				&fei.back_z,
				fraction45);



	// get motor steps for heading motor on top
	// effect of roll motor on roll
			int top_x_step = get_step(&top_x, 
				x_error, 
				-gyro_x, 
				-get_derivative(&fei.roll_accel));
	// effect of yaw motor on yaw
			int top_z_step = get_step(&top_z, 
				-z_error, 
				gyro_z, 
				get_derivative(&fei.heading_accel));


	// get motor steps if heading motor behind camera
	// effect of roll motor on yaw
			int back_x_step = get_step(&back_x, 
				z_error, 
				-gyro_z, 
				-get_derivative(&fei.heading_accel));
	// effect of yaw motor on roll
			int back_z_step = get_step(&back_z, 
				-x_error, 
				gyro_x, 
				get_derivative(&fei.roll_accel));

			int total_x_step = ((FRACTION - fei.yaw_roll_fraction) * top_x_step +
				fei.yaw_roll_fraction * back_x_step) / FRACTION;
			int total_z_step = ((FRACTION - fei.yaw_roll_fraction) * top_z_step +
				fei.yaw_roll_fraction * back_z_step) / FRACTION;


	// scale feedback as it approaches 45'.  failed
	//		int feedback_scale = abs_fixed(fei.yaw_roll_fraction - FRACTION / 2) +
	//			FRACTION / 2;
	//		total_x_step = total_x_step * feedback_scale / FRACTION;
	//		total_z_step = total_z_step * feedback_scale / FRACTION;

	// fade yaw motor as it rolls.
	//		total_z_step = total_z_step * (FRACTION - yaw_pitch_fraction) / FRACTION;


#ifndef TEST_KINEMATICS
			fei.x_phase += total_x_step;
			fei.z_phase += total_z_step;
#endif




		}

//if(mane_time - fei.debug_time >= HZ / 10)
//{
//fei.debug_time = mane_time;
//TRACE
//print_fixed(&uart, fei.current_roll);
//print_fixed(&uart, fei.current_roll2);
//print_fixed(&uart, fei.current_pitch2);
//print_fixed(&uart, fei.handle_angle);
//print_fixed(&uart, yaw_roll_fraction1);
//print_fixed(&uart, fei.yaw_roll_fraction);

//print_number(&uart, fei.hall1); // roll motor
//print_fixed(&uart, fei.current_heading);
//print_number(&uart, fei.hall2);
//print_fixed(&uart, test_phase);
//print_fixed(&uart, fei.y_phase);
//print_fixed(&uart, y_error);
//print_number(&uart, fei.hall0);
//print_number(&uart, fei.hall2); // pitch motor
//}

		FIX_PHASE(fei.x_phase);
		FIX_PHASE(fei.z_phase);



// write the yaw motor directly from board 0
		motor.phase = fei.z_phase;
		write_motor();
		
		
	}
#endif // !TEST_MOTOR
}



#endif // BOARD0


