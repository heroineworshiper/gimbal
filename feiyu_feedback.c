#include "feiyu_feedback.h"
#include "feiyu_imu.h"
#include "feiyu_mane.h"
#include "feiyu_hall.h"
#include "feiyu_motor.h"
#include "feiyu_uart.h"
#include "arm_linux.h"
#include "arm_math.h"









#ifdef BOARD0


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
		1 * FRACTION,   // I as high as possible
		FRACTION / 16,   // P as high as possible
		FRACTION / 8,   // D as low as possible
		255 * FRACTION,   // error limit
		255 * FRACTION);  // rate limit
	
	
// effect of roll motor on roll
	init_ipd(&fei.top_x,  
		1 * FRACTION, 
		FRACTION / 4, 
		FRACTION / 1, 
		255 * FRACTION, 
		255 * FRACTION);
// effect of yaw motor on yaw
	init_ipd(&fei.top_z, 
		1 * FRACTION,  // I
		FRACTION / 4,  // P
		FRACTION / 2,  // D
		255 * FRACTION,  // error limit
		255 * FRACTION); // rate limit
	
	
	
// effect of roll motor on yaw
	init_ipd(&fei.back_x,  
		FRACTION / 2, 
		FRACTION / 4, 
		FRACTION / 2, 
		1 * FRACTION, 
		1 * FRACTION);
// effect of yaw motor on roll
	init_ipd(&fei.back_z, 
		1 * FRACTION,  // I
		FRACTION / 4,  // P
		FRACTION / 1,  // D
		255 * FRACTION,  // error limit
		255 * FRACTION); // rate limit


	init_ipd(&fei.top_y2, 
		10 * FRACTION,  // I
		1 * FRACTION,  // P
		0,  // D
		60 * FRACTION,  // error limit
		60 * FRACTION); // rate limit
	init_ipd(&fei.top_x2, 
		10 * FRACTION,  // I
		1 * FRACTION,  // P
		0,  // D
		60 * FRACTION,  // error limit
		60 * FRACTION); // rate limit
	init_ipd(&fei.top_z2, 
		10 * FRACTION,  // I
		1 * FRACTION,  // P
		0,  // D
		60 * FRACTION,  // error limit
		60 * FRACTION); // rate limit
	init_ipd(&fei.back_x2, 
		10 * FRACTION,  // I
		1 * FRACTION,  // P
		0,  // D
		60 * FRACTION,  // error limit
		60 * FRACTION); // rate limit
	init_ipd(&fei.back_z2, 
		10 * FRACTION,  // I
		1 * FRACTION,  // P
		0,  // D
		60 * FRACTION,  // error limit
		60 * FRACTION); // rate limit
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

#define ROLL_VERTICAL 21200
#define ROLL_MIN 19000
#define ROLL_MAX 23100

// absolute pitch in degrees
		int current_pitch2 = (fei.hall2 - PITCH_VERTICAL) * 90 * FRACTION / 
			(PITCH_UP - PITCH_VERTICAL);
		CLAMP(current_pitch2, 0, 90 * FRACTION);
		

// Amount yaw motor contributes to roll
		int yaw_roll_fraction = 1 * FRACTION - (cos_fixed(current_pitch2 * 2) + 1 * FRACTION) / 2;
//		int yaw_roll_fraction = (fei.hall2 - PITCH_VERTICAL) * FRACTION / 
//			(PITCH_UP - PITCH_VERTICAL);

// Amount yaw motor contributes to pitch
//		int yaw_pitch_fraction = 1 * FRACTION - (cos_fixed(imu2.current_roll * 2) + 1 * FRACTION) / 2;


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

			int total_x_step2 = ((FRACTION - yaw_roll_fraction) * top_x_step2 +
				yaw_roll_fraction * back_x_step2) / FRACTION;
			int total_z_step2 = ((FRACTION - yaw_roll_fraction) * top_z_step2 +
				yaw_roll_fraction * back_z_step2) / FRACTION;;

			fei.z_phase = test_z_phase + total_z_step2;
			fei.x_phase = test_x_phase + total_x_step2;


		}
		else
		{
// use IMU feedback
	// get motor steps for heading motor on top
	// effect of roll motor on roll
			int top_x_step = get_step(&fei.top_x, 
				x_error, 
				-gyro_x, 
				-get_derivative(&fei.roll_accel));
	// effect of yaw motor on yaw
			int top_z_step = get_step(&fei.top_z, 
				-z_error, 
				gyro_z, 
				get_derivative(&fei.heading_accel));


	// get motor steps if heading motor behind camera
	// effect of roll motor on yaw
			int back_x_step = get_step(&fei.back_x, 
				z_error, 
				-gyro_z, 
				-get_derivative(&fei.heading_accel));
	// effect of yaw motor on roll
			int back_z_step = get_step(&fei.back_z, 
				-x_error, 
				gyro_x, 
				get_derivative(&fei.roll_accel));

			int total_x_step = ((FRACTION - yaw_roll_fraction) * top_x_step +
				yaw_roll_fraction * back_x_step) / FRACTION;
			int total_z_step = ((FRACTION - yaw_roll_fraction) * top_z_step +
				yaw_roll_fraction * back_z_step) / FRACTION;


	// scale feedback as it approaches 45'.  failed
	//		int feedback_scale = abs_fixed(yaw_roll_fraction - FRACTION / 2) +
	//			FRACTION / 2;
	//		total_x_step = total_x_step * feedback_scale / FRACTION;
	//		total_z_step = total_z_step * feedback_scale / FRACTION;

	// fade yaw motor as it rolls.
	//		total_z_step = total_z_step * (FRACTION - yaw_pitch_fraction) / FRACTION;

			fei.x_phase += total_x_step;
			fei.z_phase += total_z_step;
		}


		FIX_PHASE(fei.x_phase);
		FIX_PHASE(fei.z_phase);


if(mane_time - fei.debug_time >= HZ / 10)
{
fei.debug_time = mane_time;
//TRACE
//print_fixed(&uart, yaw_roll_fraction);
//print_number(&uart, fei.hall2);
//print_fixed(&uart, test_phase);
//print_fixed(&uart, fei.y_phase);
//print_fixed(&uart, y_error);
// print_fixed(&uart, yaw_roll_fraction);
// //print_number(&uart, fei.hall0);
// //print_number(&uart, fei.hall1); // roll motor
// //print_number(&uart, fei.hall2); // pitch motor
}

// write the yaw motor directly from board 0
		motor.phase = fei.z_phase;
		write_motor();
		
		
	}
#endif // !TEST_MOTOR
}



#endif // BOARD0


