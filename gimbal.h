#ifndef GIMBAL_H
#define GIMBAL_H

#include "settings.h"
#include "math.h"
#include "pid.h"
#include "imu.h"

// the camera imu
extern imu_t imu1;

// the yaw imu
extern imu_t imu2;


#define SYNC_CODE 0xe5

// set of PID constants
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
} pd_table_t;


typedef struct
{
	int dump_freqs;
	int roll_motor;
	int pitch_motor;
	int heading_motor;
// lock axes for calibrating mag
	int lock_motors;
// amount of torque is determined by this 0 - 255
	int power[MOTORS];
// battery voltage the power is normalized to
	int normal_battery;
	int min_battery;

// Sets of PD constants for the 3 positions of the heading motor
// heading motor on top
	pd_table_t top_x;
	pd_table_t top_y;
	pd_table_t top_z;
// heading motor behind camera
	pd_table_t back_x;
	pd_table_t back_z;
	
// if motors appear to over compensate, check just P feedback & invert this
	int feedback_sign[MOTORS];
// signs when yaw & roll are swapped
	int back_feedback_sign[MOTORS];
	

	int timer;
	int feedback_counter;
	int feedback_downsample;
// phase of each motor (0 - 360) * FRACTION
	int phase[MOTORS];

// target in deg * FRACTION
	int target_roll;
	int target_pitch;
	int target_heading;
	
	derivative_t roll_accel;
	derivative_t pitch_accel;
	derivative_t heading_accel;

	int analog_throw_away;

// analog values
	int x_stick;
	int y_stick;
	int stick_active;
	int battery;
	int stick_center;
// from center to edge
	int stick_mag;
// from center to effective range
	int stick_deadband;
// maximum amount to turn in a feedback sample
	int stick_turn_rate;
	int stick_downsample;
	int stick_counter;
	int total_imu_crash;
} gimbal_t;


extern gimbal_t gimbal;




#endif




