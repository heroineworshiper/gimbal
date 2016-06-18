#ifndef FEIYU_MOTOR_H
#define FEIYU_MOTOR_H

#include "arm_math.h"
#include "feiyu_pid.h"

typedef struct
{
	int deadband;
// the phase in degrees * FRACTION
	int phase;
// 0-MAX_POWER
	int power;
// target angle in hall reading
	int hall;
// change in hall error
	derivative_t derror;
	int derror_prev[4];
	mypid_t pid;
	
} motor_t;

#define MAX_POWER 1024

extern motor_t motor;

void init_motor();
void update_motor();
void motor_test();

#endif
