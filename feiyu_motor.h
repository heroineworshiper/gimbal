#ifndef FEIYU_MOTOR_H
#define FEIYU_MOTOR_H

#include "arm_math.h"
#include "feiyu_pid.h"


#define FIX_PHASE(x) \
	while(x < 0) x += 360 * FRACTION; \
	while(x >= 360 * FRACTION) x -= 360 * FRACTION;

typedef struct
{
	int initialized;
	int deadband;
// the phase in degrees * FRACTION
	int phase;
// 0-MAX_POWER
	int power;
// debug values
	int pwm1, pwm2, pwm3;
} motor_t;

#define MAX_POWER 1024

extern motor_t motor;

void init_motor();
void update_motor();
void motor_test();

#endif
