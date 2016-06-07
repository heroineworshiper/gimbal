#ifndef FEIYU_MOTOR_H
#define FEIYU_MOTOR_H


typedef struct
{
	int deadband;
// the phase in degrees * FRACTION
	int phase;
// 0-MAX_POWER
	int power;
} motor_t;

#define MAX_POWER 1024

extern motor_t motor;

void init_motor();
void set_deadband();

#endif
