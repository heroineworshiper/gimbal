#ifndef FEIYU_MOTOR_H
#define FEIYU_MOTOR_H


typedef struct
{
	int deadband;
// the phase in degrees * FRACTION
	int phase;
} motor_t;

extern motor_t motor;

void init_motor();
void set_deadband();

#endif
