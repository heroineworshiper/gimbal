#ifndef FEIYU_HALL
#define FEIYU_HALL


#include "arm_math.h"

typedef struct
{
	void (*current_function)();
	int value;
	int time;
} hall_t;


hall_t hall;


void init_hall();

#endif






