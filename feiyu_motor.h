
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
