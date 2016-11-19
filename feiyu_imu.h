
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

#ifndef FEIYU_IMU
#define FEIYU_IMU

#include "feiyu_mane.h"

typedef struct
{
#ifdef BOARD2
	void (*imu_function)();

// I2c variables
	void (*i2c_function)();
	unsigned char reg;
	int bytes;
	int current_byte;
	int error;
	unsigned char i2c_buffer[16];
	int time;
	int time2;
	int count;
	int count2;
	int got_readout;
	int initialized;
#endif

#ifdef BOARD0
// use more blending to recover from a gyro saturation
	int recover_count;
#endif

} imu_t;

extern imu_t imu;


void init_imu();
void do_ahrs(unsigned char *imu_buffer);
void send_imu_result();
void do_imu();

#define handle_imu() \
{ \
	imu.count2++; \
	if(imu.imu_function != 0) \
	{ \
		imu.imu_function(); \
	} \
}


#endif





