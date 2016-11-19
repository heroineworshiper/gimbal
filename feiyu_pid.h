
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

#ifndef FEIYU_PID_H
#define FEIYU_PID_H



typedef struct
{
// * FRACTION
	int p_factor;
// * FRACTION
	int i_factor;
// * FRACTION
	int d_factor;
	
	int p_limit;
	int i_limit;
	int d_limit;
	int out_limit;


// i accumulator * FRACTION
	int accum;

	int counter;
	int error_accum;
	int i_downsample;
// call it mypid_t to avoid conflicting with the linux pid_t
} mypid_t;


void init_pid(mypid_t *pid, 
	int p_factor, 
	int i_factor, 
	int d_factor, 
	int p_limit,
	int i_limit,
	int d_limit,
	int out_limit,
	int i_downsample);

int do_pid(mypid_t *pid, int error, int error_rate);
void reset_pid(mypid_t *pid);


#endif


