
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

#include "arm_linux.h"
#include "feiyu_mane.h"

#include <stdint.h>



void bzero(void *ptr, int size)
{
	int i;
	if((size % 4) || (((int)ptr) % 4))
	{
		unsigned char *ptr2 = ptr;
		for(i = 0; i < size; i++)
			ptr2[i] = 0;
	}
	else
	{
		int size4 = size / 4;
		uint32_t *ptr2 = ptr;
		for(i = 0; i < size4; i++)
			ptr2[i] = 0;
	}
}


void init_linux()
{
}


void _exit()
{
}

void mdelay(int msec)
{
	int start_time = mane_time;
	while(mane_time - start_time < msec)
	{
	}
}
