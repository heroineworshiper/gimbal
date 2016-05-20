#include "arm_linux.h"

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
