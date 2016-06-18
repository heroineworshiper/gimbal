// construct tables

#include "stdio.h"
#include "stdlib.h"
#include <math.h>
#include "stdint.h"

#define N_SIN 256
#define MAX_PWM 65535
int table[N_SIN];



int pwm_table()
{
	int i;
	for(i = 0; i < N_SIN; i++)
	{
      table[i] = MAX_PWM / 2.0 + sin(2.0 * i / N_SIN * M_PI) * MAX_PWM / 2.0;
	}

	printf("const uint16_t sin_table[] = {\n\t");
	for(i = 0; i < N_SIN; i++)
	{
		printf("0x%04x", table[i]);
		if(i < N_SIN - 1) printf(", ");
		if(!((i + 1) % 16) && i != N_SIN - 1) printf("\n\t");
	}
	printf("\n};\n\n");
}


int sin_table()
{
	int i;
	printf("static const int16_t sin_table[] = {\n\t");
	for(i = 0; i < 256; i++)
	{
		printf("0x%04x", (int)(sin((float)i / 256 * (M_PI / 2)) * 0x4000));
		if(i < 255) printf(", ");
		if(!((i + 1) % 8) && i < 255) printf("\n\t");
	}
	printf("\n};\n");
}

int main()
{
	sin_table();
}

