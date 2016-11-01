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





