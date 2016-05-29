#ifndef FEIYU_IMU
#define FEIYU_IMU


typedef struct
{
	void (*imu_function)();

// I2c variables
	void (*i2c_function)();
	unsigned char reg;
	int bytes;
	int current_byte;
	unsigned char i2c_buffer[16];
} imu_t;

extern imu_t imu;


void init_imu();

#define handle_imu() \
{ \
	if(imu.i2c_function != 0) \
	{ \
		imu.i2c_function(); \
	} \
	else \
	if(imu.imu_function != 0) \
	{ \
		imu.imu_function(); \
	} \
}


#endif





