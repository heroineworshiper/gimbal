// generic IMU

#ifndef IMU_H
#define IMU_H

#include "settings.h"
#include "hardi2c.h"

#define PACKET_SIZE 14

typedef struct
{
	void (*current_function)(void *ptr);
	unsigned char data;
	unsigned char packet2[PACKET_SIZE];
	unsigned char packet[PACKET_SIZE];
	int offset;
	int have_packet;

	int hz;
// fixed point temperature
	int temp;
	int temp_accum;
// temperature of center reading
	int temp_center;
	int temp_bandwidth;
// temperature change / analog change
	int temp_factor_x;
	int temp_factor_y;
	int temp_factor_z;

	int is_target;

	int mag_x;
	int mag_y;
	int mag_z;
	int mag_x_axis;
	int mag_y_axis;
	int mag_z_axis;
	int mag_x_sign;
	int mag_y_sign;
	int mag_z_sign;
	int mag_x_max;
	int mag_y_max;
	int mag_z_max;
	int mag_x_min;
	int mag_y_min;
	int mag_z_min;
	int mag_bandwidth;
	int got_mag;
	int calibrate_counter;
	int calibrate_mag;
	int calibrate_imu;
	int dump_theta;


	int accel_x;
	int accel_y;
	int accel_z;
	int accel_x_axis;
	int accel_y_axis;
	int accel_z_axis;
	int accel_x_sign;
	int accel_y_sign;
	int accel_z_sign;
	int accel_bandwidth;
	int got_accel;

	int gyro_x;
	int gyro_y;
	int gyro_z;
	int gyro_x_axis;
	int gyro_y_axis;
	int gyro_z_axis;
	int gyro_x_sign;
	int gyro_y_sign;
	int gyro_z_sign;
	int gyro_bandwidth;

	int compass_sign;
	int compass_offset;
	int attitude_blend;
	int blend_counter;
	int angle_to_gyro;

	int gyro_x_center;
	int gyro_y_center;
	int gyro_z_center;
	int gyro_x_accum;
	int gyro_y_accum;
	int gyro_z_accum;
	int gyro_center_count;
// maximum amount gyros can move while calculating center
	int gyro_center_max;
	int have_gyro_center;
	int gyro_x_min;
	int gyro_y_min;
	int gyro_z_min;
	int gyro_x_max;
	int gyro_y_max;
	int gyro_z_max;
	int gyro_roll;
	int gyro_pitch;
	int gyro_heading;

	int abs_roll;
	int abs_pitch;
	int abs_heading;

// final output
	int current_roll;
	int current_pitch;
	int current_heading;
	int got_ahrs;
	
	int total_accel;
	int total_mag;
	int total_gyro;
	int total_temp;

	int gyro_count;
	int mag_timeout;
	
	hardi2c_t i2c;
} imu_t;

void init_imu(imu_t *imu);

int compass_heading(imu_t *imu, int mag_x, int mag_y, int mag_z);
void imu_update_mag(imu_t *imu, int mag_x, int mag_y, int mag_z);
void imu_read_packet(imu_t *imu);

void imu_send_results(imu_t *imu);

#ifndef USE_UART_IMU
#define HANDLE_IMU(imu) \
{ \
	handle_hardi2c(&imu.i2c); \
	if(hardi2c_ready(&imu.i2c)) \
	{ \
		imu.current_function(&imu); \
	} \
}
#else // USE_UART_IMU


#define HANDLE_IMU(imu) \
{ \
	if(imu.have_packet) \
	{ \
		imu_read_packet(&imu); \
	} \
}



#endif // !USE_UART_IMU




#endif















