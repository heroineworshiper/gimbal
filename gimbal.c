








// uart_programmer gimbal.bin


#include "stm32f4xx.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "misc.h"
#include "uart.h"
#include "linux.h"
#include "math.h"
#include "gimbal.h"
#include "idg3200.h"
#include "mpu9150.h"
#include "copter.h"



gimbal_t gimbal;

imu_t imu2;

// The mane IMU on the camera
imu_t imu1;


static int debug_counter = 0;


#define WAVEFORM_SIZE (sizeof(sin_table) / sizeof(uint16_t))
#define PWM_PERIOD 1023
#define MAX_SIN 65535




const uint16_t sin_table[] = {
        0x7fff, 0x80c8, 0x8191, 0x825a, 0x8323, 0x83ec, 0x84b5, 0x857e, 0x8647, 0x8710, 0x87d8, 0x88a1, 0x896a, 0x8a32, 0x8afa, 0x8bc3, 
        0x8c8b, 0x8d53, 0x8e1b, 0x8ee2, 0x8faa, 0x9072, 0x9139, 0x9200, 0x92c7, 0x938e, 0x9454, 0x951b, 0x95e1, 0x96a7, 0x976d, 0x9832, 
        0x98f8, 0x99bd, 0x9a82, 0x9b46, 0x9c0a, 0x9cce, 0x9d92, 0x9e56, 0x9f19, 0x9fdc, 0xa09e, 0xa161, 0xa223, 0xa2e4, 0xa3a5, 0xa466, 
        0xa527, 0xa5e7, 0xa6a7, 0xa766, 0xa826, 0xa8e4, 0xa9a3, 0xaa61, 0xab1e, 0xabdb, 0xac98, 0xad54, 0xae10, 0xaecb, 0xaf86, 0xb041, 
        0xb0fb, 0xb1b4, 0xb26d, 0xb326, 0xb3de, 0xb495, 0xb54c, 0xb603, 0xb6b9, 0xb76e, 0xb823, 0xb8d8, 0xb98c, 0xba3f, 0xbaf2, 0xbba4, 
        0xbc55, 0xbd07, 0xbdb7, 0xbe67, 0xbf16, 0xbfc5, 0xc073, 0xc120, 0xc1cd, 0xc279, 0xc324, 0xc3cf, 0xc47a, 0xc523, 0xc5cc, 0xc674, 
        0xc71c, 0xc7c2, 0xc869, 0xc90e, 0xc9b3, 0xca57, 0xcafa, 0xcb9d, 0xcc3f, 0xcce0, 0xcd80, 0xce20, 0xcebf, 0xcf5d, 0xcffa, 0xd097, 
        0xd132, 0xd1ce, 0xd268, 0xd301, 0xd39a, 0xd432, 0xd4c9, 0xd55f, 0xd5f4, 0xd689, 0xd71d, 0xd7af, 0xd842, 0xd8d3, 0xd963, 0xd9f3, 
        0xda81, 0xdb0f, 0xdb9c, 0xdc28, 0xdcb3, 0xdd3d, 0xddc6, 0xde4f, 0xded6, 0xdf5d, 0xdfe2, 0xe067, 0xe0eb, 0xe16e, 0xe1f0, 0xe271, 
        0xe2f1, 0xe370, 0xe3ee, 0xe46b, 0xe4e7, 0xe562, 0xe5dd, 0xe656, 0xe6ce, 0xe745, 0xe7bc, 0xe831, 0xe8a5, 0xe918, 0xe98b, 0xe9fc, 
        0xea6c, 0xeadb, 0xeb4a, 0xebb7, 0xec23, 0xec8e, 0xecf8, 0xed61, 0xedc9, 0xee2f, 0xee95, 0xeefa, 0xef5e, 0xefc0, 0xf022, 0xf082, 
        0xf0e1, 0xf140, 0xf19d, 0xf1f9, 0xf254, 0xf2ae, 0xf306, 0xf35e, 0xf3b4, 0xf40a, 0xf45e, 0xf4b1, 0xf503, 0xf554, 0xf5a4, 0xf5f3, 
        0xf640, 0xf68d, 0xf6d8, 0xf722, 0xf76b, 0xf7b3, 0xf7f9, 0xf83f, 0xf883, 0xf8c6, 0xf908, 0xf949, 0xf989, 0xf9c7, 0xfa04, 0xfa41, 
        0xfa7c, 0xfab5, 0xfaee, 0xfb25, 0xfb5c, 0xfb91, 0xfbc4, 0xfbf7, 0xfc28, 0xfc59, 0xfc88, 0xfcb6, 0xfce2, 0xfd0e, 0xfd38, 0xfd61, 
        0xfd89, 0xfdb0, 0xfdd5, 0xfdf9, 0xfe1c, 0xfe3e, 0xfe5e, 0xfe7e, 0xfe9c, 0xfeb9, 0xfed4, 0xfeef, 0xff08, 0xff20, 0xff37, 0xff4c, 
        0xff61, 0xff74, 0xff86, 0xff96, 0xffa6, 0xffb4, 0xffc1, 0xffcd, 0xffd7, 0xffe0, 0xffe8, 0xffef, 0xfff5, 0xfff9, 0xfffc, 0xfffe, 
        0xffff, 0xfffe, 0xfffc, 0xfff9, 0xfff5, 0xffef, 0xffe8, 0xffe0, 0xffd7, 0xffcd, 0xffc1, 0xffb4, 0xffa6, 0xff96, 0xff86, 0xff74, 
        0xff61, 0xff4c, 0xff37, 0xff20, 0xff08, 0xfeef, 0xfed4, 0xfeb9, 0xfe9c, 0xfe7e, 0xfe5e, 0xfe3e, 0xfe1c, 0xfdf9, 0xfdd5, 0xfdb0, 
        0xfd89, 0xfd61, 0xfd38, 0xfd0e, 0xfce2, 0xfcb6, 0xfc88, 0xfc59, 0xfc28, 0xfbf7, 0xfbc4, 0xfb91, 0xfb5c, 0xfb25, 0xfaee, 0xfab5, 
        0xfa7c, 0xfa41, 0xfa04, 0xf9c7, 0xf989, 0xf949, 0xf908, 0xf8c6, 0xf883, 0xf83f, 0xf7f9, 0xf7b3, 0xf76b, 0xf722, 0xf6d8, 0xf68d, 
        0xf640, 0xf5f3, 0xf5a4, 0xf554, 0xf503, 0xf4b1, 0xf45e, 0xf40a, 0xf3b4, 0xf35e, 0xf306, 0xf2ae, 0xf254, 0xf1f9, 0xf19d, 0xf140, 
        0xf0e1, 0xf082, 0xf022, 0xefc0, 0xef5e, 0xeefa, 0xee95, 0xee2f, 0xedc9, 0xed61, 0xecf8, 0xec8e, 0xec23, 0xebb7, 0xeb4a, 0xeadb, 
        0xea6c, 0xe9fc, 0xe98b, 0xe918, 0xe8a5, 0xe831, 0xe7bc, 0xe745, 0xe6ce, 0xe656, 0xe5dd, 0xe562, 0xe4e7, 0xe46b, 0xe3ee, 0xe370, 
        0xe2f1, 0xe271, 0xe1f0, 0xe16e, 0xe0eb, 0xe067, 0xdfe2, 0xdf5d, 0xded6, 0xde4f, 0xddc6, 0xdd3d, 0xdcb3, 0xdc28, 0xdb9c, 0xdb0f, 
        0xda81, 0xd9f3, 0xd963, 0xd8d3, 0xd842, 0xd7af, 0xd71d, 0xd689, 0xd5f4, 0xd55f, 0xd4c9, 0xd432, 0xd39a, 0xd301, 0xd268, 0xd1ce, 
        0xd132, 0xd097, 0xcffa, 0xcf5d, 0xcebf, 0xce20, 0xcd80, 0xcce0, 0xcc3f, 0xcb9d, 0xcafa, 0xca57, 0xc9b3, 0xc90e, 0xc869, 0xc7c2, 
        0xc71c, 0xc674, 0xc5cc, 0xc523, 0xc47a, 0xc3cf, 0xc324, 0xc279, 0xc1cd, 0xc120, 0xc073, 0xbfc5, 0xbf16, 0xbe67, 0xbdb7, 0xbd07, 
        0xbc55, 0xbba4, 0xbaf2, 0xba3f, 0xb98c, 0xb8d8, 0xb823, 0xb76e, 0xb6b9, 0xb603, 0xb54c, 0xb495, 0xb3de, 0xb326, 0xb26d, 0xb1b4, 
        0xb0fb, 0xb041, 0xaf86, 0xaecb, 0xae10, 0xad54, 0xac98, 0xabdb, 0xab1e, 0xaa61, 0xa9a3, 0xa8e4, 0xa826, 0xa766, 0xa6a7, 0xa5e7, 
        0xa527, 0xa466, 0xa3a5, 0xa2e4, 0xa223, 0xa161, 0xa09e, 0x9fdc, 0x9f19, 0x9e56, 0x9d92, 0x9cce, 0x9c0a, 0x9b46, 0x9a82, 0x99bd, 
        0x98f8, 0x9832, 0x976d, 0x96a7, 0x95e1, 0x951b, 0x9454, 0x938e, 0x92c7, 0x9200, 0x9139, 0x9072, 0x8faa, 0x8ee2, 0x8e1b, 0x8d53, 
        0x8c8b, 0x8bc3, 0x8afa, 0x8a32, 0x896a, 0x88a1, 0x87d8, 0x8710, 0x8647, 0x857e, 0x84b5, 0x83ec, 0x8323, 0x825a, 0x8191, 0x80c8, 
        0x7fff, 0x7f36, 0x7e6d, 0x7da4, 0x7cdb, 0x7c12, 0x7b49, 0x7a80, 0x79b7, 0x78ee, 0x7826, 0x775d, 0x7694, 0x75cc, 0x7504, 0x743b, 
        0x7373, 0x72ab, 0x71e3, 0x711c, 0x7054, 0x6f8c, 0x6ec5, 0x6dfe, 0x6d37, 0x6c70, 0x6baa, 0x6ae3, 0x6a1d, 0x6957, 0x6891, 0x67cc, 
        0x6706, 0x6641, 0x657c, 0x64b8, 0x63f4, 0x6330, 0x626c, 0x61a8, 0x60e5, 0x6022, 0x5f60, 0x5e9d, 0x5ddb, 0x5d1a, 0x5c59, 0x5b98, 
        0x5ad7, 0x5a17, 0x5957, 0x5898, 0x57d8, 0x571a, 0x565b, 0x559d, 0x54e0, 0x5423, 0x5366, 0x52aa, 0x51ee, 0x5133, 0x5078, 0x4fbd, 
        0x4f03, 0x4e4a, 0x4d91, 0x4cd8, 0x4c20, 0x4b69, 0x4ab2, 0x49fb, 0x4945, 0x4890, 0x47db, 0x4726, 0x4672, 0x45bf, 0x450c, 0x445a, 
        0x43a9, 0x42f7, 0x4247, 0x4197, 0x40e8, 0x4039, 0x3f8b, 0x3ede, 0x3e31, 0x3d85, 0x3cda, 0x3c2f, 0x3b84, 0x3adb, 0x3a32, 0x398a, 
        0x38e2, 0x383c, 0x3795, 0x36f0, 0x364b, 0x35a7, 0x3504, 0x3461, 0x33bf, 0x331e, 0x327e, 0x31de, 0x313f, 0x30a1, 0x3004, 0x2f67, 
        0x2ecc, 0x2e30, 0x2d96, 0x2cfd, 0x2c64, 0x2bcc, 0x2b35, 0x2a9f, 0x2a0a, 0x2975, 0x28e1, 0x284f, 0x27bc, 0x272b, 0x269b, 0x260b, 
        0x257d, 0x24ef, 0x2462, 0x23d6, 0x234b, 0x22c1, 0x2238, 0x21af, 0x2128, 0x20a1, 0x201c, 0x1f97, 0x1f13, 0x1e90, 0x1e0e, 0x1d8d, 
        0x1d0d, 0x1c8e, 0x1c10, 0x1b93, 0x1b17, 0x1a9c, 0x1a21, 0x19a8, 0x1930, 0x18b9, 0x1842, 0x17cd, 0x1759, 0x16e6, 0x1673, 0x1602, 
        0x1592, 0x1523, 0x14b4, 0x1447, 0x13db, 0x1370, 0x1306, 0x129d, 0x1235, 0x11cf, 0x1169, 0x1104, 0x10a0, 0x103e, 0x0fdc, 0x0f7c, 
        0x0f1d, 0x0ebe, 0x0e61, 0x0e05, 0x0daa, 0x0d50, 0x0cf8, 0x0ca0, 0x0c4a, 0x0bf4, 0x0ba0, 0x0b4d, 0x0afb, 0x0aaa, 0x0a5a, 0x0a0b, 
        0x09be, 0x0971, 0x0926, 0x08dc, 0x0893, 0x084b, 0x0805, 0x07bf, 0x077b, 0x0738, 0x06f6, 0x06b5, 0x0675, 0x0637, 0x05fa, 0x05bd, 
        0x0582, 0x0549, 0x0510, 0x04d9, 0x04a2, 0x046d, 0x043a, 0x0407, 0x03d6, 0x03a5, 0x0376, 0x0348, 0x031c, 0x02f0, 0x02c6, 0x029d, 
        0x0275, 0x024e, 0x0229, 0x0205, 0x01e2, 0x01c0, 0x01a0, 0x0180, 0x0162, 0x0145, 0x012a, 0x010f, 0x00f6, 0x00de, 0x00c7, 0x00b2, 
        0x009d, 0x008a, 0x0078, 0x0068, 0x0058, 0x004a, 0x003d, 0x0031, 0x0027, 0x001e, 0x0016, 0x000f, 0x0009, 0x0005, 0x0002, 0x0000, 
        0x0000, 0x0000, 0x0002, 0x0005, 0x0009, 0x000f, 0x0016, 0x001e, 0x0027, 0x0031, 0x003d, 0x004a, 0x0058, 0x0068, 0x0078, 0x008a, 
        0x009d, 0x00b2, 0x00c7, 0x00de, 0x00f6, 0x010f, 0x012a, 0x0145, 0x0162, 0x0180, 0x01a0, 0x01c0, 0x01e2, 0x0205, 0x0229, 0x024e, 
        0x0275, 0x029d, 0x02c6, 0x02f0, 0x031c, 0x0348, 0x0376, 0x03a5, 0x03d6, 0x0407, 0x043a, 0x046d, 0x04a2, 0x04d9, 0x0510, 0x0549, 
        0x0582, 0x05bd, 0x05fa, 0x0637, 0x0675, 0x06b5, 0x06f6, 0x0738, 0x077b, 0x07bf, 0x0805, 0x084b, 0x0893, 0x08dc, 0x0926, 0x0971, 
        0x09be, 0x0a0b, 0x0a5a, 0x0aaa, 0x0afb, 0x0b4d, 0x0ba0, 0x0bf4, 0x0c4a, 0x0ca0, 0x0cf8, 0x0d50, 0x0daa, 0x0e05, 0x0e61, 0x0ebe, 
        0x0f1d, 0x0f7c, 0x0fdc, 0x103e, 0x10a0, 0x1104, 0x1169, 0x11cf, 0x1235, 0x129d, 0x1306, 0x1370, 0x13db, 0x1447, 0x14b4, 0x1523, 
        0x1592, 0x1602, 0x1673, 0x16e6, 0x1759, 0x17cd, 0x1842, 0x18b9, 0x1930, 0x19a8, 0x1a21, 0x1a9c, 0x1b17, 0x1b93, 0x1c10, 0x1c8e, 
        0x1d0d, 0x1d8d, 0x1e0e, 0x1e90, 0x1f13, 0x1f97, 0x201c, 0x20a1, 0x2128, 0x21af, 0x2238, 0x22c1, 0x234b, 0x23d6, 0x2462, 0x24ef, 
        0x257d, 0x260b, 0x269b, 0x272b, 0x27bc, 0x284f, 0x28e1, 0x2975, 0x2a0a, 0x2a9f, 0x2b35, 0x2bcc, 0x2c64, 0x2cfd, 0x2d96, 0x2e30, 
        0x2ecc, 0x2f67, 0x3004, 0x30a1, 0x313f, 0x31de, 0x327e, 0x331e, 0x33bf, 0x3461, 0x3504, 0x35a7, 0x364b, 0x36f0, 0x3795, 0x383c, 
        0x38e2, 0x398a, 0x3a32, 0x3adb, 0x3b84, 0x3c2f, 0x3cda, 0x3d85, 0x3e31, 0x3ede, 0x3f8b, 0x4039, 0x40e8, 0x4197, 0x4247, 0x42f7, 
        0x43a9, 0x445a, 0x450c, 0x45bf, 0x4672, 0x4726, 0x47db, 0x4890, 0x4945, 0x49fb, 0x4ab2, 0x4b69, 0x4c20, 0x4cd8, 0x4d91, 0x4e4a, 
        0x4f03, 0x4fbd, 0x5078, 0x5133, 0x51ee, 0x52aa, 0x5366, 0x5423, 0x54e0, 0x559d, 0x565b, 0x571a, 0x57d8, 0x5898, 0x5957, 0x5a17, 
        0x5ad7, 0x5b98, 0x5c59, 0x5d1a, 0x5ddb, 0x5e9d, 0x5f60, 0x6022, 0x60e5, 0x61a8, 0x626c, 0x6330, 0x63f4, 0x64b8, 0x657c, 0x6641, 
        0x6706, 0x67cc, 0x6891, 0x6957, 0x6a1d, 0x6ae3, 0x6baa, 0x6c70, 0x6d37, 0x6dfe, 0x6ec5, 0x6f8c, 0x7054, 0x711c, 0x71e3, 0x72ab, 
        0x7373, 0x743b, 0x7504, 0x75cc, 0x7694, 0x775d, 0x7826, 0x78ee, 0x79b7, 0x7a80, 0x7b49, 0x7c12, 0x7cdb, 0x7da4, 0x7e6d, 0x7f36
};



#define CALCULATE_WAVEFORM(x) ((int)sin_table[x] * power / MAX_SIN)


int normalized_power(int motor)
{
	int battery = gimbal.battery / FRACTION;
	int power = gimbal.power[motor];
	if(battery - gimbal.min_battery > 0)
	{
		power = power * 
			(gimbal.normal_battery - gimbal.min_battery) / 
			(battery - gimbal.min_battery);
		CLAMP(power, 0, PWM_PERIOD);
	}
	return power;
}

// write motor phase to hardware
void write_motor(int number)
{
	int index1 = (gimbal.phase[number] * WAVEFORM_SIZE / 360 / FRACTION) % WAVEFORM_SIZE;
	int index2 = (index1 + 120 * WAVEFORM_SIZE / 360) % WAVEFORM_SIZE;
	int index3 = (index1 + 240 * WAVEFORM_SIZE / 360) % WAVEFORM_SIZE;

	int power = normalized_power(number);
//TRACE2
//print_number(index1);
//flush_uart();
	
	
	switch(number)
	{
		case 0:
			TIM5->CCR1 = CALCULATE_WAVEFORM(index1);
			TIM5->CCR2 = CALCULATE_WAVEFORM(index2);
			TIM5->CCR3 = CALCULATE_WAVEFORM(index3);
			break;
		case 1:
			TIM3->CCR1 = CALCULATE_WAVEFORM(index1);
			TIM3->CCR2 = CALCULATE_WAVEFORM(index2);
			TIM3->CCR3 = CALCULATE_WAVEFORM(index3);
			break;
		case 2:
			TIM1->CCR2 = CALCULATE_WAVEFORM(index1);
			TIM1->CCR3 = CALCULATE_WAVEFORM(index2);
			TIM1->CCR4 = CALCULATE_WAVEFORM(index3);
			break;
	}
}

static void test_motors()
{
	int i = 0, j;
	int motor = gimbal.heading_motor;
//	int step = FRACTION / 4;
	int step = 1;
	int delay = 1;

	write_motor(0);
	write_motor(1);
	write_motor(2);



	while(1)
	{
  		for(i = 0; i < 360 * FRACTION; i += step)
  		{
  			gimbal.phase[motor] = i;
  			write_motor(motor);
  			udelay(100);
  		}

  		for(i = 360 * FRACTION; i >= 0; i -= step)
  		{
  			gimbal.phase[motor] = i;
  			write_motor(motor);
  			udelay(100);
  		}

/*
 * 		i += step;
 * 		i %= 360 * FRACTION;
 * 		gimbal.phase[motor] = i;
 * 		write_motor(motor);
 * 		mdelay(delay);
 */

/*
 * 		for(j = 0; j  < 5; j++)
 * 		{
 * 			for(i = 0; i < 360; i += step)
 * 			{
 * 				gimbal.phase[motor] = i * FRACTION;
 * 				write_motor(motor);
 * 				mdelay(delay);
 * 			}
 * 		}
 * 	
 */

/*
 * 		for(j = 0; j  < 5; j++)
 * 		{
 * 			for(i = 360; i >= 0; i -= step)
 * 			{
 * 				gimbal.phase[motor] = i * FRACTION;
 * 				write_motor(motor);
 * 				mdelay(delay);
 * 			}
 * 		}
 */
		
	}
}

#define FIX_PHASE(x) \
	while(x < 0) x += 360 * FRACTION; \
	while(x >= 360 * FRACTION) x -= 360 * FRACTION;

#define BANDWIDTH 1
#define TOTAL_THROW_AWAY 4096

void init_analog()
{
	gimbal.analog_throw_away = TOTAL_THROW_AWAY;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_Init(ADC3, &ADC_InitStructure);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_480Cycles);
	ADC_Cmd(ADC1, ENABLE);
	ADC_SoftwareStartConv(ADC1);

	ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_480Cycles);
	ADC_Cmd(ADC2, ENABLE);
	ADC_SoftwareStartConv(ADC2);

	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_480Cycles);
	ADC_Cmd(ADC3, ENABLE);
	ADC_SoftwareStartConv(ADC3);

}



void handle_analog()
{
	if((ADC1->SR & ADC_FLAG_EOC) != 0)
	{
		ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
		if(gimbal.analog_throw_away > 0)
		{
			gimbal.y_stick = ADC1->DR * FRACTION;
		}
		else
		{
			gimbal.y_stick = (gimbal.y_stick * (FRACTION - BANDWIDTH) +
				ADC1->DR * FRACTION * BANDWIDTH) / FRACTION;
		}
		ADC_SoftwareStartConv(ADC1);
	}
	
	if((ADC2->SR & ADC_FLAG_EOC) != 0)
	{
		ADC_ClearFlag(ADC2, ADC_FLAG_EOC);
		if(gimbal.analog_throw_away > 0)
		{
			gimbal.x_stick = ADC2->DR * FRACTION;
		}
		else
		{
			gimbal.x_stick = (gimbal.x_stick * (FRACTION - BANDWIDTH) +
				ADC2->DR * FRACTION * BANDWIDTH) / FRACTION;
		}
		ADC_SoftwareStartConv(ADC2);
	}
	
	if((ADC3->SR & ADC_FLAG_EOC) != 0)
	{
		ADC_ClearFlag(ADC3, ADC_FLAG_EOC);
		if(gimbal.analog_throw_away > 0)
		{
			gimbal.battery = ADC3->DR * FRACTION;
		}
		else
		{
			gimbal.battery = (gimbal.battery * (FRACTION - BANDWIDTH) +
				ADC3->DR * FRACTION * BANDWIDTH) / FRACTION;
		}
		ADC_SoftwareStartConv(ADC3);
	}

	if(gimbal.analog_throw_away > 0)
	{
		gimbal.analog_throw_away--;
	}
}



static int get_step(pd_table_t *table, int error, int rate, int derivative)
{
	int limit = 0x7fffffff / table->i;
	CLAMP(error, -limit, limit);

	int i_result = error * table->i / FRACTION;
	CLAMP(i_result, -table->error_limit, table->error_limit);

	limit = 0x7fffffff / table->p;
	CLAMP(rate, -limit, limit);

	int p_result = rate * table->p / FRACTION;
	CLAMP(p_result, -table->rate_limit, table->rate_limit);
	
	limit = 0x7fffffff / table->d;
	CLAMP(derivative, -limit, limit);

	int d_result = derivative * table->d / FRACTION;
	CLAMP(d_result, -table->rate_limit, table->rate_limit);

	int result = i_result + p_result + d_result;
	CLAMP(result, -table->rate_limit, table->rate_limit);
	return result;
}

static void feedback()
{
	int x_error, y_error, z_error;
	int rate;
	int step;

#if MOTORS == 3
	gimbal.stick_counter++;
	if(gimbal.stick_counter >= gimbal.stick_downsample)
	{
		gimbal.stick_counter = 0;
		int x_stick = gimbal.x_stick / FRACTION;
		if(x_stick > gimbal.stick_center + gimbal.stick_deadband)
		{
			gimbal.stick_active = 1;
//			int magnitude = x_stick - gimbal.stick_center - gimbal.stick_deadband;
//			int speed = gimbal.stick_turn_rate * magnitude / (gimbal.stick_mag - gimbal.stick_deadband);
			int speed = gimbal.stick_turn_rate;
			gimbal.target_heading += speed;
			gimbal.target_heading = fix_angle(gimbal.target_heading);
		}
		else
		if(x_stick < gimbal.stick_center - gimbal.stick_deadband)
		{
			gimbal.stick_active = 1;
//			int magnitude = gimbal.stick_center - gimbal.stick_deadband - x_stick;
//			int speed = gimbal.stick_turn_rate * magnitude / (gimbal.stick_mag - gimbal.stick_deadband);
			int speed = gimbal.stick_turn_rate;
			gimbal.target_heading -= speed;
			gimbal.target_heading = fix_angle(gimbal.target_heading);
		}
		else
		if(gimbal.stick_active)
		{
			gimbal.stick_active = 0;
			gimbal.target_heading = imu1.current_heading;
		}
	}
#endif // MOTORS == 3


// Amount yaw motor contributes to roll
	int yaw_roll_fraction = 1 * FRACTION - (cos_fixed(imu2.current_pitch * 2) + 1 * FRACTION) / 2;
//	if(imu2.current_pitch < 0) yaw_roll_fraction *= -1;

// Amount yaw motor contributes to pitch
	int yaw_pitch_fraction = 1 * FRACTION - (cos_fixed(imu2.current_roll * 2) + 1 * FRACTION) / 2;
//	if(imu2.current_roll > 0) yaw_pitch_fraction *= -1;


debug_counter++;
if(!(debug_counter % 100))
{
//TRACE2
//print_number(normalized_power(0));
//print_number(normalized_power(1));
//print_number(normalized_power(2));
//print_fixed(gimbal.x_stick);
//print_fixed(gimbal.y_stick);
//print_fixed(gimbal.battery);
//print_fixed(gimbal.target_heading);
//print_fixed(imu1.current_roll);
//print_fixed(imu1.current_pitch);
//print_fixed(imu1.current_heading);
//print_fixed(yaw_roll_fraction);
//print_fixed(yaw_pitch_fraction);
}

//yaw_roll_fraction = 0 * FRACTION;

// angle errors at camera
	x_error = get_angle_change_fixed(imu1.current_roll, gimbal.target_roll);
	y_error = get_angle_change_fixed(imu1.current_pitch, gimbal.target_pitch);
	z_error = get_angle_change_fixed(imu1.current_heading, gimbal.target_heading);


// get motor steps for heading motor on top
// effect of roll motor on roll
	int top_x_step = get_step(&gimbal.top_x, 
		x_error, 
		-(imu1.gyro_x - imu1.gyro_x_center) / FRACTION, 
		-get_derivative(&gimbal.roll_accel) / FRACTION);
// effect of pitch motor on pitch
	int top_y_step = get_step(&gimbal.top_y, 
		-y_error, 
		(imu1.gyro_y - imu1.gyro_y_center) / FRACTION, 
		get_derivative(&gimbal.pitch_accel) / FRACTION);
// effect of yaw motor on yaw
	int top_z_step = get_step(&gimbal.top_z, 
		-z_error, 
		(imu1.gyro_z - imu1.gyro_z_center) / FRACTION, 
		get_derivative(&gimbal.heading_accel) / FRACTION);


// get motor steps if heading motor behind camera
// effect of roll motor on yaw
	int back_x_step = gimbal.back_feedback_sign[gimbal.roll_motor] * 
		get_step(&gimbal.back_x, 
		z_error, 
		-(imu1.gyro_z - imu1.gyro_z_center) / FRACTION, 
		-get_derivative(&gimbal.heading_accel) / FRACTION);
// effect of yaw motor on roll
	int back_z_step = gimbal.back_feedback_sign[gimbal.heading_motor] * 
		get_step(&gimbal.back_z, 
		-x_error, 
		(imu1.gyro_x - imu1.gyro_x_center) / FRACTION, 
		get_derivative(&gimbal.roll_accel) / FRACTION);



	if(imu2.current_pitch < 0)
	{
		back_x_step *= -1;
		back_z_step *= -1;
	}
	
	int total_y_step = top_y_step;
	
#if MOTORS == 3
// swap yaw & roll motors
	int total_x_step = ((FRACTION - yaw_roll_fraction) * top_x_step +
		yaw_roll_fraction * back_x_step) / FRACTION;
	int total_z_step = ((FRACTION - yaw_roll_fraction) * top_z_step +
		yaw_roll_fraction * back_z_step) / FRACTION;
// fade yaw motor as it rolls
	total_z_step = total_z_step * (FRACTION - yaw_pitch_fraction) / FRACTION;
#else
	int total_x_step = top_x_step;
	int total_z_step = 0;

#endif



	if(!gimbal.lock_motors)
	{
		gimbal.phase[gimbal.roll_motor] -= total_x_step * gimbal.feedback_sign[gimbal.roll_motor];
		gimbal.phase[gimbal.pitch_motor] -= total_y_step * gimbal.feedback_sign[gimbal.pitch_motor];
		gimbal.phase[gimbal.heading_motor] -= total_z_step * gimbal.feedback_sign[gimbal.heading_motor];

		FIX_PHASE(gimbal.phase[gimbal.roll_motor]);
		FIX_PHASE(gimbal.phase[gimbal.pitch_motor]);
		FIX_PHASE(gimbal.phase[gimbal.heading_motor]);
	}

	write_motor(0);
	write_motor(1);
	write_motor(2);

if(!(debug_counter % 100))
{
//TRACE2
//print_fixed(y_error);
//print_number(total_y_step);
//print_fixed(gimbal.phase[gimbal.heading_motor]);
//print_fixed(imu1.current_heading);
//print_fixed(gimbal.target_heading);
//print_number(-imu1.gyro_y / FRACTION);
//print_fixed(error);
//print_number(step);
}

}


void init_pd_table(pd_table_t *ptr, 
	int i, 
	int p, 
	int d, 
	int error_limit, 
	int rate_limit)
{
	ptr->i = i;
	ptr->p = p;
	ptr->d = d;
	ptr->error_limit = error_limit;
	ptr->rate_limit = rate_limit;
}


void init_feedback()
{
	init_derivative(&gimbal.roll_accel, 8);
	init_derivative(&gimbal.pitch_accel, 4);
	init_derivative(&gimbal.heading_accel, 8);


#if MOTORS == 2
	gimbal.power[gimbal.roll_motor] = PWM_PERIOD;
	gimbal.power[gimbal.pitch_motor] = PWM_PERIOD * 50 / 100;
	gimbal.power[gimbal.heading_motor] = PWM_PERIOD * 80 / 100;



// effect of roll motor on roll
	init_pd_table(&gimbal.top_x, FRACTION * 150 / 100, 128, 256, 16 * FRACTION, 16 * FRACTION);
// effect of pitch motor on pitch
	init_pd_table(&gimbal.top_y, 3 * FRACTION, 128, 256, 16 * FRACTION, 16 * FRACTION);

	gimbal.feedback_sign[gimbal.roll_motor] = 1;
	gimbal.feedback_sign[gimbal.pitch_motor] = 1;
	
	gimbal.target_roll = -3 * FRACTION;
	gimbal.target_pitch = -3 * FRACTION;

#else // MOTORS == 2

	gimbal.power[gimbal.roll_motor] = PWM_PERIOD * 80 / 100;
	gimbal.power[gimbal.pitch_motor] = PWM_PERIOD * 50 / 100;
	gimbal.power[gimbal.heading_motor] = PWM_PERIOD * 80 / 100;






// effect of roll motor on roll
	init_pd_table(&gimbal.top_x,  1024, 32, 64, 2048, 2048);
// effect of pitch motor on pitch
//	init_pd_table(&gimbal.top_y,  96,   12, 24, 2048, 2048);
	init_pd_table(&gimbal.top_y,  96,   12, 24, 2048, 2048);
// effect of yaw motor on yaw
	init_pd_table(&gimbal.top_z,  512,  32, 64, 2048, 2048);

// effect of roll motor on yaw
	init_pd_table(&gimbal.back_x, 1024, 32, 64, 2048, 2048);
// effect of yaw motor on roll
	init_pd_table(&gimbal.back_z, 256,  32, 64, 2048, 2048);





	gimbal.feedback_sign[gimbal.roll_motor] = 1;
	gimbal.feedback_sign[gimbal.pitch_motor] = 1;
	gimbal.feedback_sign[gimbal.heading_motor] = 1;
	gimbal.back_feedback_sign[gimbal.roll_motor] = -1;
	gimbal.back_feedback_sign[gimbal.pitch_motor] = 1;
	gimbal.back_feedback_sign[gimbal.heading_motor] = 1;


	gimbal.target_roll = -1 * FRACTION;
	gimbal.target_pitch = -1 * FRACTION;


#endif  // !(MOTORS == 2)



}


void init_motors()
{
	int i;

	GPIO_InitTypeDef  GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | 
		GPIO_Pin_1 | 
		GPIO_Pin_2 | 
		GPIO_Pin_6 |
		GPIO_Pin_7 |
		GPIO_Pin_9 |
		GPIO_Pin_10 |
		GPIO_Pin_11;
	GPIO_ResetBits(GPIOA, GPIO_InitStructure.GPIO_Pin);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_ResetBits(GPIOB, GPIO_InitStructure.GPIO_Pin);
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
// 32khz
	TIM_TimeBaseStructure.TIM_Prescaler = 4;
// 16khz
//	TIM_TimeBaseStructure.TIM_Prescaler = 8;
// 8 Khz
//	TIM_TimeBaseStructure.TIM_Prescaler = 16;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler /= 2;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

// all motors start as off
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_Pulse = 0;
	
#if MOTORS == 3
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	TIM_Cmd(TIM1, ENABLE);
 	TIM_CtrlPWMOutputs(TIM1, ENABLE);
#endif

	
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	
	TIM_OC3Init(TIM5, &TIM_OCInitStructure);
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);
  	
  	TIM_Cmd(TIM3, ENABLE);
 	TIM_CtrlPWMOutputs(TIM3, ENABLE);
  	TIM_Cmd(TIM5, ENABLE);
 	TIM_CtrlPWMOutputs(TIM5, ENABLE);


}



#ifdef USE_UART_IMU

imu_t imu1;
imu_t imu2;


void USART2_IRQHandler(void)
{
	imu1.data = USART2->DR;
	imu1.current_function(&imu1);
}

void USART3_IRQHandler(void)
{
	imu2.data = USART3->DR;
	imu2.current_function(&imu2);
	
}

void init_imu1()
{
	init_imu(&imu1);

	const int rx_pin = 3;
    GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	GPIO_PinAFConfig(GPIOA, rx_pin, GPIO_AF_USART2);

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = 1 << rx_pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 500000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
/* USART configuration */
  	USART_Init(USART2, &USART_InitStructure);
/* Enable USART */
  	USART_Cmd(USART2, ENABLE);

/* Enable the UART Interrupt */
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
  	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

}

void init_imu2()
{
	init_imu(&imu2);


	const int rx_pin = 11;
    GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	GPIO_PinAFConfig(GPIOB, rx_pin, GPIO_AF_USART3);

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = 1 << rx_pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 230400;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
/* USART configuration */
  	USART_Init(USART3, &USART_InitStructure);
/* Enable USART */
  	USART_Cmd(USART3, ENABLE);

/* Enable the UART Interrupt */
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
  	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	
	
	
}


#endif // USE_UART_IMU





int main()
{
	bzero(&gimbal, sizeof(gimbal_t));
	// enable the interrupt handler
 	NVIC_SetVectorTable(NVIC_VectTab_FLASH, PROGRAM_START - 0x08000000);

	
	gimbal.roll_motor = 1;
	gimbal.pitch_motor = 0;
	gimbal.heading_motor = 2;
	gimbal.lock_motors = 0;
	gimbal.dump_freqs = 0;
// battery voltage the PWM power is calibrated for
// 12V
	gimbal.normal_battery = 2510;
	gimbal.battery = gimbal.normal_battery * FRACTION;
// fudge factor to get the current to stay equal for nearby V
	gimbal.min_battery = -2000;
	gimbal.stick_center = 2072;
	gimbal.stick_mag = 2000;
//	gimbal.stick_deadband = 200;
	gimbal.stick_deadband = 1536;
//	gimbal.stick_turn_rate = 60;
	gimbal.stick_turn_rate = 40;
	gimbal.stick_downsample = 10;

	gimbal.feedback_downsample = 1;



	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |
			RCC_AHB1Periph_GPIOB |
			RCC_AHB1Periph_GPIOC |
			RCC_AHB1Periph_GPIOD |
			RCC_AHB1Periph_GPIOE |
			RCC_AHB1Periph_CCMDATARAMEN, 
		ENABLE);
// general purpose timer
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
	TIM_DeInit(TIM10);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 8;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM10, ENABLE);


// debug pin
 	GPIO_InitTypeDef  GPIO_InitStructure;
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
 	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
 	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);


	CLEAR_PIN(GPIOB, GPIO_Pin_4);
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	CLEAR_PIN(GPIOB, GPIO_Pin_3);

	init_uart();
	TRACE
	print_text("Welcome to gimbal controller\n");
	flush_uart();


	init_analog();	
	init_motors();
	init_feedback();
	init_imu1();
#if MOTORS == 3
	init_imu2();
#endif

// dump raw IMU values
	imu1.calibrate_mag = 0;
	imu2.calibrate_mag = 0;
	imu1.calibrate_imu = 0;
	imu2.calibrate_imu = 0;
	imu1.dump_theta = 0;
	imu2.dump_theta = 0;

	imu1.hz = 1700;
	imu1.temp_bandwidth = 1;
	imu1.temp_factor_x = 0;
	imu1.temp_factor_y = 0;
//	imu1.temp_factor_z = -130;
	imu1.temp_factor_z = 0;
	imu1.accel_x_axis = 1;
	imu1.accel_y_axis = 0;
	imu1.accel_z_axis = 2;
	imu1.accel_x_sign = 1;
	imu1.accel_y_sign = 1;
	imu1.accel_z_sign = -1;
	imu1.accel_bandwidth = 16;
		
	imu1.gyro_x_axis = 0;
	imu1.gyro_y_axis = 1;
	imu1.gyro_z_axis = 2;
	imu1.gyro_x_sign = 1;
	imu1.gyro_y_sign = 1;
	imu1.gyro_z_sign = 1;
	imu1.gyro_bandwidth = 256;
	
	
	imu1.compass_sign = 1;
	imu1.attitude_blend = 64;
	imu1.angle_to_gyro = 910;
//	imu1.gyro_center_max = 128;
	imu1.gyro_center_max = 900;

	imu2.hz = 1000;
	imu2.temp_bandwidth = 1;
	imu2.accel_x_axis = 2;
	imu2.accel_y_axis = 0;
	imu2.accel_z_axis = 1;
	imu2.accel_x_sign = 1;
	imu2.accel_y_sign = 1;
	imu2.accel_z_sign = 1;
	imu2.accel_bandwidth = 16;
		
	imu2.gyro_x_axis = 0;
	imu2.gyro_y_axis = 2;
	imu2.gyro_z_axis = 1;
	imu2.gyro_x_sign = 1;
	imu2.gyro_y_sign = 1;
	imu2.gyro_z_sign = -1;
	imu2.gyro_bandwidth = 256;
	
	
	imu2.compass_sign = 1;
	imu2.attitude_blend = 16;
	imu2.angle_to_gyro = 50;
	imu2.gyro_center_max = 128;



	if(imu1.calibrate_mag ||
		imu2.calibrate_mag ||
		imu1.calibrate_imu ||
		imu2.calibrate_imu ||
		imu1.dump_theta ||
		imu2.dump_theta)
	{
		gimbal.lock_motors = 1;
		gimbal.power[gimbal.pitch_motor] = 0;
		gimbal.power[gimbal.heading_motor] = 0;
		gimbal.power[gimbal.roll_motor] = 0;
	}
	
//	test_motors();

// DEBUG
//	write_motor(0);
//	write_motor(1);
//	write_motor(2);


	while(1)
	{
		if(TIM10->SR & TIM_FLAG_Update)
		{
			gimbal.timer++;
			TIM10->SR = ~TIM_FLAG_Update;
			
			if(!(gimbal.timer % 300))
			{
				if(!imu1.total_gyro)
				{
					TRACE2
					print_text("IMU1 dead");
					gimbal.total_imu_crash++;
				}

				if(!imu2.total_gyro)
				{
					TRACE2
					print_text("IMU2 dead");
					gimbal.total_imu_crash++;
				}
				
				if(imu1.total_gyro && imu2.total_gyro)
				{
					gimbal.total_imu_crash = 0;
				}
				
				if(gimbal.total_imu_crash >= 3)
				{
					TRACE2
					print_text("disabling motors");
					gimbal.lock_motors = 1;
					gimbal.power[gimbal.pitch_motor] = 0;
					gimbal.power[gimbal.heading_motor] = 0;
					gimbal.power[gimbal.roll_motor] = 0;
					write_motor(0);
					write_motor(1);
					write_motor(2);

				}

				if(gimbal.dump_freqs)
				{
					TRACE2
					print_text("gyro1=");
					print_number(imu1.total_gyro);

					print_text("gyro2=");
					print_number(imu2.total_gyro);
				}

				imu1.total_accel = imu1.total_mag = imu1.total_gyro = 0;
				imu2.total_accel = imu2.total_mag = imu2.total_gyro = 0;
			}
		}


		if((ADC1->SR & ADC_FLAG_EOC) != 0 ||
			(ADC2->SR & ADC_FLAG_EOC) != 0 ||
			(ADC3->SR & ADC_FLAG_EOC) != 0)
		{
			handle_analog();
		}


		HANDLE_IMU(imu1);

#if MOTORS == 3
		HANDLE_IMU(imu2);
#endif

		if(imu1.got_ahrs 
#if MOTORS == 3
			&& imu2.have_gyro_center
#else
			&& imu1.have_gyro_center
#endif
			)
		{
			imu1.got_ahrs = 0;
			update_derivative(&gimbal.roll_accel, (imu1.gyro_x - imu1.gyro_x_center));
			update_derivative(&gimbal.pitch_accel, (imu1.gyro_y - imu1.gyro_y_center));
			update_derivative(&gimbal.heading_accel, (imu1.gyro_z - imu1.gyro_z_center));
		
		
			gimbal.feedback_counter++;
			if(gimbal.feedback_counter >= gimbal.feedback_downsample)
			{
				gimbal.feedback_counter = 0;
				feedback();
			}
		}

		handle_uart();
	}


}
















