// Motion detection routine for the motion tracking camera.



#include "cam.h"
#include "motion.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <unistd.h>


#define ABS(a)	   (((a) > 0) ? (a) : (-(a)))
#define CLAMP(x, y, z) ((x) = ((x) < (y) ? (y) : ((x) > (z) ? (z) : (x))))
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define BAUD B115200


motion_t motion;




int init_serial(const char *path)
{
	struct termios term;

	printf("init_serial %d: opening %s\n", __LINE__, path);

// Initialize serial port
	int fd = open(path, O_RDWR | O_NOCTTY | O_SYNC);
	if(fd < 0)
	{
		printf("init_serial %d: path=%s: %s\n", __LINE__, path, strerror(errno));
		return -1;
	}
	
	if (tcgetattr(fd, &term))
	{
		printf("init_serial %d: path=%s %s\n", __LINE__, path, strerror(errno));
		close(fd);
		return -1;
	}


/*
 * printf("init_serial: %d path=%s iflag=0x%08x oflag=0x%08x cflag=0x%08x\n", 
 * __LINE__, 
 * path, 
 * term.c_iflag, 
 * term.c_oflag, 
 * term.c_cflag);
 */
	tcflush(fd, TCIOFLUSH);
	cfsetispeed(&term, BAUD);
	cfsetospeed(&term, BAUD);
//	term.c_iflag = IGNBRK;
	term.c_iflag = 0;
	term.c_oflag = 0;
	term.c_lflag = 0;
//	term.c_cflag &= ~(PARENB | PARODD | CRTSCTS | CSTOPB | CSIZE);
//	term.c_cflag |= CS8;
	term.c_cc[VTIME] = 1;
	term.c_cc[VMIN] = 1;
/*
 * printf("init_serial: %d path=%s iflag=0x%08x oflag=0x%08x cflag=0x%08x\n", 
 * __LINE__, 
 * path, 
 * term.c_iflag, 
 * term.c_oflag, 
 * term.c_cflag);
 */
	if(tcsetattr(fd, TCSANOW, &term))
	{
		printf("init_serial %d: path=%s %s\n", __LINE__, path, strerror(errno));
		close(fd);
		return -1;
	}

	printf("init_serial %d: opened %s\n", __LINE__, path);
	return fd;
}


void* servo_listener(void *ptr)
{
	while(1)
	{
		char c = 0;
		int result = read(motion.servo_fd, &c, 1);
	}
}


void init_servos()
{
	motion.servo_fd = init_serial("/dev/ttyACM0");
	if(motion.servo_fd < 0) motion.servo_fd = init_serial("/dev/ttyACM1");
	if(motion.servo_fd < 0) motion.servo_fd = init_serial("/dev/ttyACM2");





	if(motion.servo_fd >= 0)
	{
		pthread_t tid;
		pthread_attr_t  attr;
		pthread_attr_init(&attr);
		pthread_create(&tid, &attr, servo_listener, 0);

// wait for arduino bootloader
		sleep(2);
	}
}

// Send a character
void write_char(unsigned char c)
{
	int result;
	do
	{
		result = write(motion.servo_fd, &c, 1);
	} while(!result);
}


void write_servos()
{
	if(motion.servo_fd >= 0)
	{
		char string[1024];
		sprintf(string, "CAM %d %d\n", motion.pan, motion.tilt);

//		for(int i = 0; i < strlen(string); i++)
//		{
//printf("write_servos %d %c\n", __LINE__, string[i]);
//			write_char(string[i]);
//			usleep(10000);
//		}

		int temp = write(motion.servo_fd, string, strlen(string));
	}
}

void process_app(vision_engine_t *engine)
{
//printf("detect_motion %d state=%d\n", __LINE__, vision.state);
	if(motion.state == MOVE_CAM)
	{
		if(vision.total_frames >= motion.move_frame + motion.move_frames)
		{
printf("process_app %d servo done\n", __LINE__);
			motion.state = DETECT_MOTION;
// update the ref
			memcpy(motion.ref, engine->in_y, vision.working_w * vision.working_h);
		}
		else
		{
			memcpy(engine->out_y, engine->in_y, vision.working_w * vision.working_h);
			memcpy(engine->out_u, engine->in_u, vision.working_w * vision.working_h);
			memcpy(engine->out_v, engine->in_v, vision.working_w * vision.working_h);
		}
	}


	if(motion.state == DETECT_MOTION)
	{
		int i, j;
		int64_t diff_total = 0;
		int64_t diff_accum_x = 0;
		int64_t diff_accum_y = 0;
		int max_diff = 0;
		int min_x = 65536;
		int max_x = 0;
		int min_y = 65536;
		int max_y = 0;

		memset(engine->out_u, 0x80, vision.working_w * vision.working_h);
		memset(engine->out_v, 0x80, vision.working_w * vision.working_h);

		for(i = 0; i < vision.working_h; i++)
		{
			unsigned char *y1 = motion.ref + i * vision.working_w;
			unsigned char *y2 = engine->in_y + i * vision.working_w;

			unsigned char *out = engine->out_y + i * vision.working_w;
			for(j = 0; j < vision.working_w; j++)
			{
				*out = *y2;

				int diff = ABS(*y2 - *y1);

				y1++;
				y2++;
				out++;

				if(diff > motion.threshold)
				{
					if(diff > max_diff)
					{
						max_diff = diff;
					}

					diff_total += diff;
					diff_accum_x += diff * j;
					diff_accum_y += diff * i;
					min_x = MIN(min_x, j);
					max_x = MAX(max_x, j);
					min_y = MIN(min_y, i);
					max_y = MAX(max_y, i);
				}
			}
		}

		int diff_w = max_x - min_x;
		int diff_h = max_y - min_y;

// color magnitude of motion
		if(diff_total > 0)
		{
			for(i = 0; i < vision.working_h; i++)
			{
				unsigned char *y1 = motion.ref + i * vision.working_w;
				unsigned char *y2 = engine->in_y + i * vision.working_w;

				unsigned char *out_y = engine->out_y + i * vision.working_w;
				unsigned char *out_u = engine->out_u + i * vision.working_w;
				unsigned char *out_v = engine->out_v + i * vision.working_w;
				for(j = 0; j < vision.working_w; j++)
				{
					int diff = ABS(*y2 - *y1);
					if(diff > motion.threshold)
					{
						int value = diff * 0x80 / max_diff;
						*out_v += value;
					}

					y1++;
					y2++;
					out_y++;
					out_u++;
					out_v++;
				}
			}


			diff_accum_x /= diff_total;
			diff_accum_y /= diff_total;
			int cross_size = 25;
			draw_line(engine, 
				diff_accum_x - cross_size, 
				diff_accum_y, 
				diff_accum_x + cross_size, 
				diff_accum_y);
			draw_line(engine, 
				diff_accum_x, 
				diff_accum_y - cross_size, 
				diff_accum_x, 
				diff_accum_y + cross_size);

			draw_rect(engine, min_x, min_y, max_x, max_y);
		}

/*
 * printf("detect_motion %d max_diff=%d diff_total=%ld diff_w=%d diff_h=%d\n", 
 * __LINE__, 
 * max_diff, 
 * diff_total,
 * diff_w,
 * diff_h);
 */


		if(diff_total > motion.threshold2)
		{
// Get another ref if the entire frame moved
			if(diff_w > motion.max_w &&
				diff_h > motion.max_h)
			{
				memcpy(motion.ref, engine->in_y, vision.working_w * vision.working_h);
			}
			else
// update the camera position
			{
				if(diff_accum_x > 400)
				{
// move right
					if(motion.pan > 40)
					{
printf("process_app %d %d %d moving right %d %d %d %d\n", __LINE__, diff_accum_x, diff_accum_y, diff_w, diff_h);
						motion.pan--;
						write_servos();
						motion.move_frame = vision.total_frames;
						motion.state = MOVE_CAM;
					}
				}
				else
				if(diff_accum_x < 240)
				{
// move left
					if(motion.pan < 90)
					{
printf("process_app %d moving left %d %d %d %d\n", __LINE__, diff_accum_x, diff_accum_y, diff_w, diff_h);
						motion.pan++;
						write_servos();
						motion.move_frame = vision.total_frames;
						motion.state = MOVE_CAM;
					}
				}
				else
				{
// Motion not in range.  Just update the ref
					memcpy(motion.ref, engine->in_y, vision.working_w * vision.working_h);
				}

			}
		}
	}

	
	
}


void init_app()
{
	motion.state = MOVE_CAM;
	motion.threshold = 1;
	motion.threshold2 = 100;
	motion.ref = (unsigned char*)malloc(vision.working_w * vision.working_h);
	motion.move_frame = 0;
	motion.move_frames = 30;
	motion.pan = 66;
	motion.tilt = 124;
	
	
	
	
	
	init_servos();
	write_servos();
}

void parse_config_app(char *ptr, char *value)
{
	if(!strcasecmp(ptr, "THRESHOLD")) motion.threshold = atoi(value);
	else
	if(!strcasecmp(ptr, "THRESHOLD2")) motion.threshold2 = atoi(value);
	else
	if(!strcasecmp(ptr, "PAN")) motion.pan = atoi(value);
	else
	if(!strcasecmp(ptr, "TILT")) motion.tilt = atoi(value);
	else
	if(!strcasecmp(ptr, "MOVE_FRAMES")) motion.move_frames = atoi(value);
	else
	if(!strcasecmp(ptr, "MAX_W")) motion.max_w = atoi(value);
	else
	if(!strcasecmp(ptr, "MAX_H")) motion.max_h = atoi(value);
}

void dump_config_app()
{
	printf("threshold=%d\n", motion.threshold);
	printf("threshold2=%d\n", motion.threshold2);
	printf("move_frames=%d\n", motion.move_frames);
	printf("servo_pan=%d\n", motion.pan);
	printf("servo_tilt=%d\n", motion.tilt);
}















