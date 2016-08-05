// camera controller based in LIDAR



#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/serial.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <jpeglib.h>
#include <setjmp.h>
#include <unistd.h>
#include <pthread.h>



#define ABS(a) (((a) > 0) ? (a) : (-(a)))


void (*lidar_function)(unsigned char);
#define PACKET_SIZE 22
#define TOTAL_RANGES 360
unsigned char packet[PACKET_SIZE];
int distance[TOTAL_RANGES];
int distance_accum[TOTAL_RANGES];
int distance_count[TOTAL_RANGES];
int distance_counter;
int prev_distance[TOTAL_RANGES];
int diffs[TOTAL_RANGES];
int have_prev_distances = 0;
int packet_size = 0;
#define PICTURE_W 512
#define PICTURE_H 512
unsigned char *picture;
unsigned char **rows;
// meters
#define MAX_DISTANCE 4
#define OVERSAMPLE 4
int total_errors = 0;
Display *display;
Window win;
int screen;
Window rootwin;
Visual *vis;
int default_depth;
XImage *ximage;
unsigned char *x_bitmap;
GC gc;
// Output file to record
#define RECORD_PATH "lidar.out"
unsigned char *compressed_data;
int compressed_size;
int compressed_allocated;
int total_frames = 0;


int lidar_fd = -1;
int servo_fd = -1;
int min_yaw = 40;
int max_yaw = 90;
int pan = 66;
int tilt = 124;

void write_servos(int pan, int tilt);

typedef struct 
{
	struct jpeg_destination_mgr pub; /* public fields */

	JOCTET *buffer;		/* Pointer to buffer */
} mjpeg_destination_mgr;

typedef mjpeg_destination_mgr *mjpeg_dest_ptr;


METHODDEF(void) init_destination(j_compress_ptr cinfo)
{
  	mjpeg_dest_ptr dest = (mjpeg_dest_ptr)cinfo->dest;

/* Set the pointer to the preallocated buffer */
  	dest->buffer = compressed_data;
  	dest->pub.next_output_byte = compressed_data;
  	dest->pub.free_in_buffer = PICTURE_W * PICTURE_H * 3;
}

METHODDEF(boolean) empty_output_buffer(j_compress_ptr cinfo)
{
	printf("empty_output_buffer %d called\n", __LINE__);

	return TRUE;
}

METHODDEF(void) term_destination(j_compress_ptr cinfo)
{
/* Just get the length */
	mjpeg_dest_ptr dest = (mjpeg_dest_ptr)cinfo->dest;
	compressed_size = 
		compressed_allocated - 
		dest->pub.free_in_buffer;
}

GLOBAL(void) jpeg_buffer_dest(j_compress_ptr cinfo)
{
  	mjpeg_dest_ptr dest;

/* The destination object is made permanent so that multiple JPEG images
 * can be written to the same file without re-executing jpeg_stdio_dest.
 * This makes it dangerous to use this manager and a different destination
 * manager serially with the same JPEG object, because their private object
 * sizes may be different.  Caveat programmer.
 */
	if(cinfo->dest == NULL) 
	{	
/* first time for this JPEG object? */
      	cinfo->dest = (struct jpeg_destination_mgr *)
    		(*cinfo->mem->alloc_small)((j_common_ptr)cinfo, 
				JPOOL_PERMANENT,
				sizeof(mjpeg_destination_mgr));
	}

	dest = (mjpeg_dest_ptr)cinfo->dest;
	dest->pub.init_destination = init_destination;
	dest->pub.empty_output_buffer = empty_output_buffer;
	dest->pub.term_destination = term_destination;
}


void compress_jpeg()
{
	if(!compressed_data)
	{
		compressed_allocated = PICTURE_W * PICTURE_H * 3;
		compressed_data = (unsigned char*)malloc(compressed_allocated);
	}
	
	struct jpeg_compress_struct jpeg_compress;
 	struct jpeg_error_mgr jerr;
	jpeg_compress.err = jpeg_std_error(&jerr);
	jpeg_create_compress(&jpeg_compress);
	jpeg_compress.image_width = PICTURE_W;
	jpeg_compress.image_height = PICTURE_H;
	jpeg_compress.input_components = 3;
	jpeg_compress.in_color_space = JCS_RGB;
	jpeg_set_defaults(&jpeg_compress);
	jpeg_set_quality(&jpeg_compress, 80, 0);
	jpeg_compress.dct_method = JDCT_IFAST;
	jpeg_compress.comp_info[0].h_samp_factor = 2;
	jpeg_compress.comp_info[0].v_samp_factor = 2;
	jpeg_compress.comp_info[1].h_samp_factor = 1;
	jpeg_compress.comp_info[1].v_samp_factor = 1;
	jpeg_compress.comp_info[2].h_samp_factor = 1;
	jpeg_compress.comp_info[2].v_samp_factor = 1;
	jpeg_buffer_dest(&jpeg_compress);

	int i, j;
	jpeg_start_compress(&jpeg_compress, TRUE);
	while(jpeg_compress.next_scanline < jpeg_compress.image_height)
	{
		jpeg_write_scanlines(&jpeg_compress, 
			rows + jpeg_compress.next_scanline, 
			1);
	}
	
	
	
	jpeg_finish_compress(&jpeg_compress);
	jpeg_destroy((j_common_ptr)&jpeg_compress);


	char string[1024];
	sprintf(string, "%s%06d.jpg", RECORD_PATH, total_frames);
	FILE *out = fopen(string, "w");
	if(out)
	{
		fwrite(compressed_data, compressed_size, 1, out);
		fclose(out);
	}
	else
	{
		printf("Couldn't write %s\n", string);
	}
	
	total_frames++;
}

void get_sync(unsigned char c);

// Returns the FD of the serial port
static int init_serial(const char *path, int baud)
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
	cfsetispeed(&term, baud);
	cfsetospeed(&term, baud);
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



unsigned char read_char(int fd)
{
	unsigned char c;
	int result = read(fd, &c, 1);
	if(result <= 0)
	{
		printf("Unplugged\n");
		exit(1);
	}
	
	return c;
}



void draw_dot(int x, int y)
{
	int i, j;
	for(i = y - 1; i < y + 1; i++)
	{
		if(i >= 0 && i < PICTURE_H)
		{
			unsigned char *row = rows[i];
			for(j = x - 1; j < x + 1; j++)
			{
				if(j >= 0 && j < PICTURE_W)
				{
					row[j * 3 + 0] = 0;
					row[j * 3 + 1] = 0;
					row[j * 3 + 2] = 0;
				}
			}
		}
	}
}


void do_motion()
{
	int diff[TOTAL_RANGES];
	bzero(diff, sizeof(diff));
	bzero(diffs, sizeof(diffs));

	int i, j;
	if(have_prev_distances)
	{
		for(i = 0; i < TOTAL_RANGES; i++)
		{
//			if(distance_accum[i] > 0 &&
//				prev_distance[i] > 0)
			{
				diff[i] = ABS(distance_accum[i] - prev_distance[i]);
			}
//			else
//			{
//				diff[i] = 0;
//			}
		}

// get window of highest diff
		int window = 8;
		int max_diff = -1;
		int max_index = -1;
		
		
//		printf("diffs: ");
		for(i = window / 2; i < TOTAL_RANGES - window / 2; i++)
		{
			int sum = 0;
			for(j = 0; j < window; j++)
			{
				sum += diff[i - window / 2 + j];
			}


//			printf("%d ", sum);

			diffs[i] = sum;
			if(sum > 0 && sum > max_diff)
			{
				max_diff = sum;
				max_index = i;
			}
		}
//		printf("\n");

//		printf("do_motion %d max_diff=%d max_index=%d\n", 
//			__LINE__, 
//			max_diff, 
//			max_index);

	// must be bigger than minimum
		int new_angle = -1;
		max_diff = MAX_DISTANCE * 1000 * window;
		if(max_diff > 0)
		{
			for(i = 0; i < PICTURE_W; i++)
			{
				int distance_index = i * TOTAL_RANGES / PICTURE_W;
				int value = 255 - diffs[distance_index] * 255 / max_diff;
				
				for(j = 0; j < PICTURE_H; j++)
				{
					unsigned char *dst_ptr = rows[j] + i * 3;
					if(dst_ptr[0] > 0)
					{
						dst_ptr[0] = value;
						dst_ptr[1] = value;
					}
				}
			}
		
// convert to camera angle
			new_angle = max_index * (max_yaw - min_yaw) / TOTAL_RANGES + min_yaw;
			pan = new_angle;
			
//			printf("do_motion %d new_angle=%d\n", __LINE__, new_angle);


			write_servos(pan, tilt);
		}
		
		
	}


	memcpy(prev_distance, distance_accum, sizeof(distance_accum));
	have_prev_distances = 1;

}


void get_packet(unsigned char c)
{
	packet[packet_size++] = c;
	if(packet_size >= PACKET_SIZE)
	{
		lidar_function = get_sync;
		int rpm = (packet[2] | (packet[3] << 8)) / 64;
		int id = packet[1] - 0xa0;
		int error[4];
		int i, j;

/*
 * 		for(i = 0; i < PACKET_SIZE; i++)
 * 		{
 * 			printf("%02x ", packet[i]);
 * 		}
 * 		printf("\n");
 */

		if(id >= 0 && id < 90)
		{
			for(i = 0; i < 4; i++)
			{
				unsigned char *ptr = packet + 4 + i * 4;
				error[i] = (ptr[1] & 0x80) ? 1 : 0;
				if(!error[i])
				{
					distance[id * 4 + i] = ptr[0] |
						((ptr[1] & 0x3f) << 8);
				}
				else
				{
					total_errors++;
					distance[id * 4 + i] = -1;
				}
			}

// rotation done.  analyze & plot it.
			if(id == 89)
			{
				printf("rpm=%d errors=%d\n", 
					rpm,
					total_errors);
				total_errors = 0;

// accumulate distance measurements
				for(i = 0; i < TOTAL_RANGES; i++)
				{
/*
 * 					if(distance[i] > MAX_DISTANCE * 1000)
 * 					{
 * 						distance[i] = MAX_DISTANCE * 1000;
 * 					}
 */
					
/*
 * 					if(distance[i] < 0)
 * 					{
 * 						distance[i] = 0;
 * 					}
 */
					
					
					if(distance[i] >= 0 &&
						distance[i] < MAX_DISTANCE * 1000)
					{
						distance_accum[i] += distance[i];
						distance_count[i]++;
					}
				}
				
				distance_counter++;
				if(distance_counter >= OVERSAMPLE)
				{


// replace previous image
					memset(picture, 0xff, PICTURE_W * PICTURE_H * 3);
// blend with previous image
/*
 * 					for(i = 0; i < PICTURE_H; i++)
 * 					{
 * 						unsigned char *ptr = rows[i];
 * 						for(j = 0; j < PICTURE_W; j++)
 * 						{
 * 							if(ptr[0] < 0xff)
 * 							{
 * 								int value = ptr[0] + 256 / OVERSAMPLE;
 * 								if(value > 0xff) value = 0xff;
 * 								ptr[0] = value;
 * 								ptr[1] = value;
 * 								ptr[2] = value;
 * 							}
 * 
 * 
 * 							ptr += 3;
 * 						}
 * 					}
 */


					for(i = 0; i < TOTAL_RANGES; i++)
					{
						if(distance_count[i] > 0)
						{
							distance_accum[i] /= distance_count[i];
		//printf("i=%d distance=%d\n", i, distance_accum[i]);

	// polar
	/*
	 * 						int x = (int)(cos(i * 2 * M_PI / TOTAL_RANGES) * 
	 * 								distance_accum[i] / MAX_DISTANCE) + 
	 * 							PICTURE_W / 2;
	 * 						int y = (int)(sin(i * 2 * M_PI / TOTAL_RANGES) * 
	 * 								distance_accum[i] / MAX_DISTANCE) + 
	 * 							PICTURE_H / 2;
	 */


	// X-Y
							int x = i * PICTURE_W / TOTAL_RANGES;
							int y = distance_accum[i] * 
								PICTURE_H / 
								(MAX_DISTANCE * 1000);

		//printf("i=%d distance_accum=%d x=%d y=%d\n", i, distance_accum[i], x, y);
							if(x >= 0 && x < PICTURE_W && y >= 0 && y < PICTURE_H)
							{
								draw_dot(x, y);
							}
						}
					}

					do_motion();

// transfer to X bitmap
					for(i = 0; i < PICTURE_H; i++)
					{
						unsigned char *dst_ptr = x_bitmap + i * ximage->bytes_per_line;
						unsigned char *src_ptr = rows[i];
						for(j = 0; j < PICTURE_W; j++)
						{
							*dst_ptr++ = *src_ptr++;
							*dst_ptr++ = *src_ptr++;
							*dst_ptr++ = *src_ptr++;
							*dst_ptr++ = 0;
						}
					}


					XPutImage(display, 
						win, 
						gc, 
						ximage, 
						0, 
						0, 
						0, 
						0, 
						PICTURE_W, 
						PICTURE_H);
					XFlush(display);

					compress_jpeg();
					
					bzero(distance_accum, sizeof(distance_accum));
					bzero(distance_count, sizeof(distance_count));
					distance_counter = 0;
				}
			}
		}
		
/*
 * 		printf("rpm=%d id=%d error=%d%d%d%d\n", 
 * 			rpm,
 * 			packet[1] - 0xa0,
 * 			error[0],
 * 			error[1],
 * 			error[2],
 * 			error[3]);
 */
	}
}

void get_sync(unsigned char c)
{
	if(c == 0xfa)
	{
		lidar_function = get_packet;
		packet_size = 1;
		packet[0] = c;
	}
}


int init_lidar()
{
	lidar_fd = init_serial("/dev/ttyUSB0", B115200);
	if(lidar_fd < 0) lidar_fd = init_serial("/dev/ttyUSB1", B115200);
	if(lidar_fd < 0) lidar_fd = init_serial("/dev/ttyUSB2", B115200);
	if(lidar_fd < 0)
	{
		printf("Couldn't open the LIDAR serial port\n");
		return 1;
	}

	lidar_function = get_sync;
	picture = (unsigned char*)malloc(PICTURE_W * PICTURE_H * 3);
	rows = (unsigned char**)malloc(PICTURE_H * sizeof(unsigned char*));
	int i;
	for(i = 0; i < PICTURE_H; i++)
	{
		rows[i] = picture + i * PICTURE_W * 3;
	}


	bzero(distance, sizeof(distance));
	bzero(distance_accum, sizeof(distance_accum));
	bzero(distance_count, sizeof(distance_count));
	bzero(prev_distance, sizeof(prev_distance));
	distance_counter = 0;

	display = XOpenDisplay(NULL);
	screen = DefaultScreen(display);
	rootwin = RootWindow(display, screen);
	vis = DefaultVisual(display, screen);
	default_depth = DefaultDepth(display, screen);
	unsigned long mask = CWEventMask | 
				CWBackPixel | 
				CWColormap | 
				CWCursor;
	XSetWindowAttributes attr;
	mask = 0;
	win = XCreateWindow(display, 
			rootwin, 
			0, 
			0, 
			PICTURE_W, 
			PICTURE_H, 
			0, 
			default_depth, 
			InputOutput, 
			vis, 
			mask, 
			&attr);
	unsigned long gcmask = 0;
	XGCValues gcvalues;
	gc = XCreateGC(display, rootwin, gcmask, &gcvalues);
	XSizeHints size_hints;
	XGetNormalHints(display, win, &size_hints);
	size_hints.flags = PSize | PPosition;
	size_hints.width = PICTURE_W;
	size_hints.height = PICTURE_H;
	size_hints.x = 0;
	size_hints.y = 0;
	XSetStandardProperties(display, 
		win, 
		"Lidar Test", 
		"Lidar Test", 
		None, 
		0, 
		0, 
		&size_hints);
	XMapWindow(display, win); 
	XFlush(display);
	x_bitmap = 0;
	ximage = XCreateImage(display, 
		vis, 
		default_depth, 
		ZPixmap, 
		0, 
		(char*)x_bitmap, 
		PICTURE_W, 
		PICTURE_H, 
		8, 
		0);
	x_bitmap = (unsigned char*)malloc(PICTURE_H * ximage->bytes_per_line);
	XDestroyImage(ximage);
	ximage = XCreateImage(display, 
		vis, 
		default_depth, 
		ZPixmap, 
		0, 
		(char*)x_bitmap, 
		PICTURE_W, 
		PICTURE_H, 
		8, 
		0);
	return 0;
}


void* servo_listener(void *ptr)
{
	while(1)
	{
		char c = 0;
		int result = read(servo_fd, &c, 1);
	}
}

void write_servos(int pan, int tilt)
{
	if(servo_fd >= 0)
	{
		char string[1024];
		sprintf(string, "CAM %d %d\n", pan, tilt);

//		for(int i = 0; i < strlen(string); i++)
//		{
//printf("write_servos %d %c\n", __LINE__, string[i]);
//			write_char(string[i]);
//			usleep(10000);
//		}

		int temp = write(servo_fd, string, strlen(string));
	}
}

int init_servo()
{
	servo_fd = init_serial("/dev/ttyACM0", B115200);
	if(servo_fd < 0) servo_fd = init_serial("/dev/ttyACM1", B115200);
	if(servo_fd < 0) servo_fd = init_serial("/dev/ttyACM2", B115200);


	if(servo_fd >= 0)
	{
		pthread_t tid;
		pthread_attr_t  attr;
		pthread_attr_init(&attr);
		pthread_create(&tid, &attr, servo_listener, 0);

// wait for arduino bootloader
		sleep(2);
		write_servos(pan, tilt);
	}
	else
	{
		printf("Couldn't open the servo serial port\n");
		return 1;
	}
	return 0;
}



int main()
{
	if(!init_lidar())
	{
		if(!init_servo())
		{
			while(1)
			{
				unsigned char c = read_char(lidar_fd);
				lidar_function(c);
			}
		}
	}


	return 0;
}












