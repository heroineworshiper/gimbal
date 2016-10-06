// camera controller based on LIDAR



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
#include "cam.h"



#define ABS(a) (((a) > 0) ? (a) : (-(a)))


void (*lidar_function)(unsigned char);
#define PACKET_SIZE 22
// total readings in a single pass
#define TOTAL_ANGLES 360
unsigned char packet[PACKET_SIZE];
// all values for the latest pass
int distance[TOTAL_ANGLES];
// minimum number of passes to make a reference bitmap
#define PASSES 4
#define BITMAP_W TOTAL_ANGLES
#define BITMAP_H 256
// bitmaps of historic passes
#define HISTORY_SIZE 16
unsigned char history[HISTORY_SIZE][BITMAP_W * BITMAP_H];
// The 1st ranging in the history
int ref_index = 0;
// where to store the next ranging
int current_index = 0;
// total rangings in the history
int total_history = 0;
// bitmaps used for the current test
unsigned char ref_bitmap[BITMAP_W * BITMAP_H];
unsigned char current_bitmap[BITMAP_W * BITMAP_H];
// bitmap pixel ranges
#define X_THRESHOLD 1
#define Y_THRESHOLD 16
// number of dots required to calculate a position
#define TOTAL_THRESHOLD 32
// number of points required in the point cloud
#define CLOUD_THRESHOLD 16
// size of point cloud
#define CLOUD_W 32
#define CLOUD_H 32
// the last good median in pixels
int last_x = -1;
int last_y = -1;
// number of detections outside position threshold
int last_age = -1;
// maximum change in position allowed
#define POSITION_THRESHOLD_X 45
// number of detections outside threshold before discarding last position
#define AGE_THRESHOLD 4


// position in accum buffers
int current_pass = 0;
int have_ref = 0;
int packet_size = 0;
#define PICTURE_W 720
#define PICTURE_H 720
unsigned char *picture;
unsigned char **rows;
// millimeters
#define MAX_DISTANCE 4000
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
int min_yaw = 20;
int max_yaw = 110;
int pan = 66;
int tilt = 125;
cartimer_t servo_timer;
cartimer_t profile_timer;

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


void reset_timer(cartimer_t *ptr)
{
	gettimeofday(&ptr->start_time, 0);
}

int get_timer_difference(cartimer_t *ptr)
{
	struct timeval current_time;
	gettimeofday(&current_time, 0);
	int result = current_time.tv_sec * 1000 + current_time.tv_usec / 1000 -
		ptr->start_time.tv_sec * 1000 - ptr->start_time.tv_usec / 1000;
	return result;
}


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



void draw_dot(int bitmap_x, int bitmap_y, int r, int g, int b)
{
// polar
	int x = (int)(cos((double)bitmap_x * 2 * M_PI / BITMAP_W) * 
			bitmap_y * PICTURE_W / 2 / BITMAP_H) + 
		PICTURE_W / 2;
	int y = (int)(sin((double)bitmap_x * 2 * M_PI / BITMAP_W) * 
			bitmap_y * PICTURE_H / 2 / BITMAP_H) + 
		PICTURE_H / 2;


// X-Y
//	int x = bitmap_x * PICTURE_W / TOTAL_ANGLES;
//	int y = bitmap_y * PICTURE_H / BITMAP_H;

//printf("i=%d distance_accum=%d x=%d y=%d\n", i, distance_accum[i], x, y);
	if(x >= 0 && x < PICTURE_W && y >= 0 && y < PICTURE_H)
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
						row[j * 3 + 0] = r;
						row[j * 3 + 1] = g;
						row[j * 3 + 2] = b;
					}
				}
			}
		}
	}
}


void draw_line(int x1, 
	int y1, 
	int x2,
	int y2,
	int r,
	int g, 
	int b)
{
	int w = labs(x2 - x1);
	int h = labs(y2 - y1);
//printf("FindObjectMain::draw_line 1 %d %d %d %d\n", x1, y1, x2, y2);

	if(!w && !h)
	{
		draw_dot(x1, y1, r, g, b);
	}
	else
	if(w > h)
	{
// Flip coordinates so x1 < x2
		if(x2 < x1)
		{
			y2 ^= y1;
			y1 ^= y2;
			y2 ^= y1;
			x1 ^= x2;
			x2 ^= x1;
			x1 ^= x2;
		}
		int numerator = y2 - y1;
		int denominator = x2 - x1;
		int i;
		for(i = x1; i <= x2; i++)
		{
			int y = y1 + (int64_t)(i - x1) * (int64_t)numerator / (int64_t)denominator;
			draw_dot(i, y, r, g, b);
		}
	}
	else
	{
// Flip coordinates so y1 < y2
		if(y2 < y1)
		{
			y2 ^= y1;
			y1 ^= y2;
			y2 ^= y1;
			x1 ^= x2;
			x2 ^= x1;
			x1 ^= x2;
		}
		int numerator = x2 - x1;
		int denominator = y2 - y1;
		int i;
		for(i = y1; i <= y2; i++)
		{
			int x = x1 + (int64_t)(i - y1) * (int64_t)numerator / (int64_t)denominator;
			draw_dot(x, i, r, g, b);
		}
	}
}

void draw_rect(int x1, 
	int y1, 
	int x2,
	int y2,
	int r,
	int g, 
	int b)
{
	draw_line(x1, y1, x2, y1, r, g, b);
	draw_line(x2, y1, x2, y2, r, g, b);
	draw_line(x2, y2, x1, y2, r, g, b);
	draw_line(x1, y2, x1, y1, r, g, b);
}


void sort_dots(int *dots, int total)
{
	int done = 0;
	while(!done)
	{
		done = 1;
		int i;
		for(i = 0; i < total - 1; i++)
		{
			if(dots[i] > dots[i + 1])
			{
				int temp = dots[i];
				dots[i] = dots[i + 1];
				dots[i + 1] = temp;
				done = 0;
			}
		}
	}
}	




// Search for changes
void compare_frames(int *total_dots,
	int *cloud_total,
	int *median_x,
	int *median_y)
{
	int i, j, k, l;
	int index = ref_index;

// reset these but not the medians
	*total_dots = 0;
	*cloud_total = 0;

// accumulate the history to make the ref bitmap
	bzero(ref_bitmap, sizeof(ref_bitmap));

	for(i = 0; i < PASSES; i++)
	{
		unsigned char *dst = ref_bitmap;
		unsigned char *src = history[index];
		for(j = 0; j < BITMAP_H * BITMAP_W; j++)
		{
			if(*src)
			{
				*dst = 1;
			}
			
			src++;
			dst++;
		}
		
		index++;
		if(index >= HISTORY_SIZE)
		{
			index = 0;
		}
	}

// accumulate the history to make the current bitmap
	bzero(current_bitmap, sizeof(current_bitmap));
	while(index != current_index)
	{
		unsigned char *src = history[index];
		unsigned char *dst = current_bitmap;
		for(j = 0; j < BITMAP_H * BITMAP_W; j++)
		{
			if(*src)
			{
				*dst = 1;
			}
			
			src++;
			dst++;
		}
	
	
		index++;
		if(index >= HISTORY_SIZE)
		{
			index = 0;
		}
	}


// list of all the dots found outside the ref
	int x[BITMAP_W * BITMAP_H];
	int y[BITMAP_W * BITMAP_H];

	for(j = 0; j < BITMAP_H; j++)
	{
		for(i = 0; i < BITMAP_W; i++)
		{
// got current dot
			if(current_bitmap[j * BITMAP_W + i])
			{
// search for reference dot within range
				int got_it = 0;
				for(k = j - Y_THRESHOLD; 
					k < j + Y_THRESHOLD && !got_it; 
					k++)
				{
					if(k >= 0 && k < BITMAP_H)
					{
						for(l = i - X_THRESHOLD; 
							l <= i + X_THRESHOLD && !got_it; 
							l++)
						{
							if(l >= 0 && l < BITMAP_W)
							{
								if(ref_bitmap[k * TOTAL_ANGLES + l])
								{
									got_it = 1;
								}
							}
						}
					}
				}
				
				if(!got_it)
				{
					x[*total_dots] = i;
					y[*total_dots] = j;
					(*total_dots)++;

// color a dot not in the ref
					draw_dot(i, j, 255, 0, 0);
				}
			}
		}
	}


// calculate the median point
	if(*total_dots >= TOTAL_THRESHOLD)
	{
//printf("compare_frames %d\n", __LINE__);
		sort_dots(x, *total_dots);
		sort_dots(y, *total_dots);
		*median_x = x[*total_dots / 2];
		*median_y = y[*total_dots / 2];

// count the number of points near the median
		for(j = *median_y - CLOUD_H / 2; j < *median_y + CLOUD_H / 2; j++)
		{
			if(j >= 0 && j < BITMAP_H)
			{
				unsigned char *current_row = current_bitmap + j * BITMAP_W;
//				unsigned char *ref_row = ref_bitmap + j * BITMAP_W;
				for(i = *median_x - CLOUD_W / 2; i < *median_x + CLOUD_W / 2; i++)
				{
					if(i >= 0 && i < BITMAP_W)
					{
						if(current_row[i] /* || ref_row[i] */ )
						{
							(*cloud_total)++;
						}
					}
				}
			}
		}
//printf("compare_frames %d\n", __LINE__);
	}
}



void do_motion()
{
	int total_dots = 0;
	int cloud_total = 0;
	int total_dots2 = 0;
	int cloud_total2 = 0;
	int median_x = -1;
	int median_y = -1;
	int got_it = 0;
	int iterations = 0;
	int i, j;
	int prev_ref = ref_index;
	int prev_history = total_history;
//printf("do_motion %d\n", __LINE__);

//reset_timer(&profile_timer);

// advance ref_index until total_dots doesn't exceed threshold
	do
	{
		compare_frames(&total_dots,
			&cloud_total,
			&median_x,
			&median_y);
		if(total_dots >= TOTAL_THRESHOLD &&
			cloud_total >= CLOUD_THRESHOLD)
		{
			got_it = 1;
			total_dots2 = total_dots;
			cloud_total2 = cloud_total;

// advance ref until it doesn't pass anymore
			ref_index++;
			if(ref_index >= HISTORY_SIZE)
			{
				ref_index = 0;
			}
			total_history--;
		}
		iterations++;
	} while(total_dots >= TOTAL_THRESHOLD &&
		cloud_total >= CLOUD_THRESHOLD &&
		total_history > PASSES);

//printf("do_motion %d time=%d iterations=%d\n", 
//__LINE__, 
//get_timer_difference(&profile_timer),
//iterations);

	if(got_it)
	{
// test point cloud change
// 1st position always passes
		if(last_x < 0)
		{
			last_x = median_x;
			last_y = median_y;
			last_age = 0;
		}
		else
// new position was out of range
		if(ABS(last_x - median_x) > POSITION_THRESHOLD_X)
		{
			last_age++;
// hasn't been out of range long enough
			if(last_age < AGE_THRESHOLD)
			{
				got_it = 0;
			}
			else
			{
// out of range long enough.  Store new position
				last_x = median_x;
				last_y = median_y;
				last_age = 0;
			}
		}
		else
// new position was in range.  store it
		{
			last_x = median_x;
			last_y = median_y;
			last_age = 0;
		}
	}

	if(got_it)
	{
// point camera at new angle
		pan = min_yaw + 
			(TOTAL_ANGLES - median_x) * 
			(max_yaw - min_yaw) / 
			TOTAL_ANGLES;
//printf("do_motion %d\n", __LINE__);
		write_servos(pan, tilt);

printf("do_motion %d median_x=%d pan=%d total_dots2=%d cloud_total2=%d\n", 
__LINE__, 
median_x, 
pan, 
total_dots2,
cloud_total2);

// draw new point
		draw_line(median_x, median_y - 10, median_x, median_y + 10, 0, 255, 0);
		draw_line(median_x - 10, median_y, median_x + 10, median_y, 0, 255, 0);

// draw bounding box of 
		draw_rect(median_x - CLOUD_W / 2, 
			median_y - CLOUD_H / 2, 
			median_x + CLOUD_W / 2, 
			median_y + CLOUD_H / 2, 
			0, 
			255, 
			0);
	}
//printf("do_motion %d time=%d\n", __LINE__, get_timer_difference(&profile_timer));
//printf("do_motion %d\n", __LINE__);
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
		int i, j, k;

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

// Scanning done.  analyze & plot it.
			if(id == 89)
			{
// printf("rpm=%d errors=%d ref_index=%d current_index=%d total_history=%d\n", 
// rpm,
// total_errors,
// ref_index,
// current_index,
// total_history);
				total_errors = 0;

// create new bitmap from latest pass
				unsigned char *dst = history[current_index];
				bzero(dst, BITMAP_W * BITMAP_H);
				for(i = 0; i < TOTAL_ANGLES; i++)
				{
					if(distance[i] >= 0 &&
						distance[i] < MAX_DISTANCE)
					{
						int distance_scaled = distance[i] * 
							BITMAP_H / 
							MAX_DISTANCE;
						dst[distance_scaled * BITMAP_W + i] = 1;
					}
				}
				
				current_index++;
				if(current_index >= HISTORY_SIZE)
				{
					current_index = 0;
				}

				total_history++;
// overwrote start of history
				if(total_history > HISTORY_SIZE)
				{
					total_history = HISTORY_SIZE;
					ref_index++;
					if(ref_index >= HISTORY_SIZE)
					{
						ref_index = 0;
					}
// printf("get_packet %d total_history=%d ref_index=%d current_index=%d\n", 
// __LINE__,
// total_history,
// ref_index,
// current_index);
				}


// draw image of all history
				memset(picture, 0xff, PICTURE_W * PICTURE_H * 3);

				int index = ref_index;
				for(i = 0; i < total_history; i++)
				{
					unsigned char *src = history[index];
					for(j = 0; j < BITMAP_H; j++)
					{
						for(k = 0; k < BITMAP_W; k++)
						{
							if(src[j * BITMAP_W + k])
							{
								draw_dot(k, j, 0, 0, 0);
							}
						}
					} 
					index++;
					if(index >= HISTORY_SIZE)
					{
						index = 0;
					}
				}

// draw camera
				for(i = 0; i < 10; i++)
				{
					draw_dot(0, -5 + i, 0, 0, 255);
					draw_dot(90, -5 + i, 0, 0, 255);
				}
				



// store a new ref
				if(total_history > PASSES)
				{
					do_motion();
				}




// transfer to X bitmap
				for(i = 0; i < PICTURE_H; i++)
				{
					unsigned char *dst_ptr = x_bitmap + i * ximage->bytes_per_line;
					unsigned char *src_ptr = rows[i];
					for(j = 0; j < PICTURE_W; j++)
					{
						*dst_ptr++ = src_ptr[2];
						*dst_ptr++ = src_ptr[1];
						*dst_ptr++ = src_ptr[0];
						*dst_ptr++ = 0;
						src_ptr += 3;
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
	bzero(history, sizeof(history));


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

// Send a character
void write_char(int fd, unsigned char c)
{
	int result;
	do
	{
		result = write(fd, &c, 1);
	} while(!result);
}


void write_servos(int pan, int tilt)
{
	if(servo_fd >= 0)
	{
		char string[1024];
		sprintf(string, "CAM %d %d\n", pan, tilt);

		int result = write(servo_fd, string, strlen(string));
//printf("write_servos %d: result=%d %s\n", __LINE__, result, string);
	}
}

int init_servo()
{
	servo_fd = init_serial("/dev/ttyACM0", B115200);
	if(servo_fd < 0) servo_fd = init_serial("/dev/ttyACM1", B115200);
	if(servo_fd < 0) servo_fd = init_serial("/dev/ttyACM2", B115200);


	if(servo_fd >= 0)
	{
		reset_timer(&servo_timer);

		pthread_t tid;
		pthread_attr_t  attr;
		pthread_attr_init(&attr);
		pthread_create(&tid, &attr, servo_listener, 0);

// wait for arduino bootloader
		sleep(2);
		pan = (min_yaw + max_yaw) / 2;
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












