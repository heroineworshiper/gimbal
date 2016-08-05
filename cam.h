/*
 * Truck vision
 * Copyright (C) 2014-2015  Adam Williams <broadcast at earthling dot net>
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



#ifndef VISION_H
#define VISION_H



#include <stdio.h>
#include <stdint.h>
#include <linux/videodev2.h>
#include <pthread.h>
#include <semaphore.h>

// makes no difference in latency because we always consume faster than it 
// produces
#define DEVICE_BUFFERS 2
#define TOTAL_CPUS 1

#define TEXTLEN 1024
#define SQR(x) ((x) * (x))

// General purpose timer
typedef struct
{
	struct timeval start_time;
} cartimer_t;




typedef struct
{
// scaled working image
	unsigned char *in_y;
	unsigned char *in_u;
	unsigned char *in_v;
// output image for debugging
	unsigned char *out_y;
	unsigned char *out_u;
	unsigned char *out_v;

	int key_ptr;
	

// wait for frame to be ready for reading
	sem_t input_lock;
// wait for frame to be ready for processing
	sem_t ready_lock;
// wait for frame to be finished processing
	sem_t complete_lock;
// empty frame.  Stop processing.
	int eof;
// frames processed by this engine
	int total_frames;

} vision_engine_t;

typedef struct 
{
// device read buffers
	int buffer_size;
	unsigned char *frame_buffer[DEVICE_BUFFERS];

	char read_path[TEXTLEN];
	char write_path[TEXTLEN];
	char ref_path[TEXTLEN];

// imported image size
	int cam_w, cam_h;
// working image size
	int working_w, working_h;
// output image size for debugging
	int output_w, output_h;

	int spi_fd;
	int serial_fd;
// webcam FD
	int fd;
	FILE *playback_fd;
	struct v4l2_format v4l2_params;

// raw image read from camera
	unsigned char *picture_data;
	int picture_size;

// compressed input for debugging
	unsigned char *picture_in;
	int in_size;

// compressed output for debugging & web server
	unsigned char *picture_out;
	int out_size;

// copy of image for web server
	unsigned char *latest_image;
	int latest_size;

	pthread_mutex_t latest_lock;
	sem_t spi_send_lock;
	sem_t spi_complete_lock;
	unsigned char spi_tx_data[1024];
	unsigned char spi_rx_data[1024];
	int spi_tx_size;


// Total frames read or written
	int total_frames;
	int fps;
	int frames_written;
	cartimer_t timer;
	cartimer_t timer2;
	cartimer_t record_input_timer;
	int record_period;
	int led_on;

	vision_engine_t engine[TOTAL_CPUS];
// engine to put new frames into
	int current_input;
// engine to take finished frames from
	int current_output;
	

} vision_t;



extern vision_t vision;

void init_vision();
// Reset the timer.  This is not reentrant.
void reset_timer(cartimer_t *ptr);
// Get difference since last reset in ms.  This is reentrant.
int get_timer_difference(cartimer_t *ptr);
void draw_line(vision_engine_t *engine, int x1, int y1, int x2, int y2);
void draw_pixel(vision_engine_t *engine, int x, int y);
void draw_rect(vision_engine_t *engine, int x1, int y1, int x2, int y2);
int read_file_frame(FILE *fd, uint8_t *y_buffer, uint8_t *u_buffer, uint8_t *v_buffer);
void compress_output(unsigned char *out_y, 
	unsigned char *out_u,
	unsigned char *out_v,
	int w,
	int h);
void append_file(unsigned char *data, int size);


#endif // VISION_H





