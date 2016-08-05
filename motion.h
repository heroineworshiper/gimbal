#ifndef MOTION_H
#define MOTION_H






typedef struct
{
	int threshold;
	int threshold2;
	int state;
#define DETECT_MOTION 0
#define MOVE_CAM 1

	
	unsigned char *ref;
	int have_ref;
	int move_frame;
	int move_frames;
	int max_w;
	int max_h;
	int pan;
	int tilt;
	int servo_fd;
} motion_t;

extern motion_t motion;


void init_app();
void parse_config_app(char *ptr, char *value);
void dump_config_app();
void process_app(vision_engine_t *engine);


#endif



