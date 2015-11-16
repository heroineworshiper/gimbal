#ifndef PID_H
#define PID_H



typedef struct
{
// * FRACTION
	int p_factor;
// * FRACTION
	int i_factor;
// * FRACTION
	int d_factor;
	
	int p_limit;
	int i_limit;
	int d_limit;
	int out_limit;


// i accumulator * FRACTION
	int accum;

	int counter;
	int error_accum;
	int i_downsample;
} pid_t;


void init_pid(pid_t *pid, 
	int p_factor, 
	int i_factor, 
	int d_factor, 
	int p_limit,
	int i_limit,
	int d_limit,
	int out_limit);
void set_i_downsample(pid_t *pid, int value);

int do_pid(pid_t *pid, int error, int error_rate);
int read_pid(pid_t *pid, unsigned char *data, int offset);
void reset_pid(pid_t *pid);


#endif


