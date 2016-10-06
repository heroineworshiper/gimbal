#ifndef FEIYU_PID_H
#define FEIYU_PID_H



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
// call it mypid_t to avoid conflicting with the linux pid_t
} mypid_t;


void init_pid(mypid_t *pid, 
	int p_factor, 
	int i_factor, 
	int d_factor, 
	int p_limit,
	int i_limit,
	int d_limit,
	int out_limit,
	int i_downsample);

int do_pid(mypid_t *pid, int error, int error_rate);
void reset_pid(mypid_t *pid);


#endif


