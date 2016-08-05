#include "arm_linux.h"
#include "arm_math.h"
#include "feiyu_pid.h"
#include "feiyu_uart.h"


void reset_pid(mypid_t *pid)
{
	pid->accum = 0;
	pid->error_accum = 0;
	pid->counter = 0;
}

void copy_pid(mypid_t *dst, mypid_t *src)
{
	memcpy(dst, src, sizeof(mypid_t));
}

void dump_pid(mypid_t *pid)
{
	TRACE2
	print_fixed(&uart, pid->p_factor);
	print_fixed(&uart, pid->i_factor);
	print_fixed(&uart, pid->d_factor);
	print_fixed(&uart, pid->p_limit);
	print_fixed(&uart, pid->i_limit);
	print_fixed(&uart, pid->d_limit);
	print_fixed(&uart, pid->out_limit);
}


void init_pid(mypid_t *pid, 
	int p_factor, 
	int i_factor, 
	int d_factor, 
	int p_limit,
	int i_limit,
	int d_limit,
	int out_limit,
	int i_downsample)
{
	pid->p_factor = p_factor;
	pid->i_factor = i_factor;
	pid->d_factor = d_factor;
	pid->p_limit = p_limit;
	pid->i_limit = i_limit;
	pid->d_limit = d_limit;
	pid->out_limit = out_limit;
	pid->i_downsample = i_downsample;
}



int do_pid(mypid_t *pid, int error, int error_rate)
{
	int p_result = error * pid->p_factor / FRACTION;
	CLAMP(p_result, -pid->p_limit, pid->p_limit);
	
	int d_result = error_rate * pid->d_factor / FRACTION;
	CLAMP(d_result, -pid->d_limit, pid->d_limit);

	pid->error_accum += error;
	pid->counter++;
// not enough precision to do it in every cycle
	if(pid->counter >= pid->i_downsample)
	{
		pid->error_accum /= pid->counter;
		pid->accum += pid->error_accum * pid->i_factor;
		CLAMP(pid->accum, -pid->i_limit * FRACTION, pid->i_limit * FRACTION);
		
		pid->counter = 0;
		pid->error_accum = 0;
	}

//	pid->accum += error * pid->i_factor;
//	CLAMP(pid->accum, -pid->i_limit * FRACTION, pid->i_limit * FRACTION);
			
	int result = p_result + d_result + pid->accum / FRACTION;
	CLAMP(result, -pid->out_limit, pid->out_limit);
	
	return result;
}








