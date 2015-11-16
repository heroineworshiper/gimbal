#include "linux.h"
#include "math.h"
#include "pid.h"
#include "uart.h"

#define I_DOWNSAMPLE 256

void reset_pid(pid_t *pid)
{
	pid->accum = 0;
	pid->error_accum = 0;
	pid->counter = 0;
}

void copy_pid(pid_t *dst, pid_t *src)
{
	memcpy(dst, src, sizeof(pid_t));
}

void dump_pid(pid_t *pid)
{
	TRACE2
	print_fixed(pid->p_factor);
	print_fixed(pid->i_factor);
	print_fixed(pid->d_factor);
	print_fixed(pid->p_limit);
	print_fixed(pid->i_limit);
	print_fixed(pid->d_limit);
	print_fixed(pid->out_limit);
}

int read_pid(pid_t *pid, unsigned char *data, int offset)
{
	pid->p_factor = (int16_t)(data[offset] | (data[offset + 1] << 8));
	offset += 2;
	pid->i_factor = (int16_t)(data[offset] | (data[offset + 1] << 8));
	offset += 2;
	pid->d_factor = (int16_t)(data[offset] | (data[offset + 1] << 8));
	offset += 2;
	pid->p_limit = (int16_t)(data[offset] | (data[offset + 1] << 8));
	offset += 2;
	pid->i_limit = (int16_t)(data[offset] | (data[offset + 1] << 8));
	offset += 2;
	pid->d_limit = (int16_t)(data[offset] | (data[offset + 1] << 8));
	offset += 2;
	pid->out_limit = (int16_t)(data[offset] | (data[offset + 1] << 8));
	offset += 2;
// 14 bytes
	return offset;
}

void init_pid(pid_t *pid, 
	int p_factor, 
	int i_factor, 
	int d_factor, 
	int p_limit,
	int i_limit,
	int d_limit,
	int out_limit)
{
	pid->p_factor = p_factor;
	pid->i_factor = i_factor;
	pid->d_factor = d_factor;
	pid->p_limit = p_limit;
	pid->i_limit = i_limit;
	pid->d_limit = d_limit;
	pid->out_limit = out_limit;
	pid->i_downsample = I_DOWNSAMPLE;
}

void set_i_downsample(pid_t *pid, int value)
{
	pid->i_downsample = value;
}


int do_pid(pid_t *pid, int error, int error_rate)
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








