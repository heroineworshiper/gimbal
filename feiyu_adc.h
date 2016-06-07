#ifndef FEIYU_ADC
#define FEIYU_ADC


typedef struct
{
	void (*current_function)();
	int current1;
	int current2;
	int current1_avg;
	int current2_avg;
	int current1_count;
	int current2_count;
	int battery;
	int battery_count;
	int battery_avg;
} adc_t;

extern adc_t adc;

void init_adc();
void handle_adc();


#endif






