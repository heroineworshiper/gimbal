#ifndef FEIYU_ADC
#define FEIYU_ADC


typedef struct
{
	void (*current_function)();
	int current1;
	int current2;
	int current_avg;
	int count;
	int battery;
	int battery_avg;
} adc_t;

extern adc_t adc;

void init_adc();
void handle_adc();


#endif






