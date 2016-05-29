#ifndef FEIYU_ADC
#define FEIYU_ADC


typedef struct
{
	void (*current_function)();
	int value1;
	int value2;
} adc_t;

extern adc_t adc;

void init_adc();
void handle_adc();


#endif






