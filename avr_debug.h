#ifndef AVR_DEBUG_H
#define AVR_DEBUG_H

extern int uart_used;

void print_text(char *text);
void print_number(int number);
void print_number_unsigned(uint16_t number);
void flush_serial();
void handle_serial();
void init_serial();





#endif





