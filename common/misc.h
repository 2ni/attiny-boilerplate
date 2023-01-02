#ifndef __MISC_H__
#define __MISC_H__

#include <avr/io.h>

void    setup_clk();
uint32_t get_deviceid();
int      free_ram();
void     flash(PORT_t &port, uint8_t pin, uint8_t num=3);

#endif
