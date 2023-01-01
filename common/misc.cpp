#include <avr/io.h>
#include <util/delay.h>
#include "misc.h"

/*
 *  get 24bit device id
 *  can be printed with DF("Hello from 0x%06lX", get_deviceid());
 */
uint32_t get_deviceid() {
  return ((uint32_t)SIGROW.DEVICEID0<<16) | ((uint16_t)SIGROW.DEVICEID1<<8) | (uint8_t)SIGROW.DEVICEID2;
}

/*
 * flash an output, eg a led <num> times
 */
void flash(PORT_t &PORT, uint8_t PIN, uint8_t num) {
  PORT.DIRSET = PIN;
  for (uint8_t c=0; c<num; c++) {
    PORT.OUTSET = PIN;
    _delay_ms(20);
    PORT.OUTCLR = PIN;
    if (c != num-1) _delay_ms(120);
  }
}

/*
 * https://stackoverflow.com/questions/960389/how-can-i-visualise-the-memory-sram-usage-of-an-avr-program
 * returns how many bytes between end of heap and last allocated memory on stack
 * (how much the stack can grow before colliding)
 */
int free_ram() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
