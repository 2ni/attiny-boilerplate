#include <util/delay.h>
#include <avr/io.h>
#include "uart.h"

int main(void) {
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm); // 10MHz
  _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, CLKCTRL_ENABLE_bm | CLKCTRL_RUNSTDBY_bm);
  while (CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm);

  PORTMUX.CTRLB = PORTMUX_USART0_ALTERNATE_gc | PORTMUX_SPI0_ALTERNATE_gc; // alternate pins for uart, spi
  SLPCTRL.CTRLA = (SLPCTRL_SMODE_STDBY_gc | SLPCTRL_SEN_bm);

  uart_init();

  PORTB.DIRSET = PIN5_bm; // set PB5 (led) as output

  // blink 3x
  uint8_t num = 3;
  for (uint8_t c=0; c<num; c++) {
    PORTB.OUTSET = PIN5_bm;
    _delay_ms(20);
    PORTB.OUTCLR = PIN5_bm;
    if (c != num-1) _delay_ms(100);
  }

  while (1) {
    PORTB.OUTTGL = PIN5_bm;
    _delay_ms(500);
    DL("blink");
  }
}
