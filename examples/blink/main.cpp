/*
 * mcu: attiny3217
 * blink PB5 at 500ms using _delay_ms()
 * outputs text to alternate uart to PA1 (txd). PA2 (rxd) can also be use if needed
 *
 * make mcu=attiny3217 flash
 */
#include <util/delay.h>
#include <avr/io.h>
#include <avr/pgmspace.h> // for PSTR
#include <avr/interrupt.h>
#include "uart.h"
#include "misc.h"

UART uart;

/*
 * isr for tx
 */
ISR(USART0_DRE_vect) {
  uart.isr_tx();
}

int main(void) {
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm); // 10MHz
  _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, CLKCTRL_ENABLE_bm | CLKCTRL_RUNSTDBY_bm); // external xtal for 32.768kHz in use with eg attiny3217
  while (CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm);

  PORTMUX.CTRLB = PORTMUX_USART0_ALTERNATE_gc | PORTMUX_SPI0_ALTERNATE_gc; // alternate pins for uart, spi

  flash(PORTB, PIN5_bm); // also sets PB5 as output
  uart.hello();

  while (1) {
    PORTB.OUTTGL = PIN5_bm;
    _delay_ms(500);
    uart.DL(PSTR("blink"));
  }
}
