/*
 * mcu: attiny3217
 * communicte with uart
 *
 * make mcu=attiny3217 flash
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "misc.h"

// UART uart(USART0, PORTA, PIN2_bm, PIN1_bm, 19200);
UART uart;

/*
 * isr for tx
 */
ISR(USART0_DRE_vect) {
  uart.isr_tx();
}

/*
 * isr for rx
 * usually incoming data should be saved in a buffer
 * and then be processed in the main loop
 */
ISR(USART0_RXC_vect) {
  uint8_t in = USART0.RXDATAL;
  switch (in) {
    case '1':
      uart.DL("1");
      break;
    case '0':
      uart.DL("0");
      break;
  }
}

int main(void) {
  setup_clk();

  _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, CLKCTRL_ENABLE_bm | CLKCTRL_RUNSTDBY_bm); // external xtal for 32.768kHz
  while (CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm);

  PORTMUX.CTRLB = PORTMUX_USART0_ALTERNATE_gc | PORTMUX_SPI0_ALTERNATE_gc; // alternate pins for uart, spi

  flash(PORTB, PIN5_bm);
  uart.hello();
  uart.enable_rx();

  uint8_t c = 254;
  uart.DF("c: %u\n", c);
  uart.DF("hex: 0x%02x\n", c);
  uart.DL(PSTR("hello"));
  while (1);
}
