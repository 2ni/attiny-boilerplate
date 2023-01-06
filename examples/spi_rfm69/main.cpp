/*
 * mcu: attiny3217
 * communicate with rfm69 via spi
 * PC0: SCK
 * PC1: MISO
 * PC2: MOSI
 * PC3: CS
 *
 * make mcu=attiny3217 flash
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "spi.h"
#include "misc.h"

#ifdef DEBUG
  UART uart;
#else
  UARTDUMMY uart;
#endif

SPI spi;

/*
 * isr for tx
 */
ISR(USART0_DRE_vect) {
  uart.isr_tx();
}

uint8_t read_reg(uint8_t addr) {
  spi.select();
  spi.transfer_byte(addr & 0x7f);
  uint8_t val = spi.transfer_byte(0);
  spi.deselect();
  return val;
}

int main(void) {
  setup_clk();

  _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, CLKCTRL_ENABLE_bm | CLKCTRL_RUNSTDBY_bm); // external xtal for 32.768kHz
  while (CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm);

  PORTMUX.CTRLB = PORTMUX_USART0_ALTERNATE_gc | PORTMUX_SPI0_ALTERNATE_gc; // alternate pins for uart, spi

  flash(PORTB, PIN5_bm);
  uart.hello();

  spi.set_cs(PORTC, PIN3_bm);

  uint8_t version = read_reg(0x10);
  uart.DF("version: 0x%02x\n", version);
  uart.DL(version == 0x24 ? OK("ok") : NOK("failed"));
  while (1);

}
