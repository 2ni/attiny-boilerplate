#include <util/delay.h>
#include <avr/io.h>
#include "uart.h"
#include "stepper.h"
#include <avr/interrupt.h>

STEPPER stepper;
uint32_t current_tick = 0;

ISR(RTC_CNT_vect) {
  RTC.INTFLAGS = RTC_OVF_bm;
  current_tick++;
}

int main(void) {
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm); // 10MHz

  PORTMUX.CTRLB = PORTMUX_USART0_ALTERNATE_gc | PORTMUX_SPI0_ALTERNATE_gc; // alternate pins for uart, spi
  SLPCTRL.CTRLA = (SLPCTRL_SMODE_STDBY_gc | SLPCTRL_SEN_bm);

  uart_init();

  PORTA.DIRSET = PIN4_bm; // set PB5 (led) as output

  // blink 3x
  uint8_t num = 3;
  for (uint8_t c=0; c<num; c++) {
    PORTA.OUTSET = PIN4_bm;
    _delay_ms(20);
    PORTA.OUTCLR = PIN4_bm;
    if (c != num-1) _delay_ms(100);
  }

  // rtc isr running every ~1ms (= 32*1000/32768 ms)
  while (RTC.STATUS > 0);
  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;
  RTC.INTCTRL = RTC_OVF_bm;
  RTC.CNT = 0;
  RTC.PER = 32;
  RTC.CTRLA = RTC_PRESCALER_DIV1_gc | RTC_RTCEN_bm | RTC_RUNSTDBY_bm; // 32768/32 = 1024Hz = 0.9765625ms

  stepper.init(PORTA, PIN7_bm, PIN6_bm, PORTB, PIN2_bm, PIN3_bm, &current_tick);
  // moves stepper by 100 steps
  stepper.move(100, 1);
  while (1) {
    stepper.loop();
  }
}
