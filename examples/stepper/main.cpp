/*
 * make mcu=attiny1604 port=1 flash
 *
 * hall sensors
 * LIMIT1 = PB1 (position = 1, left) (move to here: -1500)
 * LIMIT2 = PB0 (position = 0, right) (move to here: 1500)
 *
 * stepper:
 * INA1 = PA7
 * INA2 = PA6
 * INB1 = PB2
 * INB2 = PB3
*/

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h> // for PSTR
#include "uart.h"
#include "stepper.h"
#include "misc.h"

UART uart;
STEPPER stepper;
uint32_t current_tick = 0;
volatile uint8_t limit = 0;

/*
 * isr for uart tx
 */
ISR(USART0_DRE_vect) {
  uart.isr_tx();
}

/*
 * isr for uart rx
 */
ISR(USART0_RXC_vect) {
  char in = USART0.RXDATAL;
  switch (in) {
    case '0': // move right possible if sensor right != 0
      if ((PORTB.IN & PIN0_bm) != 0) {
        stepper.move(100, 1);
      }
      break;
    case '1': // move left
      if ((PORTB.IN & PIN1_bm) != 0) {
        stepper.move(-100, 1);
      }
      break;
  }
}

/*
 * isr for rtc tick every ~1ms
 */
ISR(RTC_CNT_vect) {
  RTC.INTFLAGS = RTC_OVF_bm;
  current_tick++;
}

/*
 * isr to detect limits (hall sensors)
 */
ISR(PORTB_PORT_vect) {
  uint8_t flags = PORTB.INTFLAGS;
  PORTB.INTFLAGS = flags; // clear flags
  if (flags & (PORT_INT1_bm | PORT_INT0_bm)) {
    uint8_t position = (flags & PORT_INT1_bm) ? 1 : 0;
    uint8_t limitactive = (~PORTB.IN & (PIN1_bm | PIN0_bm)) ? 1 : 0;

    if (limitactive) {
      stepper.stop();
      if (position) {
        PORTA.OUTSET = PIN3_bm;
        PORTA.OUTCLR = PIN4_bm;
      } else {
        PORTA.OUTSET = PIN4_bm;
        PORTA.OUTCLR = PIN3_bm;
      }
    } else {
      PORTA.OUTCLR = PIN4_bm;
      PORTA.OUTCLR = PIN3_bm;
    }
  }
}

int main(void) {
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm); // 10MHz

  PORTMUX.CTRLB = PORTMUX_USART0_ALTERNATE_gc;

  uart.hello();
  uart.enable_rx();

  PORTA.DIRSET = PIN4_bm; // set PA4 (led green) as output
  PORTA.DIRSET = PIN3_bm; // set PA3 (led red) as output
  PORTB.PIN1CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc; // limits, detect when limit reached and left
  PORTB.PIN0CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;

  // blink 3x
  uint8_t num = 3;
  for (uint8_t c=0; c<num; c++) {
    PORTA.OUTSET = PIN3_bm;
    _delay_ms(20);
    PORTA.OUTCLR = PIN3_bm;
    if (c != num-1) _delay_ms(100);
  }

  // rtc isr running every ~1ms (= 32*1000/32768 ms)
  while (RTC.STATUS > 0);
  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;
  RTC.INTCTRL = RTC_OVF_bm;
  RTC.CNT = 0;
  RTC.PER = 32;
  RTC.CTRLA = RTC_PRESCALER_DIV1_gc | RTC_RTCEN_bm | RTC_RUNSTDBY_bm; // 32768/32 = 1024Hz = 0.9765625ms

  if (~PORTB.IN & (PIN1_bm | PIN0_bm)) {
    uint8_t position = (~PORTB.IN & PIN1_bm) ? 1 : 0;
    if (position) {
      PORTA.OUTSET = PIN3_bm;
      PORTA.OUTCLR = PIN4_bm;
    } else {
      PORTA.OUTSET = PIN4_bm;
      PORTA.OUTCLR = PIN3_bm;
    }
  }

  uart.DL(PSTR("1=left, 0=right"));
  stepper.init(PORTA, PIN7_bm, PIN6_bm, PORTB, PIN2_bm, PIN3_bm, &current_tick);
  while (1) {
    stepper.loop();
  }
}
