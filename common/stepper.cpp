#include "stepper.h"

STEPPER::STEPPER() {
}

void STEPPER::init(PORT_t &PA, uint8_t INA1, uint8_t INA2, PORT_t &PB, uint8_t INB1, uint8_t INB2, uint32_t *current_tick) {
  this->PA = &PA;
  this->PA->DIR |= INA1 | INA2;
  this->INA1 = INA1;
  this->INA2 = INA2;

  this->PB = &PB;
  this->PB->DIR |= INB1 | INB2;
  this->INB1 = INB1;
  this->INB2 = INB2;

  this->current_tick = current_tick;
  direction = 0;
  current_step = 0;
  steps_left = 0;
}

void STEPPER::set_step(uint8_t step) {
  switch (step) {
    case 0: // 1010
      PA->OUTSET = INA1;
      PA->OUTCLR = INA2;
      PB->OUTSET = INB1;
      PB->OUTCLR = INB2;
      break;
    case 1: // 0110
      PA->OUTCLR = INA1;
      PA->OUTSET = INA2;
      PB->OUTSET = INB1;
      PB->OUTCLR = INB2;
      break;
    case 2: // 0101
      PA->OUTCLR = INA1;
      PA->OUTSET = INA2;
      PB->OUTCLR = INB1;
      PB->OUTSET = INB2;
      break;
    case 3: // 1001
      PA->OUTSET = INA1;
      PA->OUTCLR = INA2;
      PB->OUTCLR = INB1;
      PB->OUTSET = INB2;
      break;
  }
}

void STEPPER::move_one_step(int8_t direction) {
  current_step += direction;
  if (direction == 1 && current_step == 4) current_step = 0;
  if (direction == -1 && current_step == -1) current_step = 3;

  set_step(current_step);
}

void STEPPER::stop() {
  PA->OUT &= ~(INA1 | INA2);
  PB->OUT &= ~(INB1 | INB2);
  steps_left = 0;
}

/*
 * activates motor coil without moving
 * used eg to keep position with power or
 * eg just to consume some power for ack on DCC signals
 */
void STEPPER::keep() {
  set_step(current_step);
}

/*
 * speed is given in number of ticks,
 * the lower, the faster
 * ie amount of time to wait between the moves
 *   STEPPER stepper;
 *   uint32_t current_tick = 0;

 *   ISR(RTC_CNT_vect) {
 *     RTC.INTFLAGS = RTC_OVF_bm;
 *     current_tick++;
 *   }
 *
 *   while (1) {
 *     // rtc isr running every ~1ms (= 32*1000/32768 ms)
 *     while (RTC.STATUS > 0);
 *     RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;
 *     RTC.INTCTRL = RTC_OVF_bm;
 *     RTC.CNT = 0;
 *     RTC.PER = 32;
 *     RTC.CTRLA = RTC_PRESCALER_DIV1_gc | RTC_RTCEN_bm | RTC_RUNSTDBY_bm; // 32768/32 = 1024Hz = 0.9765625ms

 *     stepper.init(PORTA, PIN7_bm, PIN6_bm, PORTB, PIN2_bm, PIN3_bm, &current_tick);
 *     stepper.move(100, 1);
 *   }
 */
void STEPPER::move(int16_t steps, uint8_t speed) {
  steps_left = steps;
  this->speed = speed;
  last_tick = *current_tick;
}

void STEPPER::loop(void (*fn)()) {
  // move as long as steps_left <> 0
  if (steps_left && (*current_tick - last_tick) > speed) {
    last_tick = *current_tick;
    move_one_step(steps_left > 0 ? 1 : -1);
    steps_left = steps_left + (steps_left > 0 ? -1 : 1);

    if (!steps_left) {
      if (fn) (*fn)(); // callback
      stop();
    }
  }
}
