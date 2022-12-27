/*
 * pin configuration settings for forgetmenot
 */

#ifndef __STEPPER_H__
#define __STEPPER_H__

#include <avr/io.h>

class STEPPER {
  public:
    STEPPER();
    void init(PORT_t &PA, uint8_t INA1, uint8_t INA2, PORT_t &PB, uint8_t INB1, uint8_t INB2, uint32_t *current_tick);
    void set_step(uint8_t step);
    void stop();
    void keep();
    void move(int16_t steps, uint8_t ticks);
    void loop(void (*fn)() = 0);

  private:
    void move_one_step(int8_t direction);
    PORT_t *PA;
    PORT_t *PB;
    uint8_t INA1, INA2, INB1, INB2;

    int8_t current_step, direction;
    uint8_t speed;
    int16_t steps_left;
    uint32_t last_tick;
    uint32_t *current_tick;
};

#endif
