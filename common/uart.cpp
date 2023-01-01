#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h> // itoa
#include <string.h> // strlen
// #include <stdarg.h> // va_list
#include "uart.h"
#include "misc.h"

UART::UART() {
  #ifndef TEST
    this->init(USART0, PORTA, PIN2_bm, PIN1_bm, 19200);
  #endif
}

UART::UART(USART_t &USART, PORT_t &PORT, uint8_t PIN_RX, uint8_t PIN_TX, uint16_t BPS) {
  #ifndef TEST
    this->init(USART, PORT, PIN_RX, PIN_TX, BPS);
  #endif
}

void UART::init(USART_t &USART, PORT_t &PORT, uint8_t PIN_RX, uint8_t PIN_TX, uint16_t BPS) {
  sei();
  this->USART = &USART;
  this->USART->BAUD = (uint16_t)((float)(F_CPU * 64 / (16 * (float)BPS)) + 0.5);
  this->USART->CTRLB = USART_TXEN_bm;  // enable TX
  PORT.DIRSET = PIN_TX;
  PORT.DIRCLR = PIN_RX;;
  tx_in = 0;
  tx_out = 0;
}

/*
 * waits some time until the serial terminal on the computer is ready
 * then prints out device id
 */
void UART::hello() {
  _delay_ms(600); // the key listener needs some time to start
  DF("\033[1;38;5;18;48;5;226m Hello from 0x%06lX @ %uMHz \033[0m\n", get_deviceid(), (uint8_t)(F_CPU/1000000));
}

/*
 * enabled RX
 * needs the following in the main code:
 *   ISR(USART0_RXC_vect) {
 *     uint8_t in = USART0.RXDATAL;
 *     // do sth with in
 *   }
 */
void UART::enable_rx() {
    this->USART->CTRLB |= USART_RXEN_bm;  // enable RX
    this->USART->CTRLA |= USART_RXCIE_bm; // enable RX interrupt
    DL("\033[1;38;5;18;48;5;46m RX enabled \033[0m");
}

void UART::send_char(unsigned char c) {
  uint8_t next = tx_in;
  rollover(&next, TX_BUFF_SIZE);
  while (next == tx_out);
  tx_buff[tx_in] = c;
  // wait until at least one byte free
  tx_in = next;
  this->USART->CTRLA |= USART_DREIE_bm;
}

void UART::send_string(char *s) {
  while (*s) send_char(*s++);
}

void UART::send_string_p(const char *s) {
  while (pgm_read_byte(s)) send_char(pgm_read_byte(s++));
}

uint8_t UART::is_busy() {
  return (!(this->USART->STATUS & USART_TXCIF_bm));
}

/*
 * convert an uint to a char array
 * eg 345 -> "3.45"
 *
 * returns length of char
 *
 * char out[5];
 * uart.u2c(out, get_temperature(), 2);
 * DF("temp: %sC\n", out);
 *
 * char buf[10] = {0};
 * uint8_t len = uart_u2c(buf, sensor_humidity, 0);
 * buf[len] = '%';
 * buf[len + 1] = '\0';
 * screen.text(buf, 0, 0, 2);
 */
uint8_t UART::u2c(char *buf, uint16_t value, uint8_t precision) {
  itoa(value, buf, 10);
  uint8_t len = strlen(buf);
  uint8_t wrote_dot = 0;

  // ensure char array is at least precision+1 length -> fill it with 0
  int8_t p_read = len - 1;
  uint8_t p_write;
  if (precision == 0) {
    p_write = len - 1;
  } else if (precision >= len) {
    p_write = precision + 1;
  } else {
    p_write = len;
  }
  /*
  DT_I("len", len);
  DT_I("precision", precision);
  DT_I("p_read", p_read);
  DT_I("p_write", p_write);
  */

  if (precision) {
    for (int8_t i=p_write; i>=0; i--) {
      // DF("i:%i p_read:%i\n", i, p_read);
      if (!wrote_dot && precision == 0) {
        wrote_dot = 1;
        buf[i] = '.';
      }
      else if (p_read < 0) {
        buf[i] = '0';
      } else {
        buf[i] = buf[p_read];
        p_read--;
      }

      precision--;
    }
  }
  buf[p_write+1] = '\0';

  return p_write + 1;
}

/*
 * convert seconds to human readable char array
 * max 2^16 = 18h12m16s
 *
 * returns length of char
 */
uint8_t UART::sec2human(char *buf, uint16_t secs) {
  uint8_t minutes = (secs / 60) % 60;
  uint8_t hours = secs / 3600;
  uint8_t seconds = secs % 60;

  if (hours) return snprintf(buf, 10, "%02uh%02um%02us", hours, minutes, seconds);
  else if (minutes)  return snprintf(buf, 7, "%02um%02us", minutes, seconds);
  else return snprintf(buf, 4, "%02us", seconds);
}

/*
 * fast implemention to print arrays
 * (avoid using any printf call)
 */
void UART::arr(const char *name, uint8_t *arr, uint8_t len, uint8_t newline) {
  const char *name_start = name;
  while (*name) send_char(*name++);
  if (name_start != name) {
    send_char(':');
    send_char(' ');
  }

  for (uint8_t i=0; i<len; i++) {
    uint8_t hi = (arr[i] >> 4) & 0xf;
    uint8_t lo = arr[i] & 0xf;
    send_char(hi < 10 ? '0' + hi : ('a' + hi - 10));
    send_char(lo < 10 ? '0' + lo : ('a' + lo - 10));
    if (i<(len-1)) send_char(' ');
  }
  if (newline) send_char('\n');
}

/*
 * increment value and wrap around if > max
 * (for ring buffer)
 */
void UART::rollover(uint8_t *value, uint8_t max) {
  *value = ++*value >= max ? 0 : *value;
}

void UART::rollover(volatile uint8_t *value, uint8_t max) {
  *value = ++*value >= max ? 0 : *value;
}

/*
 * decrement value and wrap around if < 0
 * (for ring buffer)
 */
void UART::rollbefore(uint8_t *value, uint8_t max) {
  *value = *value == 0 ? max - 1 : *value - 1;
}

void UART::rollbefore(volatile uint8_t *value, uint8_t max) {
  *value = *value == 0 ? max - 1 : *value - 1;
}

/*
void UART::DF(const char *format, ...) {
  char buf[120];
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);
  send_string(buf);
}
*/

void UART::D(const char *str) {
  send_string_p(str);
}

/*
 * not sure how to use PSTR. I don't see any benefit in RAM usage,
 * but program gets lower, data stays same, even free_ram()
 */
void UART::DL(const char *str) {
  D(str);
  send_string_p("\n");
}
