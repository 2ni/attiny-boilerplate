#ifndef __UART_H__
#define __UART_H__

#include <avr/io.h>
#include <stdio.h>
#include <avr/pgmspace.h> // for PSTR

// can be overwritten, if set before calling "uart.h"
#ifndef TX_BUFF_SIZE
  #define TX_BUFF_SIZE 16
#endif

/*
 * https://stackoverflow.com/questions/4842424/list-of-ansi-color-escape-sequences
 * https://stackoverflow.com/questions/3219393/stdlib-and-colored-output-in-c
 *
 * eg uart.DF(NOK("Failures!!!: %u/%u.") "\n", number_of_failed, number_of_tests);
 */
#define NOK(str)  "\033[31;1m" str "\033[0m"  // output in red
#define OK(str)   "\033[32;1m" str "\033[0m"  // output in green
#define WARN(str) "\033[33;1m" str "\033[0m"  // output in yellow
#define BOLD(str) "\033[1m" str "\033[0m"     // output bold

class UART {
  public:
    UART();
    UART(USART_t &USART, PORT_t &PORT, uint8_t PIN_RX, uint8_t PIN_TX, uint16_t BPS);
    void     init(USART_t &USART, PORT_t &PORT, uint8_t PIN_RX, uint8_t PIN_TX, uint16_t BPS=19200);
    void     init();
    void     hello();
    void     enable_rx();
    uint8_t  is_busy();
    uint8_t  u2c(char *buf, uint16_t value, uint8_t precision=2);
    uint8_t  sec2human(char *buf, uint16_t seconds);
    void     arr(const char *name, uint8_t *arr, uint8_t len, uint8_t newline = 0);
    void     isr_tx();
    // void     DF(const char *format, ...);
    void     D(const char *str);
    void     DL(const char *str);
    template<typename... Args>
    void     DF(const char *format, Args... args);

  private:
    void     send_char(unsigned char c);
    void     send_string(char *s);
    void     send_string_p(const char *s);
    void     rollover(uint8_t *value, uint8_t max);
    void     rollover(volatile uint8_t *value, uint8_t max);
    void     rollbefore(uint8_t *value, uint8_t max);
    void     rollbefore(volatile uint8_t *value, uint8_t max);

    uint8_t tx_buff[TX_BUFF_SIZE];
    uint8_t tx_in; // pointer of filling buffer
    volatile uint8_t tx_out; // pointer of sending
    USART_t *USART;
};

template<typename... Args>
void UART::DF(const char *format, Args... args) {
  char buf[100];
  snprintf(buf, sizeof(buf), format, args...);
  send_string(buf);
}

inline void UART::isr_tx() {
  // nothing to send
  if (this->tx_in == this->tx_out) {
    this->USART->CTRLA &= ~USART_DREIE_bm;
    return;
  }

  this->USART->TXDATAL = tx_buff[tx_out];
  this->USART->STATUS = USART_TXCIF_bm; // clear txcif
  this->rollover(&tx_out, TX_BUFF_SIZE);
}

#endif
