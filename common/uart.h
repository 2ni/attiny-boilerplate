#ifndef __UART_H__
#define __UART_H__

#include <avr/io.h>
#include <stdio.h>

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

/*
 * this class can be used if eg we want to toggle all the output
 * without the need to change/remove all output manually
 *
 *   #ifdef DEBUG
 *     UART uart;
 *   #else
 *     UARTDUMMY uart;
 *   #endif
 */
class UARTDUMMY {
  public:
    UARTDUMMY() {}
    UARTDUMMY(USART_t &USART, PORT_t &PORT, uint8_t PIN_RX, uint8_t PIN_TX, uint16_t BPS) {}
    void     init(USART_t &USART, PORT_t &PORT, uint8_t PIN_RX, uint8_t PIN_TX, uint16_t BPS=19200) {}
    void     init() {}
    void     hello() {}
    void     enable_rx() {}
    uint8_t  is_busy() { return 0; }
    uint8_t  u2c(char *buf, uint16_t value, uint8_t precision=2) { return 0; }
    uint8_t  sec2human(char *buf, uint16_t seconds) { return 0; }
    void     isr_tx() {}
    void     ARR(const char *name, uint8_t *arr, uint8_t len, uint8_t newline = 1) {}
    void     D(const char *str) {}
    void     DL(const char *str) {}
    template<typename... Args>
    void     DF(const char *format, Args... args) {}
};

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
    void     isr_tx();
    // these functions can also be used outside of the uart class in combination with buffers
    static void     rollbefore(uint8_t *value, uint8_t max);
    static void     rollbefore(volatile uint8_t *value, uint8_t max);
    static void     rollover(uint8_t *value, uint8_t max);
    static void     rollover(volatile uint8_t *value, uint8_t max);
    // void     DF(const char *format, ...);
    void     ARR(const char *name, uint8_t *arr, uint8_t len, uint8_t newline = 1);
    void     D(const char *str);
    void     DL(const char *str);
    template<typename... Args>
    void     DF(const char *format, Args... args);

  private:
    void     send_char(unsigned char c);
    void     send_string(char *s);
    void     send_string(const char *s);

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
