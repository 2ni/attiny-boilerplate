/*
 * attiny3217
 * creates a DCC signal which can be controlled via uart commands.
 * "help" shows all possible commands
 *
 * make mcu=attiny3217 [nodebug=1] flash
 *
 * we use TCD to create a "pwm" on 2 outputs with 58us for "1" / 116us for "0"
 * the duration is loaded when a cycle starts via ISR
 *
 * WOA: PA4
 * WOB: PA5
 */
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include "string.h"
#include "uart.h"
#include "misc.h"

namespace PRG {
  typedef enum {
    SERVICE = 0,
    OPS = 1,
  } Mode;

  typedef enum {
    RESERVED = 0,
    READ = 0b01,
    WRITE = 0b11,
    BIT = 0b10,
  } Type;
}

namespace SIGNAL {
  typedef enum {
    IDLE = 0,
    DATA = 1,
  } Type;
}

namespace CMD {
  typedef enum {
    NONE,
    TURN_LEFT,
    TURN_RIGHT,
    TOGGLE_IDLE,
    TURN_AUTO,
    TOGGLE_EXTENDED,
    TOGGLE_PRG_MODE,
    PROCESS_UART_CMD,
    PROCESS_UART_DATA_WAITFOR,
    PROCESS_UART_DATA
  } State;
}

#ifdef DEBUG
UART uart;
#else
UARTDUMMY uart;
#endif

// idle packets
#define SIGNAL_IDLE_SIZE_BYTES 5
uint8_t signal_idle[SIGNAL_IDLE_SIZE_BYTES] = { 0xff, 0xf7, 0xf8, 0x01, 0xff };
volatile uint16_t signal_idle_p;
volatile uint8_t fill_with_idle = 0;

// data
// signal buffer must be > max size command to avoid lock in
// eg preamble+packets+separators = 20 + 4*8 + 5 = 57 -> 8
#define SIGNAL_SIZE_BYTES 8
uint8_t signal[SIGNAL_SIZE_BYTES] = { 0 };
uint8_t is_extended_mode = 0;
uint8_t turn_auto_on = 0;
PRG::Mode prg_mode = PRG::SERVICE;
uint8_t is_red = 1;

volatile uint16_t buff_pos_in = 0;
volatile uint16_t buff_pos_out = 0; // TODO needs volatile?

// uart
volatile CMD::State cmd_state = CMD::NONE;
char uart_buff[20];
char uart_data[20];
volatile uint8_t uart_p = 0;
char uart_cmd[6];

uint16_t EEMEM ee_decoder_addr;
volatile uint16_t decoder_addr;

/*****************************************************************************
 * functions
 *****************************************************************************/

void set_timer(uint8_t bit) {
  if (bit) {
    TCD0.CMPASET = 0;
    TCD0.CMPACLR = 580;
    TCD0.CMPBSET = 580;
    TCD0.CMPBCLR = 1160;
  } else {
    TCD0.CMPASET = 0;
    TCD0.CMPACLR = 1160;
    TCD0.CMPBSET = 1160;
    TCD0.CMPBCLR = 2320;
  }
  while (!(TCD0.STATUS & TCD_ENRDY_bm)); // wait for any synch going on
  TCD0.CTRLE = TCD_SYNC_bm; // load values to registers
}

uint8_t get_bit_from_buffer(uint8_t *buffer, uint16_t bit_pos) {
  return (buffer[bit_pos / 8] & (1<<(7-(bit_pos % 8)))) ? 1 : 0;
}

void rollover(uint16_t *value, uint16_t max) {
  *value = ++*value >= max ? 0 : *value;
}

void rollover(volatile uint16_t *value, uint16_t max) {
  *value = ++*value >= max ? 0 : *value;
}

void send_bit(uint8_t bit) {
  uint16_t next = buff_pos_in;
  rollover(&next, SIGNAL_SIZE_BYTES * 8);
  while (next == buff_pos_out); // wait for free space
  /*
  uint8_t ovf_printed = 0;
  while (next == buff_pos_out) {
    if (!ovf_printed) {
      ovf_printed = 1;
      uart.DF("buff ovf: %u %u, %u\n", buff_pos_in, buff_pos_out, TCD0.CTRLA & TCD_ENABLE_bm);
    }
  }

  if (ovf_printed) {
    uart.DL("buff ovf clear");
    ovf_printed = 0;
  }
  */

  uint8_t v = 1<<(7-(buff_pos_in % 8));
  if (bit) {
    signal[buff_pos_in/8] |= v;
  } else {
    signal[buff_pos_in/8] &= ~v;
  }
  buff_pos_in = next;

  // we have data, so activate timer in case it's not yet running
  if (!(TCD0.CTRLA & TCD_ENABLE_bm)) {
    TCD0.CTRLA |= TCD_ENABLE_bm;
  }
}

void send_byte(uint8_t byte) {
  for (uint8_t i=0; i<8; i++) {
    send_bit(byte & (1<<(7-i)) ? 1 : 0);
  }
}

/*
 * convert eg turnout address to module address and port
 * (for basic accessories)
 * see https://wiki.rocrail.net/doku.php?id=addressing:accessory-pg-de
 * addr starts at 1
 * module starts at 1 (except for lenz / roco it starts at 0)
 */
void split_addr(uint16_t addr, uint8_t *module_addr, uint8_t *port, uint8_t is_roco) {
  *module_addr = (addr - 1) / 4 + (is_roco ? 0 : 1);
  *port = (addr - 1) % 4;
}

void send_packet(uint8_t *packets, uint8_t len, PRG::Mode mode = PRG::OPS) {
  // we "activate" the new buffer entries only at the end
  // uint8_t pause = pause_buffer_processing();

  // preamble (ops: 12bits, service: 20bits)
  send_byte(0xff);
  if (mode == PRG::SERVICE) send_byte(0xff);
  for (uint8_t c=0; c<4; c++) send_bit(1);

  // data
  uint8_t x = 0;
  for (uint8_t c=0; c<len; c++) {
    send_bit(0);
    send_byte(packets[c]);
    x ^= packets[c];
  }
  send_bit(0);
  send_byte(x);
  send_bit(1);

  // uart.ARR("packets", packets, len);
  // uart.DF("xor: 0x%02x\n", x);
  // uart.DF("a: 0x%02x, d: 0x%02x, c: 0x%02x\n", addr, data, addr^data);
}

/*
 * Basic accessory decoder 11bit address
 * {preamble} 0 10AAAAAA 0 1AAACDDR 0 EEEEEEEE 1
 * example  weiche / turnout / track switch
 *
 * weiche 5 = module addr (AAAAAAAAA): 2, port (DD): 1 (port 1-4 are sent as 0-3)
 *            0 10000010 0 11110001 (0x82 0xf0)
 *
 * each module_addr (9bit) has 4 ports (1-4)
 *
 * basic (decoder addr)  9bit: {preamble} 0 10AAAAAA 0 1aaaCDDR 0 EEEEEEEE 1
 * basic (output addr)  11bit: {preamble} 0 10AAAAAA 0 1aaaCAAR 0 EEEEEEEE 1
 * extended 11bit:             {preamble} 0 10AAAAAA 0 0aaa0AA1 0 DDDDDDDD 0 EEEEEEEE 1
 *                                          10A7A6A5A4A3A2 0 0a10a9a80A1A01 0
 *
 * A: address bit
 * a: address bit complement
 * C: power/activation (0=inactive, 1=active)
 * D: port (0-3)
 * R: direction/output (0:left/diverting/stop/red, 1:right/straight/run/green)
 *
 */
void basic_accessory(uint16_t addr, uint8_t power, uint8_t direction, uint8_t is_roco = 0) {
  uint8_t packets[2];
  uint8_t module_addr;
  uint8_t port;
  split_addr(addr, &module_addr, &port, is_roco);

  // we use output addressing from the given address
  // the decoder can interpret the incoming data as output or decoder addressing
  packets[0] = 0x80 | (module_addr & 0x3f);
  packets[1] = 0x80 | ((~module_addr & 0x1c0)>>2) | ((port & 0x03)<<1) | (power ? 0x01<<3 : 0) | (direction ? 0x01 : 0);
  send_packet(packets, 2);
  // uart.ARR("packets", packets, 2);
}

/*
 * accessory decoder configuration variable access
 * 3+ reset packets, 5+ command packets, reset packets until got ack/timeout
 *
 * basic/ops     {preamble} 0 10AAAAAA 0 1AAA1AA0 0 1110CCVV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1
 * basic/service {preamble} 0                       0111CCVV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1
 *
 * CC: 00=reserved, 01=verify byte, 11=writy byte, 10=bit manipulation
 * VVVVVVVVVV: 10bit address for CV1-1024
 * DDDDDDDD: data of CV
 *
 * cv_addr: 1-1024
 * mode:     PRG::OPS, PRG::SERVICE
 * cmd_type: PRG::READ, PRG::WRITE, PRG::BIT
 * addr is ignored if PRG::SERVICE
 *
 * longest packet:
 *   service: 25 reset packets + 1 data packet with 4 bytes = 25*(12+3*8+4) + 20+4*8+5 = 1057 ≈ 133bytes
 *   ops: 2 data packets with 6 bytes = 2*(12+6*8+7) = 134bits ≈ 17bytes
 */
void basic_accessory_prg(uint16_t addr, PRG::Mode mode, PRG::Type cmd_type, uint16_t cv_addr, uint8_t cv_data, uint8_t skip_resets = 0, uint8_t is_roco = 0) {
  uint8_t packets[5] = {0};
  uint8_t module_addr;
  uint8_t port;
  uint8_t index = 0;
  uint8_t num = 3;
  uint8_t cmd_start = 0b0111<<4; // service

  // uint8_t pause = pause_buffer_processing();

  // send reset packets 1st (only in service mode before 1st packet)
  if (mode == PRG::SERVICE && !skip_resets) {
    for (uint8_t c=0; c<3; c++) {
      send_packet(packets, 2, mode);
    }
  }

  // add address packets
  split_addr(addr, &module_addr, &port, is_roco);
  if (mode == PRG::OPS) {
    packets[0] = 0x80 | (module_addr & 0x3f);
    packets[1] = 0x88 | ((~module_addr & 0x1c0)>>2) | ((port & 0x03)<<1);
    index += 2;
    num += 2;
    cmd_start = 0b1110<<4;
  }

  cv_addr -= 1; // input: 1-1024
  packets[index] = cmd_start | (cmd_type<<2) | ((cv_addr & 0x300)>>8);
  packets[index+1] = 0xff & cv_addr;
  packets[index+2] = cv_data;
  uint8_t repetitions = 1;
  // service mode requires 5+ repetitions
  if (mode == PRG::SERVICE) repetitions = 5;
  // write requires 2 similar packets while in ops mode (in real env)
  else if (mode == PRG::OPS && (cmd_type == PRG::WRITE || (cmd_type == PRG::BIT && cv_data & 0x10))) repetitions = 2;

  for (uint8_t c=0; c<repetitions; c++) {
    send_packet(packets, num, mode);
  }
  // uart.ARR("prg", packets, num);
  // activate (=send) newest buffer entries
  // resume_buffer_processing(pause);
}

void basic_accessory_prg_bit(uint16_t addr, PRG::Mode mode, uint16_t cv_addr, uint8_t write, uint8_t bit_addr, uint8_t bit_value, uint8_t is_roco = 0) {
  uint8_t cv_data = 0xe0 | (write ? 0x10 : 0x00) | (bit_value ? 0x08 : 0x00) | (0x07 & bit_addr);
  basic_accessory_prg(addr, mode, PRG::BIT, cv_addr, cv_data, 0, is_roco);
}

/*
 * extended accessory decoder 11bit address
 * uses output addressing as in basic accessory
 * {preamble} 0 10AAAAAA 0 0aaa0AA1 0 000XXXXX 0 EEEEEEEE 1
 *              10A7A6A5A4A3A2 0 0a10a9a80A1A01 0
 *
 */
void extended_accessory(uint16_t addr, uint8_t output) {
  uint8_t packets[3];
  uint8_t module_addr;
  uint8_t port;
  split_addr(addr, &module_addr, &port, 0);

  packets[0] = 0x80 | (module_addr & 0x3f);
  packets[1] = 0x01 | ((~module_addr & 0x1c0)>>2) | ((port & 0x03)<<1);
  packets[2] = output;
  send_packet(packets, 3);
}

/*
 * extended accessory decoder configuration variable access
 * basic/ops     {preamble} 0 10AAAAAA 0 0AAA0AA1 0 1110CCVV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1
 * basic/service {preamble} 0                       0111CCVV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1
 *
 * CC: 00=reserved, 01=verify byte, 11=writy byte, 10=bit manipulation
 * VVVVVVVVVV: 10bit address for CV1-1024
 * DDDDDDDD: data of CV
 *
 * cv_addr: 1-1024
 * mode:     PRG::OPS, PRG::SERVICE
 * cmd_type: PRG::READ, PRG::WRITE, (not yet supported: PRG::BIT)
 * addr is ignored if PRG::SERVICE
 */
void extended_accessory_prg(uint16_t addr, PRG::Mode mode, PRG::Type cmd_type, uint8_t cv_addr, uint8_t cv_data, uint8_t skip_resets = 0) {
  basic_accessory_prg(addr, mode, cmd_type, cv_addr, cv_data, skip_resets);
}

void extended_accessory_prg_bit(uint16_t addr, PRG::Mode mode, uint16_t cv_addr, uint8_t write, uint8_t bit_addr, uint8_t bit_value) {
  uint8_t cv_data = 0xe0 | (write ? 0x10 : 0x00) | (bit_value ? 0x08 : 0x00) | (0x07 & bit_addr);
  extended_accessory_prg(addr, mode, PRG::BIT, cv_addr, cv_data);
}


/*****************************************************************************
 * uart functions
 *****************************************************************************/

/*
 * get <num_of_chars> chars before current pointer of ring buffer
 */
uint8_t get_prev_chars(char *buff, char *data, uint8_t num_of_chars) {
  uint8_t len_buff = strlen(buff);
  num_of_chars = num_of_chars > len_buff ? len_buff : num_of_chars;
  uint8_t pos = uart_p;
  for (uint8_t i=0; i<num_of_chars; i++) {
    UART::rollbefore(&pos, 20);
    data[num_of_chars-1-i] = uart_buff[pos];
  }
  data[num_of_chars] = '\0';

  return num_of_chars;
}

void bitwise(char *buf, uint8_t value) {
  for (uint8_t i=0; i<8; i++) {
    buf[i] = value & 0x80 ? '1' : '0';
    value <<= 1;
  }
}

void print_outputs(uint8_t port_outputs) {
  char buf[9];
  bitwise(buf, port_outputs);
  buf[8] = '\0';
  uart.DF("outputs: %s\n", buf);
}

uint16_t _char2int(char *data_in, uint8_t len) {
  uint16_t value = 0;
  uint8_t base = 10;
  uint8_t start = 0;
  // if addr_char 0x1234 -> hex
  if (data_in[1] == 'x') {
    base = 16;
    start = 2;
  }
  // if addr_char 0b10101 -> binary
  else if (data_in[1] == 'b') {
    base = 2;
    start = 2;
  }
  for (uint8_t i=start; i<len; i++) {
    value = value*base + (data_in[i] >= 'a' ? data_in[i] - 'a' + 10 : data_in[i] - '0');
  }

  return value;
}

/*
 * convert char to uint
 * eg 123, 0x123, 0b001
 */
uint16_t buff2int(char *buff) {
  char data_in[19];
  uint8_t l = get_prev_chars(buff, data_in, 18);
  return _char2int(data_in, l);
}

uint16_t char2int(char *buff) {
  return _char2int(buff, strlen(buff));
}

/*
 * isr for tx
 */
ISR(USART0_DRE_vect) {
  uart.isr_tx();
}

/*
 * isr for uart read
 */
ISR(USART0_RXC_vect) {
  uint8_t in = USART0.RXDATAL;
  if (in != '\n') {
    // backspace
    if (in == '\r') {
      UART::rollbefore(&uart_p, 20);
    }
    // any key
    else {
      uart_buff[uart_p] = in;
      UART::rollover(&uart_p, 20);
    }
  }

  if (cmd_state == CMD::NONE) {
    switch (in) {
      case '0'+CMD::TURN_LEFT : cmd_state = CMD::TURN_LEFT; break;
      case '0'+CMD::TURN_RIGHT: cmd_state = CMD::TURN_RIGHT; break;
      case '0'+CMD::TOGGLE_IDLE: cmd_state = CMD::TOGGLE_IDLE; break;
      case '0'+CMD::TURN_AUTO: cmd_state = CMD::TURN_AUTO; break;
      case '0'+CMD::TOGGLE_EXTENDED: cmd_state = CMD::TOGGLE_EXTENDED; break;
      case '0'+CMD::TOGGLE_PRG_MODE: cmd_state = CMD::TOGGLE_PRG_MODE; break;
    }
  }

  if (in == '\n') {
    cmd_state = cmd_state == CMD::PROCESS_UART_DATA_WAITFOR ? CMD::PROCESS_UART_DATA : CMD::PROCESS_UART_CMD;
  }
}

/*
 * set timer duration (pwm signal) according to the according bit of the buffer
 * and point to next bit on buffer
 * buffer can be data or idle buffer
 */
void set_pwm_duration_from_next_bit(SIGNAL::Type type) {
  // handle idle signal
  if (type == SIGNAL::IDLE) {
    TCD0.CTRLA |= TCD_ENABLE_bm;
    set_timer(get_bit_from_buffer(signal_idle, signal_idle_p));
    rollover(&signal_idle_p, SIGNAL_IDLE_SIZE_BYTES * 8);
  // handle "real" data
  } else if (type == SIGNAL::DATA) {
    // timer is activated in send_bit(), as soon as we have data we activate the timer
    // for some reasons, it sometimes stopped to early
    // TCD0.CTRLA |= TCD_ENABLE_bm;
    set_timer(get_bit_from_buffer(signal, buff_pos_out));
    rollover(&buff_pos_out, SIGNAL_SIZE_BYTES * 8);
  }
}

/*
 * pointer always already shows to the future bit
 */
ISR(TCD0_OVF_vect) {
  TCD0.INTFLAGS = TCD_OVF_bm;

  // as long as idle packets iteration not finished -> finish it
  // or if no data and idle packets activated
  if ((signal_idle_p != 0) || (fill_with_idle && buff_pos_in == buff_pos_out)) {
    set_pwm_duration_from_next_bit(SIGNAL::IDLE); // idle packets
    return;
  }
  // no more idle packets and we have data packets -> process them
  else if (buff_pos_in != buff_pos_out) {
    set_pwm_duration_from_next_bit(SIGNAL::DATA); // data packets
    return;
  }

  // no idle packets, no data -> turn off timer
  if (TCD0.CTRLA & TCD_ENABLE_bm) {
    TCD0.CTRLA &= ~TCD_ENABLE_bm;
  }
}

int main(void) {

  setup_clk();

  _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, CLKCTRL_ENABLE_bm | CLKCTRL_RUNSTDBY_bm); // external xtal for 32.768kHz in use with eg attiny3217
  while (CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm);

  PORTMUX.CTRLB = PORTMUX_USART0_ALTERNATE_gc | PORTMUX_SPI0_ALTERNATE_gc; // alternate pins for uart, spi

  flash(PORTB, PIN5_bm); // also sets PB5 as output
  uart.hello();
  uart.enable_rx();

  decoder_addr = eeprom_read_word(&ee_decoder_addr);
  if (decoder_addr == 0xffff) decoder_addr = 0x105; // default 261=0x105
  uart.DF("address in use: %u\n", decoder_addr);

  // setup TCD to create dcc signal (as pwm signal)
  TCD0.CTRLA = TCD_CNTPRES_DIV1_gc | TCD_SYNCPRES_DIV1_gc | TCD_CLKSEL_SYSCLK_gc;
  TCD0.CTRLB = TCD_WGMODE_ONERAMP_gc;
  TCD0.INTCTRL = TCD_OVF_bm;
  _PROTECTED_WRITE(TCD0.FAULTCTRL, TCD_CMPAEN_bm | TCD_CMPBEN_bm);

  set_timer(0); // ensure we have some data initially. If all values 0 and timer is enabled, the isr will direclty disable it again

  // show help on start
  cmd_state = CMD::PROCESS_UART_CMD;
  strncpy(uart_buff, "help", 4);
  uart_p = 4;

  uint8_t repetitions = 0;
  uint8_t port_outputs = 0;
  while (1) {
    switch (cmd_state) {
      case CMD::TURN_LEFT:
        if (is_extended_mode) {
          extended_accessory(decoder_addr, 0);
          if (++repetitions == (fill_with_idle ? 4 : 1)) {
            repetitions = 0;
            cmd_state = CMD::NONE;
          }
        } else {
          basic_accessory(decoder_addr, repetitions < (fill_with_idle ? 4 : 1), 0, 0); // addr, power, direction, is_roco
          if (++repetitions == (fill_with_idle ? 8 : 2)) {
            port_outputs &= ~(1<<7); // clear bit 7
            print_outputs(port_outputs);
            repetitions = 0;
            cmd_state = CMD::NONE;
          }
        }
        break;
      case CMD::TURN_RIGHT:
        if (is_extended_mode) {
          extended_accessory(decoder_addr, 1);
          if (++repetitions == (fill_with_idle ? 4 : 1)) {
            repetitions = 0;
            cmd_state = CMD::NONE;
          }
        } else {
          basic_accessory(decoder_addr, repetitions < (fill_with_idle ? 4 : 1), 1, 0);
          if (++repetitions == (fill_with_idle ? 8 : 2)) {
            port_outputs |= (1 << 7); // set bit 7
            print_outputs(port_outputs);
            repetitions = 0;
            cmd_state = CMD::NONE;
          }
        }
        break;
      case CMD::TOGGLE_IDLE:
        fill_with_idle = !fill_with_idle;
        if (fill_with_idle) {
          set_pwm_duration_from_next_bit(SIGNAL::IDLE);
        }
        uart.DF("(%u) fill with idle: %s\n", CMD::TOGGLE_IDLE, fill_with_idle ? "yes" : "no");
        // rtc_count = 0; // TODO
        cmd_state = CMD::NONE;
        break;
      case CMD::TURN_AUTO:
        if (turn_auto_on == 1) turn_auto_on = 2;
        else if (!turn_auto_on) turn_auto_on = 1;
        uart.DF("(%u) autoturn: %s\n", CMD::TURN_AUTO, turn_auto_on == 2 && !is_red ? "turning off" : (turn_auto_on ? "yes" : "no"));
        cmd_state = CMD::NONE;
        break;
      case CMD::TOGGLE_EXTENDED:
        is_extended_mode = !is_extended_mode;
        uart.DF("(%u) mode: %s\n", CMD::TOGGLE_EXTENDED, is_extended_mode ? "extended" : "basic");
        cmd_state = CMD::NONE;
        break;
      case CMD::TOGGLE_PRG_MODE:
        prg_mode = prg_mode == PRG::OPS ? PRG::SERVICE : PRG::OPS;
        uart.DF("(%u) prg mode: %s\n", CMD::TOGGLE_PRG_MODE, prg_mode == PRG::OPS ? "ops" : "service");
        cmd_state = CMD::NONE;
        break;
      case CMD::PROCESS_UART_CMD:
        if (get_prev_chars(uart_buff, uart_cmd, 4) && !strcmp(uart_cmd, "help")) {
          uart.DL("usage:");
          uart.DF("  %u : turn left/diverting/red\n", CMD::TURN_LEFT);
          uart.DF("  %u : turn right/straight/green\n", CMD::TURN_RIGHT);
          uart.DF("  %u : %s|%s idle packets in between (toggle)\n", CMD::TOGGLE_IDLE, fill_with_idle ? OK("yes") : "yes", fill_with_idle ? "no": OK("no"));
          uart.DF("  %u : %s|%s autoturn (toggle)\n", CMD::TURN_AUTO, turn_auto_on ? OK("yes") : "yes", turn_auto_on ? "no": OK("no"));
          uart.DF("  %u : %s|%s mode (toggle)\n", CMD::TOGGLE_EXTENDED, is_extended_mode ? "basic" : OK("basic"), is_extended_mode ? OK("extended") : "extended");
          uart.DF("  %u : %s|%s prg mode (toggle)\n", CMD::TOGGLE_PRG_MODE, prg_mode == PRG::OPS ? "service" : OK("service"), prg_mode == PRG::OPS ? OK("ops") : "ops");
          uart.DF("  saddr: " OK("%u/0x%03x") " address cmds are sent to\n", decoder_addr, decoder_addr);
          uart.DL("  caddr: change device addr");
          uart.DL("  cv  : verify/write cv");
          uart.DL("  cvb : verify/write cv bit");
          uart.DL("  help: show this help");
          cmd_state = CMD::NONE;
        }
        else if (get_prev_chars(uart_buff, uart_cmd, 5) && !strcmp(uart_cmd, "caddr")) {
          uart.D("new addr for device: ");
          cmd_state = CMD::PROCESS_UART_DATA_WAITFOR;
        }
        else if (get_prev_chars(uart_buff, uart_cmd, 5) && !strcmp(uart_cmd, "saddr")) {
          uart.D("new addr cmds are sent to: ");
          cmd_state = CMD::PROCESS_UART_DATA_WAITFOR;
        }
        else if (get_prev_chars(uart_buff, uart_cmd, 2) && !strcmp(uart_cmd, "cv")) {
          uart.D("<cv addr>[r|w]<cv value> (eg 121r0x05): ");
          cmd_state = CMD::PROCESS_UART_DATA_WAITFOR;
        }
        else if (get_prev_chars(uart_buff, uart_cmd, 3) && !strcmp(uart_cmd, "cvb")) {
          uart.D("<cv addr>[r|w]<bitpos><bitvalue> (eg 121r71): ");
          cmd_state = CMD::PROCESS_UART_DATA_WAITFOR;
        }
        else {
          cmd_state = CMD::NONE;
        }
        // clear ring buffer
        uart_p = 0;
        for (uint8_t ii=0; ii<20; ii++) uart_buff[ii] = 0;
        break;
      case CMD::PROCESS_UART_DATA:
        get_prev_chars(uart_buff, uart_data, 20);
        // process cmd with data
        if (!strcmp(uart_cmd, "caddr")) {
          uint16_t addr = char2int(uart_data);
          // lsb
          if (is_extended_mode) {
            extended_accessory_prg(decoder_addr, prg_mode, PRG::WRITE, 121, addr & 0xff);
          } else {
            basic_accessory_prg(decoder_addr, prg_mode, PRG::WRITE, 121, addr & 0xff);
          }
          if (prg_mode == PRG::SERVICE) {
            // 1 reset 7.772ms -> 14 packets
            // wait 100ms filling with reset (=12.86670098 reset packets) as we can't read ack from decoder yet
            uint8_t packets[2] = {0};
            for (uint8_t i=0; i<13; i++) {
              send_packet(packets, 2);
            }
          }
          // msb (skip intro reset packets in service mode, none in ops mode anyways)
          if (is_extended_mode) {
            extended_accessory_prg(decoder_addr, prg_mode, PRG::WRITE, 120, (addr >> 8) & 0xff);
          } else {
            basic_accessory_prg(decoder_addr, prg_mode, PRG::WRITE, 120, (addr >> 8) & 0xff, 1);
          }
          uart.DF("\nnew addr: %u/0x%03x\n", addr, addr);
        }
        else if (!strcmp(uart_cmd, "saddr")) {
          uint16_t addr = buff2int(uart_data);
          eeprom_update_word(&ee_decoder_addr, addr);
          decoder_addr = addr;
          uart.DF("address in use: %u (0x%04x)\n", decoder_addr, decoder_addr);
        }
        else if (!strcmp(uart_cmd, "cv")) {
          // uart_data = "121r0x05" cv121 compare with 0x05
          char cv_addr_char[5] = {0};
          uint8_t cv_addr_pos = 0;
          uint8_t rw_seen = 0;
          char cv_value_char[10] = {0};
          uint8_t cv_value_pos = 0;
          PRG::Type cmd_type = PRG::RESERVED;
          for (uint8_t i=0; i<strlen(uart_data); i++) {
            if (uart_data[i] == 'r' || uart_data[i] == 'w') {
              rw_seen = 1;
              cmd_type = uart_data[i] == 'r' ? PRG::READ : PRG::WRITE;
            } else if (rw_seen == 0) {
              cv_addr_char[cv_addr_pos++] = uart_data[i];
            } else {
              cv_value_char[cv_value_pos++] = uart_data[i];
            }
          }

          uint16_t cv_addr = char2int(cv_addr_char);
          uint8_t cv_value = char2int(cv_value_char);
          uart.DF("%s cv#%u: 0x%02x\n", cmd_type == PRG::WRITE ? "write" : "read", cv_addr, cv_value);
          if (is_extended_mode) {
            extended_accessory_prg(decoder_addr, prg_mode, cmd_type, cv_addr, cv_value);
          } else {
            basic_accessory_prg(decoder_addr, prg_mode, cmd_type, cv_addr, cv_value);
          }
        }
        else if (!strcmp(uart_cmd, "cvb")) {
          // uart_data = "121r70" cv121 compare bit 7 with 0
          char cv_addr_char[5] = {0};
          uint8_t cv_addr_pos = 0;
          uint8_t rw_seen = 0;
          uint8_t bit_addr = 0;
          uint8_t bit_value = 0;
          uint8_t write = 0;
          for (uint8_t i=0; i<strlen(uart_data); i++) {
            if (uart_data[i] == 'r' || uart_data[i] == 'w') {
              rw_seen = 1;
              write = uart_data[i] == 'w' ? 1 : 0;
            } else if (rw_seen == 0) {
              cv_addr_char[cv_addr_pos++] = uart_data[i];
            } else {
              bit_addr = uart_data[i] - '0';
              bit_value = uart_data[i+1] - '0';
              break;
            }
          }
          uint16_t cv_addr = char2int(cv_addr_char);
          uart.DF("%s cv#%u: bit %u, value %u\n", write == 1 ? "write" : "read", cv_addr, bit_addr, bit_value);
          if (is_extended_mode) {
            extended_accessory_prg_bit(decoder_addr, prg_mode, cv_addr, write, bit_addr, bit_value);
          } else {
            basic_accessory_prg_bit(decoder_addr, prg_mode, cv_addr, write, bit_addr, bit_value);
          }
        }

        uart_data[0] = '\0';
        // clear ring buffer
        uart_p = 0;
        for (uint8_t ii=0; ii<20; ii++) uart_buff[ii] = 0;
        cmd_state = CMD::NONE;
        break;

      default:
        break;
    }
  }
}
