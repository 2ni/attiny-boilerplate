#include <avr/io.h>
#include "spi.h"

SPI::SPI() {
  #ifndef TEST
    this->init(SPI0, PORTC, PIN2_bm, PIN1_bm, PIN0_bm);
  #endif
}

SPI::SPI(SPI_t &SPI_OBJECT, PORT_t &PORT, uint8_t PIN_MOSI, uint8_t PIN_MISO, uint8_t PIN_SCK) {
  #ifndef TEST
    this->init(SPI_OBJECT, PORT, PIN_MOSI, PIN_MISO, PIN_SCK);
  #endif
}

void SPI::init(SPI_t &SPI_OBJECT, PORT_t &PORT, uint8_t PIN_MOSI, uint8_t PIN_MISO, uint8_t PIN_SCK) {
  PORT.DIR |= PIN_MOSI | PIN_SCK;
  PORT.DIRCLR = PIN_MISO;

  this->SPI_OBJECT = &SPI_OBJECT;
  this->SPI_OBJECT->CTRLB = SPI_BUFEN_bm | SPI_BUFWR_bm | SPI_SSD_bm | SPI_MODE_0_gc; // ignore SS pin setting for master, set spi mode 0
  this->SPI_OBJECT->CTRLA = SPI_ENABLE_bm | SPI_MASTER_bm | SPI_PRESC_DIV4_gc; // MSB first, master mode, enable spi, no double clk, prescaler
  this->SPI_OBJECT->DATA; // empty rx
}

/*
 * send / receive data in full-duplex
 */
void SPI::transfer (uint8_t *out, uint8_t *in, uint8_t len) {
  for (uint8_t i=0; i<len; i++) {
    this->SPI_OBJECT->DATA = out[i];

    while (!(this->SPI_OBJECT->INTFLAGS & SPI_IF_bm));
    in[i] =  this->SPI_OBJECT->DATA;
  }
}

/*
 * send data only, ignore receiving data
 */
void SPI::send (uint8_t *out, uint8_t len) {
  for (uint8_t i=0; i<len; i++) {
    this->SPI_OBJECT->DATA = out[i];

    while (!(this->SPI_OBJECT->INTFLAGS & SPI_IF_bm));
  }
}

/*
 * send / receive  1 byte full-duplex
 */
uint8_t SPI::transfer_byte (uint8_t data) {
  this->SPI_OBJECT->DATA = data;

  while (!(this->SPI_OBJECT->INTFLAGS & SPI_TXCIF_bm));
  this->SPI_OBJECT->INTFLAGS |= SPI_TXCIF_bm; // clear flag
  return this->SPI_OBJECT->DATA;
}

/*
 * set spi mode 0, 1, 2 or 3
 */
void SPI::set_mode(uint8_t mode) {
  if (mode > 3) return;

  this->SPI_OBJECT->CTRLB |=  mode;
}

/*
 * set chip select
 * can also be done outside if we have eg multiple spi slaves with same spi settings
 */
void SPI::set_cs(PORT_t &PORT_CS, uint8_t PIN_CS) {
  this->PORT_CS = &PORT_CS;
  this->PIN_CS = PIN_CS;
  this->PORT_CS->DIRSET = PIN_CS;
  this->PORT_CS->OUTSET = PIN_CS;
}

void SPI::select() {
  this->PORT_CS->OUTCLR = this->PIN_CS;
}

void SPI::deselect() {
  this->PORT_CS->OUTSET = this->PIN_CS;
}
