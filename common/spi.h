#ifndef __SPI_H__
#define __SPI_H__

#include <avr/io.h>

class SPI {
  public:
    SPI();
    SPI(SPI_t &SPI_OBJECT, PORT_t &PORT, uint8_t PIN_MOSI, uint8_t PIN_MISO, uint8_t PIN_SCK);
    void    set_cs(PORT_t &PORT_CS, uint8_t PIN_CS);
    void    transfer(uint8_t *out, uint8_t *in, uint8_t len);
    void    send(uint8_t *out, uint8_t len);
    uint8_t transfer_byte(uint8_t data);
    void    set_mode(uint8_t mode);
    void    select();
    void    deselect();

  private:
    void    init(SPI_t &SPI_OBJECT, PORT_t &PORT, uint8_t PIN_MOSI, uint8_t PIN_MISO, uint8_t PIN_SCK);

    SPI_t *SPI_OBJECT;
    PORT_t *PORT_CS;
    uint8_t PIN_CS;
};

#endif
