#ifndef SPI_H
#define SPI_H

//ATmega8 as slave
#define SPI_DDR    DDRB
#define SPI_PORT   PORTB
#define DD_MOSI    PB3
#define DD_MISO    PB4
#define DD_SCK     PB5
//#define DD_SS      PB2


volatile unsigned char receivedByte;

void initSPI (void);
unsigned char readByteSPI (void);

#endif // SPI_H
