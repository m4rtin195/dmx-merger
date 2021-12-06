#ifndef SPI_H
#define SPI_H

//ATmega324 as master
#define SPI_DDR    DDRB
#define SPI_PORT   PORTB
#define DD_MOSI    PB5
#define DD_MISO    PB6
#define DD_SCK     PB7
#define DD_SS      PB4

volatile unsigned char receivedByte;

void initSPI (void);
void writeByteSPI (unsigned char byte);
//unsigned char readByteSPI (void);

#endif // SPI_H
