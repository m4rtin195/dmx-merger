/// SPI slave

#include <avr/io.h>
#include <avr/interrupt.h>
#include "spiS.h"

void initSPI(void)
{
    SPI_DDR |= (1<<DD_MISO);    // set MISO as output
    SPI_DDR &= ~((1 << DD_SCK) | (1 << DD_MOSI) | (1 << DD_SS)); // set other as inputs
    SPI_PORT |= (1 << DD_SCK) | (1 << DD_MOSI) | (1 << DD_SS) ;  // set pull-ups

    SPCR |= (1<<SPE);       // SPI enable
    SPCR |= (1<<SPIE);      // SPI interrupt enable
    SPCR &= ~(1<<MSTR);     // Set as slave
    SPCR &= ~(1<<DORD);     // Data Order MSB first

}

unsigned char readByteSPI(void)
{
	//while(!(SPSR & (1<<SPIF))); 	// cakaj na ukoncenie prenosu
	//return SPDR;                    // prijate data
	return receivedByte;
}

ISR(SPI_STC_vect)
{
    PORTD |= (1 << PD7);
    receivedByte = SPDR;
}
