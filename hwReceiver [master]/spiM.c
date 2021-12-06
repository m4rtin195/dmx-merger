/// SPI master

#include <avr/io.h>
#include <avr/interrupt.h>
#include "spiM.h"

#define true 1
#define false 0

volatile uint8_t inprogress;

void initSPI(void)
{
    SPI_PORT |= (1<<DD_SS); //set SS high, inak nepovoli nastavenie MSTR
    SPI_DDR |= (1<<DD_MOSI) | (1<<DD_SCK) | (1<<DD_SS); // SCK, MOSI and SS as outputs
    SPI_DDR &= ~(1<<DD_MISO);                           // MISO as input
    SPI_PORT |= (1 << DD_MISO);                         // enable pull-up on MISO

    SPCR0 |= (1 << SPIE0);      // SPI interrupt enable
    SPCR0 |= (1 << SPE0);       // SPI enable
    SPCR0 |= (1 << MSTR0);      // Set as master
    SPCR0 &= ~(1<< DORD0);      // Data Order MSB first
    //SPSR0 |= (1 << SPI2X0);   // double speed enable

    SPCR0 |= (1<<SPR00);        // divide clock by 64

    inprogress = false;
}

void writeByteSPI(unsigned char byte)
{
    while(inprogress);
    inprogress = true;
    SPI_PORT &= ~(1 << DD_SS);
    SPDR0 = byte;					// Nahraj bajt do SPI dátového registra
    //while(!(SPSR0 & (1<<SPIF0))); 	// Cakaj na ukoncenie prenosu
}

ISR(SPI_STC_vect)
{
    SPI_PORT |=  (1 << DD_SS);
    inprogress = false;
    return;
}

