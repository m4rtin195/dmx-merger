#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <limits.h>

#include "defines8.h"


/// /////////////// DMX CONTROLLER ///////////////// ///

#define Calcprescale(B)     ( ( (((F_CPU)/16)/(B)) - 1 )  )


uint8_t  dmxData[DMXSERIAL_MAX+1];  // Entry 0 will never be used for DMX data but will store the startbyte
int     _dmxChannel;                // the next channel byte to be sent.
bool    tx_enabled;

/** ----- forwards ----- **/

void _DMXSerialSetBaud(uint16_t baud_setting, uint8_t format);
void _DMXSerialWriteByte(uint8_t data);


void DMX_Controller_init()
{
  // initialize global variables
  _dmxChannel = 0;

  // initialize the DMX buffer
  for (int n = 0; n < DMXSERIAL_MAX+1; n++)
    dmxData[n] = 0;

  // Enable transmitter and interrupt
  UCSRB = (1<<TXEN) | (1<<TXCIE);

  // Start sending a BREAK and loop (forever) in UDRE ISR
  _DMXSerialSetBaud(Calcprescale(BREAKSPEED), BREAKFORMAT);
  _DMXSerialWriteByte((uint8_t)0);
  _dmxChannel = 0;
} // init()

// Terminale operation
void DMX_Controller_term(void)
{
  // Disable all USART Features, including Interrupts
  UDR = 0;
  UCSRA, UCSRB, UCSRC = 0;
  UBRRL, UBRRH = 0;
} // term()

/** ----- internal functions and interrupt implementations ----- **/

// Initialize the Hardware serial port with the given baud rate
// using 8 data bits, no parity, 2 stop bits for data
// and 8 data bits, even parity, 1 stop bit for the break
void _DMXSerialSetBaud(uint16_t baud_setting, uint8_t format)
{
  UCSRA = 0;
  UBRRH = baud_setting >> 8;
  UBRRL = baud_setting;
  UCSRC = format;
} // _DMXSerialSetBaud

// send the next byte after current byte was sent completely.
void _DMXSerialWriteByte(uint8_t data)
{
  // putting data into buffer sends the data
  UDR = data;
} // _DMXSerialWrite


/******/

// Interrupt service routines that are called when the actual byte was sent.
// When changing speed (for sending break and sending start code) we use TX finished interrupt
// which occurs shortly after the last stop bit is sent
// When staying at the same speed (sending data bytes) we use data register empty interrupt
// which occurs shortly after the start bit of the *previous* byte
// When sending a DMX sequence it just takes the next channel byte and sends it out.
// In DMXController mode when the buffer was sent completely the DMX sequence will resent, starting with a BREAK pattern.
// In DMXReceiver mode this interrupt is disabled and will not occur.
ISR(USARTn_TX_vect)
{
  if (_dmxChannel == -1)
  {
    // this interrupt occurs after the stop bits of the last data byte
    // start sending a BREAK and loop forever in ISR
    _DMXSerialSetBaud(Calcprescale(BREAKSPEED), BREAKFORMAT);
    _DMXSerialWriteByte((uint8_t)0);
    _dmxChannel = 0;

  }
  else if (_dmxChannel == 0)
  {
    // this interrupt occurs after the stop bits of the break byte
    // now back to DMX speed: 250000baud
    _DMXSerialSetBaud(Calcprescale(DMXSPEED), DMXFORMAT);
    // take next interrupt when data register empty (early)
    UCSRB = (1<<TXEN) | (1<<UDRIE);
    // write start code
    _DMXSerialWriteByte((uint8_t)0);
    _dmxChannel = 1;

  } // if
} // ISR(USARTn_TX_vect)


// this interrupt occurs after the start bit of the previous data byte
ISR(USARTn_UDRE_vect)
{
  _DMXSerialWriteByte(dmxData[_dmxChannel++]);

  if (_dmxChannel > DMXSERIAL_MAX)
  {
    _dmxChannel   = -1; // this series is done. Next time: restart with break.
    // get interrupt after this byte is actually transmitted
    UCSRB = (1<<TXEN) | (1<<TXCIE); /// potrebne?????
  } // if
} // ISR(USARTn_UDRE_vect)


/// END OF DMX PART ///






/// BEGINNING OF SPI PART ///

volatile unsigned int SPIchannel;
volatile unsigned char receivedByte;


void initSPI(void)
{
    SPI_DDR |= (1<<DD_MISO);    // set MISO as output
    SPI_DDR &= ~((1 << DD_SCK) | (1 << DD_MOSI) | (1 << DD_SS) | (1 << DD_RSTPIN)); // set other as inputs
    SPI_PORT |= (1 << DD_SCK) | (1 << DD_MOSI) | (1 << DD_SS) | (1 << DD_RSTPIN);  // set pull-ups

    SPCR |= (1<<SPE);       // SPI enable
    SPCR |= (1<<SPIE);      // SPI interrupt enable
    SPCR &= ~(1<<MSTR);     // Set as slave
    SPCR &= ~(1<<DORD);     // Data Order MSB first

    receivedByte = 0;
    SPIchannel = 0;
}


ISR(SPI_STC_vect)
{
    if(!!((SPI_PIN & (1<<DD_RSTPIN))) && (SPDR == 0)) //dvojita negacia stavu vstup.pinu
    {
        SPIchannel = 1;
    }
    else
    {
        dmxData[SPIchannel++] = SPDR;
        SPDR = 0;
    }
    if(SPIchannel > DMXSERIAL_MAX)
        SPIchannel = 0;
}

/// END OF PERIPHERIALS ////




int main()
{
    //OSCCAL = 0xAC;

    DDRD |= (1 << TX_LED);     //set as output
    DDRD &= ~(1<<DD_LEDPIN);   //set as input

    tx_enabled = false;
    initSPI();
    //DMX_Controller_init();

    sei();

    while(1)
    {
        _delay_us(30);

        if(PIND & (1 << DD_LEDPIN)) //vysielat
        {
            if(tx_enabled == false)
            {
                tx_enabled = true;
                DMX_Controller_init();
            }
            PORTD |= (1 << TX_LED);
        }
        else                        //stop vysielania
        {
            tx_enabled = false;
            DMX_Controller_term();
            PORTD &= ~(1 << TX_LED);
        }
    }

    return 0;
}
