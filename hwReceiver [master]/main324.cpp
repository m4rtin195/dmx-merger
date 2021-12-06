#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <limits.h>

#include "defines324.h"


/// MILLIS TIMER ///

volatile unsigned long ms = 0;

unsigned long int millis(void)
{
    return ms;
}

void millis_init(void)
{
    //Timer0_Initialization
    GTCCR = (1 << PSRSYNC); // reset prescaler
    TCCR0B |= (1 << CS01);  // prescalar = 8
    TIMSK0 |= (1 << TOIE0); // timer 0 interrupt enable
}

volatile uint8_t times = 0;

ISR(TIMER0_OVF_vect)    // 0,5ms
{
    times++;
    if(times==10)
        {
            ms++;
            times=0;
        }
    TCNT0 = 155;  //250 tickov
}
/// TIMER END ///


/// /////////////// DMX CONTROLLER ///////////////// ///

#define Calcprescale(B)     ( ( (((F_CPU)/8)/(B)) - 1 ) / 2 )


volatile uint8_t    dmxData[DMXSERIAL_MAX+1];    // Array of DMX values (raw) // Entry 0 will not be used, only exist for logical simplicity reason...
uint16_t            BORDER_ADDR;           // hranièná adresa medzi príjmaèmi

volatile uint8_t    _dmxRecvState1; // Current State of receiving DMX Bytes
volatile uint8_t    _dmxRecvState2;
volatile int        _dmxChannel1;   // the next channel byte to be sent.
volatile int        _dmxChannel2;
volatile unsigned long _gotLastPacket1 = 0; // the last time (using the millis function) a packet was received.
volatile unsigned long _gotLastPacket2 = 0;


void DMX_Receivers_init()
{
  // initialize global variables
  _dmxRecvState1, _dmxRecvState2 = IDLE; // initial state
  _dmxChannel1, _dmxChannel2 = 0;

  // initialize the DMX buffer
  for (int n = 0; n < DMXSERIAL_MAX+1; n++)
    dmxData[n] = 0;

  // Set baud-settings and data format
  uint16_t baud_setting = Calcprescale(DMXSPEED);

  UCSR0A = 0;
  UBRR0L = baud_setting;
  UBRR0H = baud_setting >> 8;
  UCSR0C = DMXFORMAT;

  UCSR1A = 0;
  UBRR1L = baud_setting;
  UBRR1H = baud_setting >> 8;
  UCSR1C = DMXFORMAT;

  // Enable receivers and Receive interrupts
  UCSR0B = (1<<RXEN0) | (1<<RXCIE0);
  UCSR1B = (1<<RXEN1) | (1<<RXCIE1);
} // init()


// Calculate how long no data packet was received
unsigned long noDataSince1()
{
    if(_gotLastPacket1 == 0) return LONG_MAX;
    unsigned long now = millis();
    return (now - _gotLastPacket1);
} // noDataSince()

// Calculate how long no data packet was received
unsigned long noDataSince2()
{
    if(_gotLastPacket2 == 0) return LONG_MAX;
    unsigned long now = millis();
    return (now - _gotLastPacket2);
} // noDataSince()


/******/

// This Interrupt Service Routine is called when a byte or frame error was received.
ISR(USART0_RX_vect)
{
  uint8_t  USARTstate = UCSR0A;         //get state before data!
  uint8_t  DmxByte    = UDR0;	        //get data
  uint8_t  DmxState   = _dmxRecvState1;	//just load once from SRAM to increase speed

  if (USARTstate & (1<<FE0)) //check for break
  {
      _dmxRecvState1 = BREAK; // break condition detected.
      _dmxChannel1 = 0;       // The next data byte is the start byte

  }
  else if (DmxState == BREAK)
  {
      if (DmxByte == 0)
      {
          _dmxRecvState1 = DATA;  // normal DMX start code detected
          _dmxChannel1 = 1;       // start with channel # 1
          _gotLastPacket1 = millis(); // remember current (relative) time in msecs.
      }
      else
      {
        _dmxRecvState1 = IDLE; // This might be a RDM command
      } // if
  }
  else if (DmxState == DATA)
  {
      if(_dmxChannel1 >= 1 && _dmxChannel1 < (signed)BORDER_ADDR)
        dmxData[_dmxChannel1] = DmxByte;	// store received data into dmx data buffer.

      _dmxChannel1++;
      if(_dmxChannel1>DMXSERIAL_MAX) DmxState = IDLE; //error
   } // if
} // ISR(USARTn_RX_vect)

/**/

ISR(USART1_RX_vect)
{
  uint8_t  USARTstate = UCSR1A;         //get state before data!
  uint8_t  DmxByte    = UDR1;	        //get data
  uint8_t  DmxState   = _dmxRecvState2;	//just load once from SRAM to increase speed

  if (USARTstate & (1<<FE1)) //check for break
  {
      _dmxRecvState2 = BREAK; // break condition detected.
      _dmxChannel2 = 0;       // The next data byte is the start byte
  }
  else if (DmxState == BREAK)
  {
      if (DmxByte == 0)
      {
          _dmxRecvState2 = DATA;  // normal DMX start code detected
          _dmxChannel2 = 1;       // start with channel # 1
          _gotLastPacket2 = millis(); // remember current (relative) time in msecs.
      }
      else
      {
        _dmxRecvState2 = IDLE; // This might be a RDM command
      } // if
  }
  else if (DmxState == DATA)
  {
      if(!!(PINA & (1 << SHIFT_PIN)))   //shift off
      {
        if(_dmxChannel2 >= (signed)BORDER_ADDR && _dmxChannel2 <= DMXSERIAL_MAX)
            dmxData[_dmxChannel2] = DmxByte;	// store received data into dmx data buffer.
      }
      else                              //shift on
      {
        if(_dmxChannel2 >= 1 && _dmxChannel2 < (signed)BORDER_ADDR)
            dmxData[_dmxChannel2+BORDER_ADDR] = DmxByte;
      }

      _dmxChannel2++;
      if(_dmxChannel2>DMXSERIAL_MAX) DmxState = IDLE; //error
   } // if
} // ISR(USARTn_RX_vect)


/// END OF DMX PART /// /////////////////////////////////////////////////////////////





/// BEGINNING OF SPI PART ///

volatile unsigned int SPIchannel;
volatile bool inprogress;


void initSPI(void)
{
    SPI_PORT |= (1<<DD_SS);     //set SS high, inak nepovoli nastavenie MSTR
    SPI_DDR |= (1<<DD_MOSI) | (1<<DD_SCK) | (1<<DD_SS) | (1 << DD_RSTPIN) ; // SCK, MOSI and SS and RST as outputs
    SPI_DDR &= ~(1<<DD_MISO);   // MISO as input
    SPI_PORT |= (1 << DD_MISO); // enable pull-up on MISO

    //SPCR0 |= (1 << SPIE0);    // SPI interrupt enable
    SPCR0 |= (1 << SPE0);       // SPI enable
    SPCR0 |= (1 << MSTR0);      // Set as master
    SPCR0 &= ~(1<< DORD0);      // Data Order MSB first
    //SPSR0 |= (1 << SPI2X0);   // double speed enable

    SPCR0 |= (1<<SPR00);        // divide clock by 64

    inprogress = false;
}

void writeByteSPI()
{
    if(SPIchannel == 0)
    {
        SPI_PORT |= (1 << DD_RSTPIN);
        SPI_PORT &= ~(1 << DD_SS);
        SPDR0 = 0;
        while(!(SPSR0 & (1<<SPIF0)));
        SPI_PORT |=  (1 << DD_SS) | (1 << DD_RSTPIN);
        SPIchannel++;
    }
    else
    {
        SPI_PORT &= ~((1 << DD_SS) | (1 << DD_RSTPIN));
        SPDR0 = dmxData[SPIchannel++];	// Nahraj bajt do SPI dátového registra
        while(!(SPSR0 & (1<<SPIF0))); 	// Cakaj na ukoncenie prenosu
        SPI_PORT |=  (1 << DD_SS);
    }
    if(SPIchannel > DMXSERIAL_MAX) SPIchannel = 0;
}

void writeByteSPI(unsigned char byte)
{
    SPI_PORT &= ~(1 << DD_SS);
    SPDR0 = byte;					// Nahraj bajt do SPI dátového registra
    while(!(SPSR0 & (1<<SPIF0))); 	// Cakaj na ukoncenie prenosu
    SPI_PORT |=  (1 << DD_SS);
}


/// END OF PERIPHERIALS ////


uint16_t getAddr(void)
{
    uint16_t addr;
    addr = ( ((!(PINA & (1 << PA3))) << 0) | ((!(PINA & (1 << PA2))) << 1) | ((!(PINA & (1 << PA4))) << 2) | ((!(PINA & (1 << PA1))) << 3) | \
             ((!(PINA & (1 << PA5))) << 4) | ((!(PINA & (1 << PA0))) << 5) | ((!(PINA & (1 << PA6))) << 6) | ((!(PIND & (1 << PD6))) << 7) ) ;
    return addr;
}


int main(void)
{
    //OSCCAL = 0x49;  ///odmerat - OK na intRC

    DDRB |= (1 << DD_LEDPIN);   //set as output (signalizacia tx led)
    DDRD |= (1 << RXA_LED);     //set as output (leds)
    DDRD |= (1 << RXB_LED);

    //DIP input setting
    DDRA = 0x00;            //as input //dip
    PORTA = 0xFF;           //pullup //dip
    DDRD &= ~((1<<PD6) | (1<<KEEP_PIN)); //dip 8,10
    PORTD |= (1<<PD6) | (1<<KEEP_PIN);   //pullups
    //

    //inits
    millis_init();
    initSPI();
	DMX_Receivers_init();
	sei();


	while(1)
    {
        _delay_us(30);
        BORDER_ADDR = getAddr();
        writeByteSPI();

        if(!!(PIND & (1 << KEEP_PIN)))
        {
            if(noDataSince1()>1000)
            {
                for(int i=1; i<BORDER_ADDR; i++)
                    dmxData[i] = 0;
            }
            if(noDataSince2()>1000)
            {
                for(int i=BORDER_ADDR; i<DMXSERIAL_MAX; i++)
                    dmxData[i] = 0;
            }
        }

        //refresh led state
        if(noDataSince1()<1000) PORTD |= (1 << RXA_LED);
        else PORTD &= ~(1<<RXA_LED);
        if(noDataSince2()<1000) PORTD |= (1 << RXB_LED);
        else PORTD &= ~(1<<RXB_LED);

        //signal pre tx led
        if(noDataSince1()<1000 || noDataSince2()<1000)
            PORTB |= (1 << DD_LEDPIN);
        else
            PORTB &= ~(1 << DD_LEDPIN);
	}

	return 0;
}
