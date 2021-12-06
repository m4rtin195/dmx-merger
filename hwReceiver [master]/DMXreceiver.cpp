/***** PART of: 'DMXreceiver' library *****/

#include <limits.h>
#include "DMXreceiver.h"


/** ----- Constants ----- **/

// These definitions are used on ATmega8 boards
#define UCSRnA UCSR0A  // USART Control and Status Register A
    #define TXCn   TXC0    // Transmit buffer clear
    #define UDREn  UDRE0   // USART Data Ready
    #define FEn    FE0     // Frame Error

#define UCSRnB UCSR0B  // USART Control and Status Register B
    #define RXCIEn RXCIE0  // Enable Receive Complete Interrupt
    #define TXCIEn TXCIE0  // Enable Transmission Complete Interrupt
    #define UDRIEn UDRIE0  // Enable Data Register Empty Interrupt
    #define RXENn  RXEN0   // Enable Receiving
    #define TXENn  TXEN0   // Enable Sending

#define UCSRnC UCSR0C  // USART Control and Status Register C
    #define USBSn  USBS0   // Stop bit select 0=1bit, 1=2bits
    #define UCSZn0 UCSZ00  // Character size 00=5, 01=6, 10=7, 11=8 bits
    #define UPMn0  UPM00   // Parity setting 00=N, 10=E, 11=O

#define UBRRn  UBRR   // USART Baud Rate Register
    #define UBRRnH UBRR0H  // USART Baud Rate Register High
    #define UBRRnL UBRR0L  // USART Baud Rate Register Low

#define UDRn   UDR0    // USART Data Register


// formats for serial transmission
#define SERIAL_8N1  ((1<<URSEL) | (0<<USBSn) | (0<<UPMn0) | (3<<UCSZn0)) //unused
#define SERIAL_8N2  (/*(1<<URSEL) | */(1<<USBSn) | (0<<UPMn0) | (3<<UCSZn0))
#define SERIAL_8E1  (/*(1<<URSEL) | */(0<<USBSn) | (2<<UPMn0) | (3<<UCSZn0))
#define SERIAL_8E2  ((1<<URSEL) | (1<<USBSn) | (2<<UPMn0) | (3<<UCSZn0)) //unused


// the break timing is 10 bits (start + 8 data + parity) of this speed
// the mark-after-break is 1 bit of this speed plus approx 6 usec
// 100000 bit/sec is good: gives 100 usec break and 16 usec MAB
// 1990 spec says transmitter must send >= 92 usec break and >= 12 usec MAB
// receiver must accept 88 us break and 8 us MAB
#define BREAKSPEED     100000
#define DMXSPEED       250000
#define BREAKFORMAT    SERIAL_8E1   // 1 stop bit and 8 bit character size, even parity
#define DMXFORMAT      SERIAL_8N2   // 2 stop bits and 8 bit character size, no parity


/** ----- Enumerations ----- **/

// State of receiving DMX Bytes
typedef enum {IDLE, BREAK, DATA} DMXReceivingState;


/** ----- Macros ----- **/

// calculate prescaler from baud rate and cpu clock rate at compile time
// nb implements rounding of ((clock / 16) / baud) - 1 per atmega datasheet
#define Calcprescale(B)     ( ( (((F_CPU)/8)/(B)) - 1 ) / 2 )


/** ----- DMXSerial Private variables ----- **/

// These variables are not class members because they have to be reached by the interrupt implementations.
// don't use these variable from outside, use the appropriate methods.

volatile uint8_t _dmxRecvState; // Current State of receiving DMX Bytes
volatile uint8_t _dmxChannel;   // the next channel byte to be sent.

volatile unsigned long _gotLastPacket = 0; // the last time (using the millis function) a packet was received.

// Create a single class instance. Multiple class instances (multiple simultaneous DMX ports) are not supported.
DMXSerialClass DMX;


/** ----- forwards ----- **/

void _DMXSerialBaud(uint16_t baud_setting, uint8_t format);


/** ----- Class implementation ----- **/

// (Re)Initialize the specified mode.
// The mode parameter should be a value from enum DMXMode.
void DMXSerialClass::init(uint16_t address)
{
  // set (recognization) device address
  ADDRESS = address;
  // initialize global variables
  _dmxRecvState = IDLE; // initial state
  _dmxChannel = 0;

  // initialize the DMX buffer
  for (int n = 0; n < CHANNELS+1; n++)
    dmxData[n] = 0;

  // now start

  // Setup Hardware
  // Enable receiver and Receive interrupt
  UCSRnB = (1<<RXENn) | (1<<RXCIEn);
  _DMXSerialBaud(Calcprescale(DMXSPEED), DMXFORMAT); // Enable serial reception with a 250k rate

} // init()


// Read the current value of a channel.
// use RELATIVE method to read channel from address+channels count range
// or  ABSOLUTE method to read channel according to whole addresing range
uint8_t DMXSerialClass::read(int channel, bool method)
{
    // adjust parameter
    if (method == RELATIVE)
        {
            if (channel > CHANNELS) return 0;
            return(dmxData[channel+1]);
        }

    if (method == ABSOLUTE)
        {
            if (channel < 1 || channel > 512) return 0;
            if (channel - ADDRESS < 0 || channel - ADDRESS >= CHANNELS) {PORTD |= (1 << PD7);return 0;} //what the fucking hell??
            return(dmxData[channel-ADDRESS+1]);
            //e.g. g.ch 20 - add 18 = 2 + 1 = r.ch 3 + 1 = dmxData[4]
        }
    return 0;
} // read()


// Calculate how long no data packet was received
unsigned long DMXSerialClass::noDataSince()
{
    if(_gotLastPacket == 0) return LONG_MAX;
//    unsigned long now = millis();
    return 0;// (now - _gotLastPacket);
} // noDataSince()


// Terminale operation
void DMXSerialClass::term(void)
{
  // Disable all USART Features, including Interrupts
  UDRn = 0;
  UCSRnA, UCSRnB, UCSRnC = 0;
  UBRRnL, UBRRnH = 0;
  //isn't required URSEL to rewrite UCSRB & UBRR ?
} // term()


/** ----- internal functions and interrupt implementations ----- **/

// Initialize the Hardware serial port with the given baud rate
// using 8 data bits, no parity, 2 stop bits for data
// and 8 data bits, even parity, 1 stop bit for the break
void _DMXSerialBaud(uint16_t baud_setting, uint8_t format)
{
  // assign the baud_setting to the USART Baud Rate Register
  UCSRnA = 0;                 // 04.06.2012: use normal speed operation
  UBRRnH = baud_setting >> 8;
  UBRRnL = baud_setting;


  UCSRnC = format;
} // _DMXSerialBaud

/******/

// This Interrupt Service Routine is called when a byte or frame error was received.
// In DMXController mode this interrupt is disabled and will not occur.
// In DMXReceiver mode when a byte was received it is stored to the dmxData buffer.
ISR(USART0_RX_vect)
{
  uint8_t  USARTstate = UCSRnA;         //get state before data!
  uint8_t  DmxByte    = UDRn;	        //get data
  uint8_t  DmxState   = _dmxRecvState;	//just load once from SRAM to increase speed

  if (USARTstate & (1<<FEn))
  {  	//check for break
      _dmxRecvState = BREAK; // break condition detected.
      _dmxChannel = 0;       // The next data byte is the start byte
  }
  else if (DmxState == BREAK)
  {
      if (DmxByte == 0)
      {
          _dmxRecvState = DATA;  // normal DMX start code detected
          _dmxChannel = 1;       // start with channel # 1
//          _gotLastPacket = millis(); // remember current (relative) time in msecs.
      }
      else
      {
        _dmxRecvState = IDLE; // This might be a RDM command -> not implemented, so wait for next BREAK !
      } // if
  }
  else if (DmxState == DATA)
  {
      if(_dmxChannel >= DMX.ADDRESS && _dmxChannel < DMX.ADDRESS+CHANNELS)
        DMX.dmxData[_dmxChannel-DMX.ADDRESS+1] = DmxByte;	// store received data into dmx data buffer.

      if (_dmxChannel > DMX.ADDRESS+CHANNELS-1) // all own channels read.
      {
        _dmxRecvState = IDLE;	// wait for next break
      } // if

      _dmxChannel++;
   } // if
} // ISR(USARTn_RX_vect)

// The End
