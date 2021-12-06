/***** PART of: 'DMXcontroller' library *****/

#include "DMXcontroller.h"
#include "defines.h"


/** ----- Macros ----- **/

// calculate prescaler from baud rate and cpu clock rate at compile time
// nb implements rounding of ((clock / 16) / baud) - 1 per atmega datasheet
#define Calcprescale(B)     ( ( (((F_CPU)/16)/(B)) - 1 )  )


/** ----- DMXSerial Private variables ----- **/

// These variables are not class members because they have to be reached by the interrupt implementations.
// don't use these variable from outside, use the appropriate methods.

int     _dmxChannel;  // the next channel byte to be sent.
volatile int     _dmxMaxChannel = 32; // the last channel used for sending. //default - broadcast 32 channels

// Create a single class instance. Multiple class instances (multiple simultaneous DMX ports) are not supported.
DMXSerialClass DMX;


/** ----- forwards ----- **/

void _DMXSerialSetBaud(uint16_t baud_setting, uint8_t format);
void _DMXSerialWriteByte(uint8_t data);


/** ----- Class implementation ----- **/

void DMXSerialClass::init()
{

  // initialize global variables
  _dmxChannel = 0;

  // initialize the DMX buffer
  for (int n = 0; n < DMXSERIAL_MAX+1; n++)
    dmxData[n] = 0;

  // Enable transmitter and interrupt
  UCSRnB = (1<<TXENn) | (1<<TXCIEn);

  // Start sending a BREAK and loop (forever) in UDRE ISR
  _DMXSerialSetBaud(Calcprescale(BREAKSPEED), BREAKFORMAT);
  _DMXSerialWriteByte((uint8_t)0);
  _dmxChannel = 0;

} // init()


void DMXSerialClass::write(int channel, uint8_t value)
{
  // adjust parameters
  if (channel < 1) channel = 1;
  if (channel > DMXSERIAL_MAX) channel = DMXSERIAL_MAX;
  if (value < 0)   value = 0;
  if (value > 255) value = 255;

  // store value for later sending
  dmxData[channel] = value;

  // Make sure we transmit enough channels for the ones used
  // original: if (channel > _dmxMaxChannel)
  if (channel > _dmxMaxChannel && channel < DMXSERIAL_MAX)
    _dmxMaxChannel = channel;
} // write()


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
void _DMXSerialSetBaud(uint16_t baud_setting, uint8_t format)
{
  // assign the baud_setting to the USART Baud Rate Register
  UCSRnA = 0;                 // 04.06.2012: use normal speed operation
  UBRRnH = baud_setting >> 8;
  UBRRnL = baud_setting;

  // 2 stop bits and 8 bit character size, no parity
  UCSRnC = format;
} // _DMXSerialSetBaud

// send the next byte after current byte was sent completely.
void _DMXSerialWriteByte(uint8_t data)
{
  // putting data into buffer sends the data
  UDRn = data;
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
    UCSRnB = (1<<TXENn) | (1<<UDRIEn);
    // write start code
    _DMXSerialWriteByte((uint8_t)0);
    _dmxChannel = 1;

  } // if
} // ISR(USARTn_TX_vect)


// this interrupt occurs after the start bit of the previous data byte
ISR(USARTn_UDRE_vect)
{
  _DMXSerialWriteByte(DMX.dmxData[_dmxChannel++]);

  if (_dmxChannel > _dmxMaxChannel)
  {
    _dmxChannel   = -1; // this series is done. Next time: restart with break.
    // get interrupt after this byte is actually transmitted
    UCSRnB = (1<<TXENn) | (1<<TXCIEn);
  } // if
} // ISR(USARTn_UDRE_vect)

// The End
