//#ifndef defines8_h
//#define defines8_h

/// DMX controller defines (USART) /////////////////////


#define USARTn_TX_vect   USART_TXC_vect  // Interrupt Data sent
#define USARTn_UDRE_vect USART_UDRE_vect // Interrupt Data Register empty


// formats for serial transmission
#define SERIAL_8N2  ((1<<URSEL) | (1<<USBS) | (0<<UPM0) | (3<<UCSZ0)) // 1 stop bit and 8 bit character size, even parity
#define SERIAL_8E1  ((1<<URSEL) | (0<<USBS) | (2<<UPM0) | (3<<UCSZ0)) // 2 stop bits and 8 bit character size, no parity


// the break timing is 10 bits (start + 8 data + parity) of this speed
// the mark-after-break is 1 bit of this speed plus approx 6 usec
// 100000 bit/sec is good: gives 100 usec break and 16 usec MAB
// 1990 spec says transmitter must send >= 92 usec break and >= 12 usec MAB
// receiver must accept 88 us break and 8 us MAB
#define BREAKSPEED     100000
#define DMXSPEED       250000
#define BREAKFORMAT    SERIAL_8E1
#define DMXFORMAT      SERIAL_8N2


#define DMXSERIAL_MAX 512


/// ////////////////////////////////////////////////// ///
/// SPI PART ///

//ATmega8 as slave
#define SPI_DDR    DDRB
#define SPI_PORT   PORTB
#define SPI_PIN    PINB
#define DD_MOSI    PB3
#define DD_MISO    PB4
#define DD_SCK     PB5
#define DD_SS      PB2

#define DD_RSTPIN  PB1
#define DD_LEDPIN  PD4

/// ////////////////////////////////////////////////// ///

#define TX_LED     PD0

//#endif
