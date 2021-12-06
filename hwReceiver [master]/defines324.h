//#ifndef defines32_h
//#define defines32_h

/// DMX controller defines (USART)
//#define UCSRnA UCSR0A  // USART Control and Status Register A
//    #define TXCn   TXC0    // Transmit buffer clear
//    #define UDREn  UDRE0   // USART Data Ready
//    #define FEn    FE0     // Frame Error
//
//#define UCSRnB UCSR0B  // USART Control and Status Register B
//    #define RXCIEn RXCIE0  // Enable Receive Complete Interrupt
//    #define TXCIEn TXCIE0  // Enable Transmission Complete Interrupt
//    #define UDRIEn UDRIE0  // Enable Data Register Empty Interrupt
//    #define RXENn  RXEN0   // Enable Receiving
//    #define TXENn  TXEN0   // Enable Sending
//
//#define UCSRnC UCSR0C  // USART Control and Status Register C
//    #define USBSn  USBS0   // Stop bit select 0=1bit, 1=2bits
//    #define UCSZn0 UCSZ00  // Character size 00=5, 01=6, 10=7, 11=8 bits
//    #define UPMn0  UPM00   // Parity setting 00=N, 10=E, 11=O
//
//#define UBRRn  UBRR   // USART Baud Rate Register
//    #define UBRRnH UBRR0H  // USART Baud Rate Register High
//    #define UBRRnL UBRR0L  // USART Baud Rate Register Low
//
//#define UDRn   UDR0    // USART Data Register

#define USARTn_TX_vect   USART_TXC_vect  // Interrupt Data sent

// formats for serial transmission
//#define SERIAL_8N1  ((1<<URSEL) | (0<<USBSn) | (0<<UPMn0) | (3<<UCSZn0)) //unused
#define SERIAL_8N2  ((1<<USBS0) | (0<<UPM00) | (3<<UCSZ00))
#define SERIAL_8E1  ((0<<USBS0) | (2<<UPM00) | (3<<UCSZ00))
//#define SERIAL_8E2  ((1<<URSEL) | (1<<USBSn) | (2<<UPMn0) | (3<<UCSZn0)) //unused


// the break timing is 10 bits (start + 8 data + parity) of this speed
// the mark-after-break is 1 bit of this speed plus approx 6 usec
// 100000 bit/sec is good: gives 100 usec break and 16 usec MAB
// 1990 spec says transmitter must send >= 92 usec break and >= 12 usec MAB
// receiver must accept 88 us break and 8 us MAB
#define BREAKSPEED     100000
#define DMXSPEED       250000
#define BREAKFORMAT    SERIAL_8E1   // 1 stop bit and 8 bit character size, even parity
#define DMXFORMAT      SERIAL_8N2   // 2 stop bits and 8 bit character size, no parity


#define DMXSERIAL_MAX 512

typedef enum {IDLE, BREAK, DATA} DMXReceivingState;


/// ////////////////////////////////////////////////// ///
/// SPI PART ///

//ATmega324 as master
#define SPI_DDR     DDRB
#define SPI_PORT    PORTB
#define DD_MOSI     PB5
#define DD_MISO     PB6
#define DD_SCK      PB7
#define DD_SS       PB4
//aux_delay_us(40);
#define DD_LEDPIN      PB0
#define DD_RSTPIN      PB3

/// ////////////////////////////////////////////////// ///

#define SHIFT_PIN   PA7     // DIP9
#define KEEP_PIN    PD7     // DIP10

#define RXA_LED     PD1
#define RXB_LED     PD3
//#endif
