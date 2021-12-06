/***** PART of: 'UART' library *****/

#include "uart.h"


UART uart;

#ifdef EN_UART1
UART1 uart1;
#endif


/** constants and macros **/

// size of RX/TX buffers
#define UART_RX_BUFFER_MASK ( UART_RX_BUFFER_SIZE - 1)
#define UART_TX_BUFFER_MASK ( UART_TX_BUFFER_SIZE - 1)

#if ( UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK )
#error RX buffer size is not a power of 2
#endif
#if ( UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK )
#error TX buffer size is not a power of 2
#endif



/**  module global variables **/

static volatile unsigned char UART_TxBuf[UART_TX_BUFFER_SIZE];
static volatile unsigned char UART_RxBuf[UART_RX_BUFFER_SIZE];
static volatile unsigned char UART_TxHead;
static volatile unsigned char UART_TxTail;
static volatile unsigned char UART_RxHead;
static volatile unsigned char UART_RxTail;
static volatile unsigned char UART_LastRxError;

#if defined( ATMEGA_USART1 )
static volatile unsigned char UART1_TxBuf[UART_TX_BUFFER_SIZE];
static volatile unsigned char UART1_RxBuf[UART_RX_BUFFER_SIZE];
static volatile unsigned char UART1_TxHead;
static volatile unsigned char UART1_TxTail;
static volatile unsigned char UART1_RxHead;
static volatile unsigned char UART1_RxTail;
static volatile unsigned char UART1_LastRxError;
#endif




/*************************************************************************
Function: UART Data Register Empty interrupt
Purpose:  called when the UART is ready to transmit the next byte
**************************************************************************/
ISR(UART0_TRANSMIT_INTERRUPT)
{
    unsigned char tmptail;


    if ( UART_TxHead != UART_TxTail)
    {
        /* calculate and store new buffer index */
        tmptail = (UART_TxTail + 1) & UART_TX_BUFFER_MASK;
        UART_TxTail = tmptail;
        /* get one byte from buffer and write it to UART */
        UART0_DATA = UART_TxBuf[tmptail];  /* start transmission */


    }
    else
    {
        /* tx buffer empty, disable UDRE interrupt */
        UART0_CONTROL &= ~_BV(UART0_UDRIE);
    }
}


/*************************************************************************
Function: uart_init()
Purpose:  initialize UART and set baudrate
Input:    baudrate
Returns:  none
**************************************************************************/
void UART::init(unsigned int baudrate)
{
    UART_TxHead = 0;
    UART_TxTail = 0;
    UART_RxHead = 0;
    UART_RxTail = 0;

#if defined AT90_UART
    /* set baud rate */
    UBRR = (unsigned char)baudrate;

    /* enable UART receiver and transmmitter and receive complete interrupt */
    UART0_CONTROL = _BV(RXCIE)|_BV(RXEN)|_BV(TXEN);

#elif defined ATMEGA_USART
    /* Set baud rate */
    //baudrate = UART_BAUD_SELECT(baudrate, F_CPU);
    baudrate = UART_BAUD_SELECT_DOUBLE_SPEED(baudrate, F_CPU);
    if ( baudrate & 0x8000 )
    {
        UART0_STATUS = (1<<U2X);  //Enable 2x speed
        baudrate &= ~0x8000;
    }

    UBRRH = (unsigned char)(baudrate>>8);
    UBRRL = (unsigned char) baudrate;

    /* Enable USART receiver and transmitter and receive complete interrupt */
    UART0_CONTROL = _BV(RXCIE)|(1<<RXEN)|(1<<TXEN);

    /* Set frame format: asynchronous, 8data, no parity, 1stop bit */
#ifdef URSEL
    UCSRC = (1<<URSEL)|(3<<UCSZ0);
#else
    UCSRC = (3<<UCSZ0);
#endif

#elif defined ATMEGA_USART0
    /* Set baud rate */
    if ( baudrate & 0x8000 )
    {
        UART0_STATUS = (1<<U2X0);  //Enable 2x speed
        baudrate &= ~0x8000;
    }
    UBRR0H = (unsigned char)(baudrate>>8);
    UBRR0L = (unsigned char) baudrate;
if(baudrate==9601)PORTB |= (1 << PB0);
    /* Enable USART receiver and transmitter and receive complete interrupt */
    UART0_CONTROL = _BV(RXCIE0)|(1<<RXEN0)|(1<<TXEN0);

    /* Set frame format: asynchronous, 8data, no parity, 1stop bit */
#ifdef URSEL0
    UCSR0C = (1<<URSEL0)|(3<<UCSZ00);
#else
    UCSR0C = (3<<UCSZ00);
#endif

#elif defined ATMEGA_UART
    /* set baud rate */
    if ( baudrate & 0x8000 )
    {
        UART0_STATUS = (1<<U2X);  //Enable 2x speed
        baudrate &= ~0x8000;
    }
    UBRRHI = (unsigned char)(baudrate>>8);
    UBRR   = (unsigned char) baudrate;

    /* Enable UART receiver and transmitter and receive complete interrupt */
    UART0_CONTROL = _BV(RXCIE)|(1<<RXEN)|(1<<TXEN);

#endif

}/* uart_init */


/*************************************************************************
Function: uart_getc()
Purpose:  return byte from ringbuffer
Returns:  lower byte:  received byte from ringbuffer
          higher byte: last receive error
**************************************************************************/
uint16_t UART::getc(void)
{
    unsigned char tmptail;
    unsigned char data;


    if ( UART_RxHead == UART_RxTail )
    {
        return UART_NO_DATA;   /* no data available */
    }

    /* calculate /store buffer index */
    tmptail = (UART_RxTail + 1) & UART_RX_BUFFER_MASK;
    UART_RxTail = tmptail;

    /* get data from receive buffer */
    data = UART_RxBuf[tmptail];

    return (UART_LastRxError << 8) + data;

}/* uart_getc */


/*************************************************************************
Function: uart_putc()
Purpose:  write byte to ringbuffer for transmitting via UART
Input:    byte to be transmitted
Returns:  none
**************************************************************************/
void UART::putc(unsigned char data)
{
    unsigned char tmphead;


    tmphead  = (UART_TxHead + 1) & UART_TX_BUFFER_MASK;

    while ( tmphead == UART_TxTail );   /* wait for free space in buffer */

    UART_TxBuf[tmphead] = data;
    UART_TxHead = tmphead;

    /* enable UDRE interrupt */
    UART0_CONTROL    |= _BV(UART0_UDRIE);

}/* uart_putc */


/*************************************************************************
Function: uart_putn() (my)
Purpose:  transmit number as string via UART
Input:    8-bit number to be transmitted
Returns:  none
**************************************************************************/
void UART::putn(uint16_t number)
{
    char temp[5];
    itoa(number,temp,10);
    puts(temp);
}/* uart_putn */


/*************************************************************************
Function: uart_putx() (my)
Purpose:  transmit hex-number as string via UART
Input:    8-bit number to be transmitted
Returns:  none
**************************************************************************/
void UART::putx(uint8_t number)
{
    char temp[5] = {"0x"};
    itoa(number,temp+2,16);
    puts(temp);
}/* uart_putn */


/*************************************************************************
Function: uart_puts()
Purpose:  transmit string via UART
Input:    string to be transmitted
Returns:  none
**************************************************************************/
void UART::puts(const char *s )
{
    while (*s)
        putc(*s++);

}/* uart_puts */


/*************************************************************************
Function: uart_puts_p()
Purpose:  transmit string from program memory via UART
Input:    program memory string to be transmitted
Returns:  none
**************************************************************************/
void UART::puts_p(const char *progmem_s )
{
    register char c;

    while ( (c = pgm_read_byte(progmem_s++)) )
        putc(c);

}/* uart_puts_p */


/*************************************************************************
Function: uart_available()
Purpose:  Determine the number of bytes waiting in the receive buffer
Input:    None
Returns:  Integer number of bytes in the receive buffer
**************************************************************************/
int UART::available(void)
{
    return (UART_RX_BUFFER_MASK + UART_RxHead - UART_RxTail) % UART_RX_BUFFER_MASK;
}/* uart_available */


/*************************************************************************
Function: uart_flush()
Purpose:  Flush bytes waiting the receive buffer. Actually ignores them.
Input:    None
Returns:  None
**************************************************************************/
void UART::flush(void)
{
    UART_RxHead = UART_RxTail;
}/* uart_flush */





//----------------------------------------------------------------------------------------------//



/*
 * these functions are only for ATmegas with two USART
 */

#if defined( ATMEGA_USART1 )

ISR(UART1_RECEIVE_INTERRUPT)
/*************************************************************************
Function: UART1 Receive Complete interrupt
Purpose:  called when the UART1 has received a character
**************************************************************************/
{
    unsigned char tmphead;
    unsigned char data;
    unsigned char usr;
    unsigned char lastRxError;


    /* read UART status register and UART data register */
    usr  = UART1_STATUS;
    data = UART1_DATA;

    /* */
    lastRxError = (usr & (_BV(FE1)|_BV(DOR1)) );

    /* calculate buffer index */
    tmphead = ( UART1_RxHead + 1) & UART_RX_BUFFER_MASK;

    if ( tmphead == UART1_RxTail )
    {
        /* error: receive buffer overflow */
        lastRxError = UART_BUFFER_OVERFLOW >> 8;
    }
    else
    {
        /* store new index */
        UART1_RxHead = tmphead;
        /* store received data in buffer */
        UART1_RxBuf[tmphead] = data;
    }
    UART1_LastRxError = lastRxError;
}


ISR(UART1_TRANSMIT_INTERRUPT)
/*************************************************************************
Function: UART1 Data Register Empty interrupt
Purpose:  called when the UART1 is ready to transmit the next byte
**************************************************************************/
{
    unsigned char tmptail;


    if ( UART1_TxHead != UART1_TxTail)
    {
        /* calculate and store new buffer index */
        tmptail = (UART1_TxTail + 1) & UART_TX_BUFFER_MASK;
        UART1_TxTail = tmptail;
        /* get one byte from buffer and write it to UART */
        UART1_DATA = UART1_TxBuf[tmptail];  /* start transmission */
    }
    else
    {
        /* tx buffer empty, disable UDRE interrupt */
        UART1_CONTROL &= ~_BV(UART1_UDRIE);
    }
}


/*************************************************************************
Function: uart1_init()
Purpose:  initialize UART1 and set baudrate
Input:    baudrate
Returns:  none
**************************************************************************/
void uart1_init(unsigned int baudrate)
{
    UART1_TxHead = 0;
    UART1_TxTail = 0;
    UART1_RxHead = 0;
    UART1_RxTail = 0;


    /* Set baud rate */
    baudrate = UART_BAUD_SELECT(baudrate, F_CPU);
    //baudrate = UART_BAUD_SELECT_DOUBLE_SPEED(baudrate, F_CPU);
    if ( baudrate & 0x8000 )
    {
        UART1_STATUS = (1<<U2X1);  //Enable 2x speed
        baudrate &= ~0x8000;
    }

    UBRR1H = (unsigned char)(baudrate>>8);
    UBRR1L = (unsigned char) baudrate;

    /* Enable USART receiver and transmitter and receive complete interrupt */
    UART1_CONTROL = _BV(RXCIE1)|(1<<RXEN1)|(1<<TXEN1);

    /* Set frame format: asynchronous, 8data, no parity, 1stop bit */
#ifdef URSEL1
    UCSR1C = (1<<URSEL1)|(3<<UCSZ10);
#else
    UCSR1C = (3<<UCSZ10);
#endif
}/* uart_init */


/*************************************************************************
Function: uart1_getc()
Purpose:  return byte from ringbuffer
Returns:  lower byte:  received byte from ringbuffer
          higher byte: last receive error
**************************************************************************/
uint16_t uart1_getc(void)
{
    unsigned char tmptail;
    unsigned char data;


    if ( UART1_RxHead == UART1_RxTail )
    {
        return UART_NO_DATA;   /* no data available */
    }

    /* calculate /store buffer index */
    tmptail = (UART1_RxTail + 1) & UART_RX_BUFFER_MASK;
    UART1_RxTail = tmptail;

    /* get data from receive buffer */
    data = UART1_RxBuf[tmptail];

    return (UART1_LastRxError << 8) + data;

}/* uart1_getc */


/*************************************************************************
Function: uart1_putc()
Purpose:  write byte to ringbuffer for transmitting via UART
Input:    byte to be transmitted
Returns:  none
**************************************************************************/
void uart1_putc(unsigned char data)
{
    unsigned char tmphead;


    tmphead  = (UART1_TxHead + 1) & UART_TX_BUFFER_MASK;

    while ( tmphead == UART1_TxTail );  /* wait for free space in buffer */

    UART1_TxBuf[tmphead] = data;
    UART1_TxHead = tmphead;

    /* enable UDRE interrupt */
    UART1_CONTROL    |= _BV(UART1_UDRIE);

}/* uart1_putc */


/*************************************************************************
Function: uart1_puts()
Purpose:  transmit string to UART1
Input:    string to be transmitted
Returns:  none
**************************************************************************/
void uart1_puts(const char *s )
{
    while (*s)
        uart1_putc(*s++);

}/* uart1_puts */


/*************************************************************************
Function: uart1_puts_p()
Purpose:  transmit string from program memory to UART1
Input:    program memory string to be transmitted
Returns:  none
**************************************************************************/
void uart1_puts_p(const char *progmem_s )
{
    register char c;

    while ( (c = pgm_read_byte(progmem_s++)) )
        uart1_putc(c);

}/* uart1_puts_p */



/*************************************************************************
Function: uart1_available()
Purpose:  Determine the number of bytes waiting in the receive buffer
Input:    None
Returns:  Integer number of bytes in the receive buffer
**************************************************************************/
int uart1_available(void)
{
    return (UART_RX_BUFFER_MASK + UART1_RxHead - UART1_RxTail) % UART_RX_BUFFER_MASK;
}/* uart1_available */



/*************************************************************************
Function: uart1_flush()
Purpose:  Flush bytes waiting the receive buffer.  Acutally ignores them.
Input:    None
Returns:  None
**************************************************************************/
void uart1_flush(void)
{
    UART1_RxHead = UART1_RxTail;
}/* uart1_flush */



#endif
