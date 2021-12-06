/********************************************************************************
// Title		UART Library
// Version	    3.0
// Modifed	    28/07/2014
// Other Files  'uart.cpp' 'devices.h'
//
// MCU      	testované na AT32
// used pins    RXD, TXD
// used perif   USART
// memory       i/o buffers, per 32B default
//
// CHANGE, BY   1.0 - original, Peter Fleury
//              2.0 - updated, Tim Sharpe
//              3.0 - prerobene na class, customised, Martin
//
// BASED ON:    UART Library by Peter Fleury, updated by Tim Sharpe
//              http://jump.to/fleury         http://beaststwo.org/avr-uart/
//              Based on Atmel Application Note AVR306
//
// NOTE:        spravit tick() ktora skontroluje errors a pripadne vyziada resend
//              nejaky return z putx?
//              putXn
********************************************************************************/


#ifndef UART_H
#define UART_H


#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "devices.h"


#if (__GNUC__ * 100 + __GNUC_MINOR__) < 304
#error "This library requires AVR-GCC 3.4 or later, update to newer AVR-GCC compiler !"
#endif


/** constants and macros **/

/* baudrates: 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600 */
#define UART_BAUD_SELECT(baudRate,xtalCpu)               ((xtalCpu)/((baudRate)*16l)-1)
#define UART_BAUD_SELECT_DOUBLE_SPEED(baudRate,xtalCpu) (((xtalCpu)/((baudRate)*8l)-1)|0x8000)

/* Size of the circular receive/transmit buffer, must be power of 2 */
#define UART_RX_BUFFER_SIZE 32
#define UART_TX_BUFFER_SIZE 32

/* test if the size of the circular buffers fits into SRAM */
#if ( (UART_RX_BUFFER_SIZE+UART_TX_BUFFER_SIZE) >= (RAMEND-0x60 ) )
#error "size of UART_RX_BUFFER_SIZE + UART_TX_BUFFER_SIZE larger than size of SRAM"
#endif


/* high byte error return code of uart_getc() */
#define UART_FRAME_ERROR      0x0800              /* Framing Error by UART       */
#define UART_OVERRUN_ERROR    0x0400              /* Overrun condition by UART   */
#define UART_BUFFER_OVERFLOW  0x0200              /* receive ringbuffer overflow */
#define UART_NO_DATA          0x0100              /* no receive data available   */

/* put char to flash memory macro */
#define puts_P(__s)         puts_p(PSTR(__s))
#define uart1_puts_P(__s)   uart1_puts_p(PSTR(__s))

/* options */
//#define EN_UART1


/** classe implementations **/

class UART
{
  public:
    void        init    (unsigned int baudrate);
    uint16_t    getc    (void);
    void        putc    (unsigned char data);
    void        putn    (uint16_t number);
    void        putx    (uint8_t number);
    void        puts    (const char *s );
    void        puts_p  (const char *s );
    void        flush   (void);
    int         available (void);
};

class UART1
{
  public:
    void        init    (unsigned int baudrate);
    uint16_t    getc    (void);
    void        putc    (unsigned char data);
    void        puts    (const char *s );
    void        puts_p  (const char *s );
    void        flush   (void);
    int         available (void);
};


/** create instance(s) **/

extern UART uart;

#ifdef EN_UART1
extern UART1 uart1;
#endif

#endif // UART_H
