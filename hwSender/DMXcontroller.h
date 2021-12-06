#ifndef DMXcontroller_h
#define DMXcontroller_h

#include <avr/io.h>
#include <avr/interrupt.h>


/** ----- Constants ----- **/

#define DMXSERIAL_MAX 512 // max. number of supported DMX data channels


/** ----- Library Class ----- **/

class DMXSerialClass
{
  public:
    // Array of DMX values (raw)
    // Entry 0 will never be used for DMX data but will store the startbyte
    uint8_t  dmxData[DMXSERIAL_MAX+1];

    // Initialize
    void    init       (void);

    // Set the maximum used channel for DMXController mode
    void    maxChannel (int channel);

    // Write a new value of a channel
    void    write      (int channel, uint8_t value);

    // Terminate operation
    void    term       (void);
};

// Use the DMXSerial library through the DMXSerial object
extern DMXSerialClass DMX;

#endif
