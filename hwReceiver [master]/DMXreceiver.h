#ifndef DMXreceiver_h
#define DMXreceiver_h

#include <avr/io.h>
#include <avr/interrupt.h>
//#include "millis.h"


/** ----- Constants ----- **/

// DMX protocol configuration:
//#define ADDRESS 20         // set in init function in new versions
#define CHANNELS 5          // number of channels to read from address (incl.)

/** ----- Enumerations ----- **/

enum {RELATIVE, ABSOLUTE};  // channel counting method - used in read()


/** ----- Library Class ----- **/

class DMXSerialClass
{
  public:
    uint16_t            ADDRESS;                // Device DMX address - first channel
    volatile uint8_t    dmxData[CHANNELS+1];    // Array of DMX values (raw).
                                                // Entry 0 will not be used, only exist for logical simplicity reason...
  public:
    void            init        (uint16_t address); // Initialize
    uint8_t         read        (int channel, bool method = RELATIVE);    // Read the last known value of a channel
    unsigned long   noDataSince (void);             // Calculate how long no data backet was received
    void            term        (void);             // Terminate operation
};

// Use the DMXSerial library through the DMXSerial object
extern DMXSerialClass DMX;

#endif
