//
// title:     SBUS_example.ino
// author:    Taylor, Brian R.
// email:     brian.taylor@bolderflight.com
// date:      2016-07-12
// license: 
//

// This example reads an SBUS packet from an
// SBUS receiver (FrSky X8R) and then takes that
// packet and writes it back to an SBUS
// compatible servo. The SBUS out capability (i.e.
// writing a command to the servo) could be generated
// independently; however, the packet timing would need
// to be controlled by the programmer, the write function
// simply generates an SBUS packet and writes it to the
// servos. In this case the packet timing is handled by the
// SBUS receiver and waiting for a good packet read.

#include "SBUS.h"

// a SBUS object, which is on Teensy hardware
// serial port 1
SBUS x8r(1);

// channel, fail safe, and lost frames data
int16_t channels[16];
uint8_t failSafe;
int lostFrames = 0;

void setup() {
  // begin the SBUS communication
  x8r.begin();
}

void loop() {

  // look for a good SBUS packet from the receiver
  if(x8r.read(&channels[0], &failSafe, &lostFrames)){

    // write the SBUS packet to an SBUS compatible servo
    x8r.write(&channels[0]);
  }
}

