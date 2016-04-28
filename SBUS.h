//
// title:     SBUS.h
// author:    Taylor, Brian R.
// email:     brian.taylor@bolderflight.com
// date:      2016-04-28 
// license: 
//

#ifndef SBUS_h
#define SBUS_h

#include "Arduino.h"

#define SBUS_HEADER 	0xF0
#define SBUS_FOOTER		0x00
#define PAYLOAD_SIZE	24
#define SBUS_SCALE		0.0012202562538133
#define SBUS_BIAS		-1.20988407565589				

class SBUS{
  public:
    SBUS(int bus);
    void begin();
    bool read(int16_t* channels, uint8_t* failsafe, int* lostFrames);
    bool readCal(float* calChannels, uint8_t* failsafe, int* lostFrames);
  private:
  	int _bus;
  	int _fpos;
  	uint8_t _payload[PAYLOAD_SIZE];
  	HardwareSerial* _port;
  	bool parse();
};

#endif
