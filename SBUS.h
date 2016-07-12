//
// title:     SBUS.h
// author:    Taylor, Brian R.
// email:     brian.taylor@bolderflight.com
// date:      2016-07-12 
// license: 
//

#ifndef SBUS_h
#define SBUS_h

#include "Arduino.h"

const uint8_t SBUS_HEADER 		= 0x0F;
const uint8_t SBUS_FOOTER 		= 0x00;
const uint8_t SBUS_LOST_FRAME 	= 0x20;
const uint8_t SBUS_FAILSAFE 	= 0x10;
const int PAYLOAD_SIZE 			= 24;
const double SBUS_SCALE 		= 0.0012202562538133;
const double SBUS_BIAS			= -1.20988407565589;				

class SBUS{
	public:
    	SBUS(int bus);
    	void begin();
    	bool read(int16_t* channels, uint8_t* failsafe, int* lostFrames);
    	bool readCal(float* calChannels, uint8_t* failsafe, int* lostFrames);
    	void write(int16_t* channels);
  	private:
  		int _bus;
  		int _fpos;
  		uint8_t _payload[PAYLOAD_SIZE];
  		HardwareSerial* _port;
  		bool parse();
};

#endif
