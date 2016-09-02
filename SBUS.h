/*
SBUS.h
Brian R Taylor
brian.taylor@bolderflight.com
2016-09-02

Copyright (c) 2016 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef SBUS_h
#define SBUS_h

#include "Arduino.h"

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
  		const double _sbusScale = 0.0012202562538133;
  		const double _sbusBias = -1.20988407565589;
  		static const uint8_t _sbusHeader = 0x0F;
  		static const uint8_t _sbusFooter = 0x00;
  		static const uint8_t _sbusLostFrame = 0x20;
  		static const uint8_t _sbusFailSafe = 0x10;
  		static const int _payloadSize = 24;
  		uint8_t _payload[_payloadSize];
  		HardwareSerial* _port;
  		bool parse();
};

#endif
