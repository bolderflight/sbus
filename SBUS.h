/*
SBUS.h
Brian R Taylor
brian.taylor@bolderflight.com
2016-11-15

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
    	SBUS(uint8_t bus);
    	void begin();
    	bool read(uint16_t* channels, uint8_t* failsafe, uint16_t* lostFrames);
    	bool readCal(float* calChannels, uint8_t* failsafe, uint16_t* lostFrames);
    	void write(uint16_t* channels);
  	private:
  		uint8_t _bus;
  		uint8_t _fpos;
  		const float _sbusScale = 0.00122025625f;
  		const float _sbusBias = -1.2098840f;
  		const uint8_t _sbusHeader = 0x0F;
  		const uint8_t _sbusFooter = 0x00;
  		const uint8_t _sbusLostFrame = 0x04;
  		const uint8_t _sbusFailSafe = 0x08;
  		static const uint8_t _payloadSize = 24;
  		uint8_t _payload[_payloadSize];
  		HardwareSerial* _port;
  		
  		bool parse();
};

#endif
