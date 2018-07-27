/*
SBUS.h
Brian R Taylor
brian.taylor@bolderflight.com

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
#include "elapsedMillis.h"

/* 
* Hardware Serial Supported:
* Teensy 3.0 || Teensy 3.1/3.2 || Teensy 3.5 || Teensy 3.6 || Teensy LC  || STM32L4 || Maple Mini
*/
#if defined(__MK20DX128__) 	|| defined(__MK20DX256__) || defined(__MK64FX512__)	\
	|| defined(__MK66FX1M0__) || defined(__MKL26Z64__) 	|| defined(STM32L496xx)		\
	|| defined(STM32L476xx) 	|| defined(STM32L433xx) 	|| defined(STM32L432xx)		\
	|| defined(_BOARD_MAPLE_MINI_H_)
#endif

class SBUS{
	public:
		SBUS(HardwareSerial& bus);
		void begin();
		bool read(uint16_t* channels, bool* failsafe, bool* lostFrame);
		bool readCal(float* calChannels, bool* failsafe, bool* lostFrame);
		void write(uint16_t* channels);
		void writeCal(float *channels);
		void setMin(uint16_t min);
		void setMax(uint16_t max);
		uint16_t getMin();
		uint16_t getMax();
  private:
		const uint8_t _sbusHeader = 0x0F;
		const uint8_t _sbusFooter = 0x00;
		const uint8_t _sbus2Footer = 0x04;
		const uint8_t _sbus2Mask = 0x0F;
		const uint32_t SBUS_TIMEOUT_US = 7000;
		uint8_t _parserState, _prevByte = _sbusFooter, _curByte;
		static const uint8_t _payloadSize = 24;
		uint8_t _payload[_payloadSize];		
		float _sbusScale = 0.00122025625f;
		float _sbusBias = -1.2098840f;
		const uint8_t _sbusLostFrame = 0x04;
		const uint8_t _sbusFailSafe = 0x08;
		const uint16_t _sbusMin = 172;
		const uint16_t _sbusMax = 1811;
		HardwareSerial* _bus;
		bool parse();
		void scaleBias();
};

#endif
