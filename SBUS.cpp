/*
SBUS.cpp
Brian R Taylor
brian.taylor@bolderflight.com
2016-10-03

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

// Teensy 3.0 || Teensy 3.1/3.2 || Teensy 3.5 || Teensy 3.6 || Teensy LC 
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || \
	defined(__MK66FX1M0__) || defined(__MKL26Z64__)

#include "Arduino.h"
#include "SBUS.h"

#if defined(__MK20DX128__) || defined(__MK20DX256__)
	// globals needed for emulating two stop bytes on Teensy 3.0 and 3.1/3.2
	IntervalTimer serialTimer;
	HardwareSerial* SERIALPORT;
	uint8_t PACKET[25];
	volatile int SENDINDEX;
	void sendByte();
#endif

/* SBUS object, input the serial bus */
SBUS::SBUS(uint8_t bus){
	_bus = bus; // serial bus
}

/* starts the serial communication */
void SBUS::begin(){

	// initialize parsing state
	_fpos = 0;

	// select the serial port
	#if defined(__MK20DX128__) || defined(__MK20DX256__) ||  defined(__MKL26Z64__) // Teensy 3.0 || Teensy 3.1/3.2 || Teensy LC

		if(_bus == 3){
			_port = &Serial3;
		}
		else if(_bus == 2){
			_port = &Serial2;
		}
		else{
			_port = &Serial1;
		}

	#endif

	#if defined(__MK64FX512__) || defined(__MK66FX1M0__) // Teensy 3.5 || Teensy 3.6

		if(_bus == 6){
			_port = &Serial6;
		}
		else if(_bus == 5){
			_port = &Serial5;
		}
		else if(_bus == 4){
			_port = &Serial4;
		}
		else if(_bus == 3){
			_port = &Serial3;
		}
		else if(_bus == 2){
			_port = &Serial2;
		}
		else{
			_port = &Serial1;
		}

	#endif

	#if defined(__MK20DX128__) || defined(__MK20DX256__)  // Teensy 3.0 || Teensy 3.1/3.2
		// begin the serial port for SBUS
		_port->begin(100000,SERIAL_8E1_RXINV_TXINV);
		SERIALPORT = _port;
	#endif

	#if defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__MKL26Z64__)  // Teensy 3.5 || Teensy 3.6 || Teensy LC 
		// begin the serial port for SBUS
		_port->begin(100000,SERIAL_8E2_RXINV_TXINV);
	#endif
}

/* read the SBUS data and calibrate it to +/- 1 */
bool SBUS::readCal(float* calChannels, uint8_t* failsafe, uint16_t* lostFrames){
	uint16_t channels[16];

	// read the SBUS data
	if(read(&channels[0],failsafe,lostFrames)){

		// linear calibration
    	for(uint8_t i = 0; i < 16; i++){
      		calChannels[i] = channels[i] * _sbusScale + _sbusBias;
    	}

    	// return true on receiving a full packet
    	return true;
  	}
  	else{

    	// return false if a full packet is not received
    	return false;
  }
}

/* read the SBUS data */
bool SBUS::read(uint16_t* channels, uint8_t* failsafe, uint16_t* lostFrames){

  	// parse the SBUS packet
  	if(parse()){

    	// 16 channels of 11 bit data
    	channels[0]  = (int16_t) ((_payload[0]    |_payload[1]<<8)                          & 0x07FF);
    	channels[1]  = (int16_t) ((_payload[1]>>3 |_payload[2]<<5)                          & 0x07FF);
    	channels[2]  = (int16_t) ((_payload[2]>>6 |_payload[3]<<2 |_payload[4]<<10)  		& 0x07FF);
    	channels[3]  = (int16_t) ((_payload[4]>>1 |_payload[5]<<7)                          & 0x07FF);
    	channels[4]  = (int16_t) ((_payload[5]>>4 |_payload[6]<<4)                          & 0x07FF);
    	channels[5]  = (int16_t) ((_payload[6]>>7 |_payload[7]<<1 |_payload[8]<<9)   		& 0x07FF);
    	channels[6]  = (int16_t) ((_payload[8]>>2 |_payload[9]<<6)                          & 0x07FF);
    	channels[7]  = (int16_t) ((_payload[9]>>5 |_payload[10]<<3)                         & 0x07FF);
    	channels[8]  = (int16_t) ((_payload[11]   |_payload[12]<<8)                         & 0x07FF);
    	channels[9]  = (int16_t) ((_payload[12]>>3|_payload[13]<<5)                         & 0x07FF);
    	channels[10] = (int16_t) ((_payload[13]>>6|_payload[14]<<2|_payload[15]<<10) 		& 0x07FF);
    	channels[11] = (int16_t) ((_payload[15]>>1|_payload[16]<<7)                         & 0x07FF);
    	channels[12] = (int16_t) ((_payload[16]>>4|_payload[17]<<4)                         & 0x07FF);
    	channels[13] = (int16_t) ((_payload[17]>>7|_payload[18]<<1|_payload[19]<<9)  		& 0x07FF);
    	channels[14] = (int16_t) ((_payload[19]>>2|_payload[20]<<6)                         & 0x07FF);
    	channels[15] = (int16_t) ((_payload[20]>>5|_payload[21]<<3)                         & 0x07FF);

    	// count lost frames
    	if (_payload[22] == _sbusLostFrame) {
      		*lostFrames = *lostFrames + 1;
    	}

    	// failsafe state
    	if (_payload[22] == _sbusFailSafe) {
      		*failsafe = 1;
    	} 
    	else{
      		*failsafe = 0;
    	}

    	// return true on receiving a full packet
    	return true;
  	}
  	else{

    	// return false if a full packet is not received
    	return false;
  	}
}

/* parse the SBUS data */
bool SBUS::parse(){

  	// see if serial data is available
  	while(_port->available() > 0){
    	uint8_t c = _port->read();

    	// find the header
    	if(_fpos == 0){
      		if(c == _sbusHeader){
        		_fpos++;
      		}
      		else{
        		_fpos = 0;
      		}
    	}
    	else{

    		// strip off the data
    		if((_fpos-1) < _payloadSize){
      			_payload[_fpos-1] = c;
      			_fpos++;
    		}

    		// check the end byte
    		if((_fpos-1) == _payloadSize){
      			if(c == _sbusFooter){
        			_fpos = 0;
        			return true;
      			}
      			else{
        			_fpos = 0;
        			return false;
      			}
    		}
    	}
  	}
  	// return false if a partial packet
  	return false;
}

/* write SBUS packets */
void SBUS::write(uint16_t* channels){
	uint8_t packet[25];

	/* assemble the SBUS packet */

	// SBUS header
	packet[0] = _sbusHeader; 

	// 16 channels of 11 bit data
  	packet[1] = (uint8_t) ((channels[0] & 0x07FF));
  	packet[2] = (uint8_t) ((channels[0] & 0x07FF)>>8 | (channels[1] & 0x07FF)<<3);
  	packet[3] = (uint8_t) ((channels[1] & 0x07FF)>>5 | (channels[2] & 0x07FF)<<6);
  	packet[4] = (uint8_t) ((channels[2] & 0x07FF)>>2);
  	packet[5] = (uint8_t) ((channels[2] & 0x07FF)>>10 | (channels[3] & 0x07FF)<<1);
  	packet[6] = (uint8_t) ((channels[3] & 0x07FF)>>7 | (channels[4] & 0x07FF)<<4);
  	packet[7] = (uint8_t) ((channels[4] & 0x07FF)>>4 | (channels[5] & 0x07FF)<<7);
  	packet[8] = (uint8_t) ((channels[5] & 0x07FF)>>1);
  	packet[9] = (uint8_t) ((channels[5] & 0x07FF)>>9 | (channels[6] & 0x07FF)<<2);
  	packet[10] = (uint8_t) ((channels[6] & 0x07FF)>>6 | (channels[7] & 0x07FF)<<5);
  	packet[11] = (uint8_t) ((channels[7] & 0x07FF)>>3);
  	packet[12] = (uint8_t) ((channels[8] & 0x07FF));
  	packet[13] = (uint8_t) ((channels[8] & 0x07FF)>>8 | (channels[9] & 0x07FF)<<3);
  	packet[14] = (uint8_t) ((channels[9] & 0x07FF)>>5 | (channels[10] & 0x07FF)<<6);  
  	packet[15] = (uint8_t) ((channels[10] & 0x07FF)>>2);
  	packet[16] = (uint8_t) ((channels[10] & 0x07FF)>>10 | (channels[11] & 0x07FF)<<1);
  	packet[17] = (uint8_t) ((channels[11] & 0x07FF)>>7 | (channels[12] & 0x07FF)<<4);
  	packet[18] = (uint8_t) ((channels[12] & 0x07FF)>>4 | (channels[13] & 0x07FF)<<7);
  	packet[19] = (uint8_t) ((channels[13] & 0x07FF)>>1);
  	packet[20] = (uint8_t) ((channels[13] & 0x07FF)>>9 | (channels[14] & 0x07FF)<<2);
  	packet[21] = (uint8_t) ((channels[14] & 0x07FF)>>6 | (channels[15] & 0x07FF)<<5);
  	packet[22] = (uint8_t) ((channels[15] & 0x07FF)>>3);

  	// flags
	packet[23] = 0x00;

	// footer
	packet[24] = _sbusFooter;

	#if defined(__MK20DX128__) || defined(__MK20DX256__) // Teensy 3.0 || Teensy 3.1/3.2
		// use ISR to send byte at a time, 
		// 130 us between bytes to emulate 2 stop bits
		noInterrupts();
		memcpy(&PACKET,&packet,sizeof(packet));
		interrupts();
		serialTimer.priority(255);
		serialTimer.begin(sendByte,130);
	#endif

	#if defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__MKL26Z64__)  // Teensy 3.5 || Teensy 3.6 || Teensy LC
		// write packet
		_port->write(packet,25);
	#endif
}

// function to send byte at a time with
// ISR to emulate 2 stop bits on Teensy 3.0 and 3.1/3.2
#if defined(__MK20DX128__) || defined(__MK20DX256__) // Teensy 3.0 || Teensy 3.1/3.2
	void sendByte(){
		if(SENDINDEX < 25) {
			SERIALPORT->write(PACKET[SENDINDEX]);
			SENDINDEX++;
		}
		else{
			serialTimer.end();
			SENDINDEX = 0;
		}
	}
#endif

#endif
