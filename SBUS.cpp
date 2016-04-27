//
// title:     SBUS.cpp
// author:    Taylor, Brian R.
// email:     brian.taylor@bolderflight.com
// date:      YYYY-MM-DD 
// license: 
//

#include "Arduino.h"
#include "SBUS.h"

/* SBUS object, input the serial bus */
SBUS::SBUS(int bus){
  _bus = bus; // serial bus
}

/* starts the serial communication */
void SBUS::begin(){

  _fpos = 0;

  // select the serial port
  if(_bus == 3){
    _port = &Serial3;
  }
  else if(_bus == 2){
    _port = &Serial2;
  }
  else{
    _port = &Serial1;
  }

  // begin the serial port for SBUS
  _port->begin(100000,SERIAL_8N2_RXINV_TXINV);
}

/* read the SBUS data */
bool SBUS::read(int16_t* channels, uint8_t* failsafe, int* lostFrames){

  if(parse()){
    channels[0]  = (int16_t) ((payload[0]    |payload[1]<<8)                          & 0x07FF);
    channels[1]  = (int16_t) ((payload[1]>>3 |payload[2]<<5)                          & 0x07FF);
    channels[2]  = (int16_t) ((payload[2]>>6 |payload[3]<<2 |payload[4]<<10)  & 0x07FF);
    channels[3]  = (int16_t) ((payload[4]>>1 |payload[5]<<7)                          & 0x07FF);
    channels[4]  = (int16_t) ((payload[5]>>4 |payload[6]<<4)                          & 0x07FF);
    channels[5]  = (int16_t) ((payload[6]>>7 |payload[7]<<1 |payload[8]<<9)   & 0x07FF);
    channels[6]  = (int16_t) ((payload[8]>>2 |payload[9]<<6)                          & 0x07FF);
    channels[7]  = (int16_t) ((payload[9]>>5 |payload[10]<<3)                         & 0x07FF);
    channels[8]  = (int16_t) ((payload[11]   |payload[12]<<8)                         & 0x07FF);
    channels[9]  = (int16_t) ((payload[12]>>3|payload[13]<<5)                         & 0x07FF);
    channels[10] = (int16_t) ((payload[13]>>6|payload[14]<<2|payload[15]<<10) & 0x07FF);
    channels[11] = (int16_t) ((payload[15]>>1|payload[16]<<7)                         & 0x07FF);
    channels[12] = (int16_t) ((payload[16]>>4|payload[17]<<4)                         & 0x07FF);
    channels[13] = (int16_t) ((payload[17]>>7|payload[18]<<1|payload[19]<<9)  & 0x07FF);
    channels[14] = (int16_t) ((payload[19]>>2|payload[20]<<6)                         & 0x07FF);
    channels[15] = (int16_t) ((payload[20]>>5|payload[21]<<3)                         & 0x07FF);


    if ((payload[22] >> 3) & 0x0001) {
      *failsafe = 1;
    } else {
      *failsafe = 0;
    }

    if ((payload[23] >> 2) & 0x0001) {
      *lostFrames = *lostFrames + 1;
    }

    return true;
  }
  else{
    return false;
  }
}

/* parse the SBUS data */
bool SBUS::parse(){

  // see if serial data is available
  while(_port->available()){
    uint8_t c = _port->read();

    // find the header
    if(_fpos == 0){
      if(c == SBUS_HEADER){
        _fpos++;
      }
      else{
        _fpos = 0;
      }
    }

    // strip off the data
    if((_fpos-1) < PAYLOAD_SIZE){
      payload[_fpos-1] = c;
      _fpos++;
    }

    // check the end byte
    if((_fpos-1) == PAYLOAD_SIZE){
      if(c == SBUS_FOOTER){
        _fpos = 0;
        return true;
      }
      else{
        _fpos = 0;
        return false;
      }
    }
  }

  // return false if a partial packet
  return false;
}
