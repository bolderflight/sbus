//
// title:     SBUS.cpp
// author:    Taylor, Brian R.
// email:     brian.taylor@bolderflight.com
// date:      2016-04-28 
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

  // initialize parsing state
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

/* read the SBUS data and calibrate it to +/- 1 */
bool SBUS::readCal(float* calChannels, uint8_t* failsafe, int* lostFrames){
  int16_t channels[16];

  // read the SBUS data
  if(read(&channels[0],failsafe,lostFrames)){

    // linear calibration
    for(uint8_t i = 0; i < 16; i++){
      calChannels[i] = channels[i] * SBUS_SCALE + SBUS_BIAS;
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
bool SBUS::read(int16_t* channels, uint8_t* failsafe, int* lostFrames){

  // parse the SBUS packet
  if(parse()){

    // 16 channels of 11 bit data
    channels[0]  = (int16_t) ((_payload[0]    |_payload[1]<<8)                          & 0x07FF);
    channels[1]  = (int16_t) ((_payload[1]>>3 |_payload[2]<<5)                          & 0x07FF);
    channels[2]  = (int16_t) ((_payload[2]>>6 |_payload[3]<<2 |_payload[4]<<10)  & 0x07FF);
    channels[3]  = (int16_t) ((_payload[4]>>1 |_payload[5]<<7)                          & 0x07FF);
    channels[4]  = (int16_t) ((_payload[5]>>4 |_payload[6]<<4)                          & 0x07FF);
    channels[5]  = (int16_t) ((_payload[6]>>7 |_payload[7]<<1 |_payload[8]<<9)   & 0x07FF);
    channels[6]  = (int16_t) ((_payload[8]>>2 |_payload[9]<<6)                          & 0x07FF);
    channels[7]  = (int16_t) ((_payload[9]>>5 |_payload[10]<<3)                         & 0x07FF);
    channels[8]  = (int16_t) ((_payload[11]   |_payload[12]<<8)                         & 0x07FF);
    channels[9]  = (int16_t) ((_payload[12]>>3|_payload[13]<<5)                         & 0x07FF);
    channels[10] = (int16_t) ((_payload[13]>>6|_payload[14]<<2|_payload[15]<<10) & 0x07FF);
    channels[11] = (int16_t) ((_payload[15]>>1|_payload[16]<<7)                         & 0x07FF);
    channels[12] = (int16_t) ((_payload[16]>>4|_payload[17]<<4)                         & 0x07FF);
    channels[13] = (int16_t) ((_payload[17]>>7|_payload[18]<<1|_payload[19]<<9)  & 0x07FF);
    channels[14] = (int16_t) ((_payload[19]>>2|_payload[20]<<6)                         & 0x07FF);
    channels[15] = (int16_t) ((_payload[20]>>5|_payload[21]<<3)                         & 0x07FF);

    // failsafe state
    if ((_payload[22] >> 3) & 0x0001) {
      *failsafe = 1;
    } else {
      *failsafe = 0;
    }

    // count lost frames
    if ((_payload[23] >> 2) & 0x0001) {
      *lostFrames = *lostFrames + 1;
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
      _payload[_fpos-1] = c;
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
