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
  if(_bus == 2){
    Serial2.begin(100000,SERIAL_8N2_RXINV_TXINV);
  }
  else if(_bus == 3){
    Serial3.begin(100000,SERIAL_8N2_RXINV_TXINV);
  }
  else{
    Serial1.begin(100000,SERIAL_8N2_RXINV_TXINV);
  }
}

