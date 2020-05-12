/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "sbus/sbus.h"

/* SBUS object */
Sbus sbus(&Serial1);

int main() {
  /* Serial to display data */
  Serial.begin(115200);
  while(!Serial) {}
  /* Begin communicating on SBUS serial */
  sbus.Begin();
  while(1) {
    /* Check if SBUS packet received */
    if (sbus.Read()) {
      /* Grab the received SBUS data */
      std::array<uint16_t, 16> sbus_data = sbus.rx_channels();
      bool lost_frame = sbus.lost_frame();
      bool failsafe = sbus.failsafe();
      /* Print channel data */
      for (unsigned int i = 0; i < 16; i++) {
        Serial.print(sbus_data[i]);
        Serial.print("\t");
      }
      /* Print lost frames and failsafe */
      Serial.print(lost_frame);
      Serial.print("\t");
      Serial.println(failsafe);
      /* Copy read SBUS data to transmit buffer */
      sbus.tx_channels(sbus_data);
      /* Write SBUS data to servos */
      sbus.Write();
    }
  }
}


