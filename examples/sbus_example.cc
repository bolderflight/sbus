/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "sbus/sbus.h"

/* SBUS object, reading SBUS */
sensors::Sbus sbus_rx(&Serial1);
/* SBUS object, writing SBUS */
actuators::Sbus sbus_tx(&Serial1);

int main() {
  /* Serial to display data */
  Serial.begin(115200);
  while(!Serial) {}
  /* Begin communicating on SBUS serial */
  sbus_rx.Begin();
  sbus_tx.Begin();
  while(1) {
    /* Check if SBUS packet received */
    if (sbus_rx.Read()) {
      /* Grab the received SBUS data */
      std::array<uint16_t, 16> sbus_data = sbus_rx.rx_channels();
      bool lost_frame = sbus_rx.lost_frame();
      bool failsafe = sbus_rx.failsafe();
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
      sbus_tx.tx_channels(sbus_data);
      /* Write SBUS data to servos */
      sbus_tx.Write();
    }
  }
}


