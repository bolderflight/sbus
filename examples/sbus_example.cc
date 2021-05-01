/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2021 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#include "sbus/sbus.h"

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx;
/* SBUS object, writing SBUS */
bfs::SbusTx sbus_tx;

int main() {
  /* Serial to display data */
  Serial.begin(115200);
  while(!Serial) {}
  /* Begin communicating on SBUS serial */
  if (!sbus_rx.Begin(&Serial2)) {
    Serial.println("Unable to establish communication with SBUS receiver");
    while (1) {}
  }
  sbus_tx.Begin(&Serial2);
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


