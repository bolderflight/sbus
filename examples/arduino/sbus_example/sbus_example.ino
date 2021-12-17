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

/*
* This example reads an SBUS packet from an SBUS receiver and writes it to an
* SBUS compatible servo. The SBUS out capability (i.e. writing a command to
* the servo) could be generated independently; however, the packet timing
* would need to be controlled by the programmer, the write function simply
* generates an SBUS packet and writes it to the servos. In this case the
* packet timing is handled by the SBUS receiver and waiting for a good packet
* read.
*/

#include "sbus.h"

/* SbusRx object on Serial1 */
bfs::SbusRx sbus_rx(&Serial1);
/* SbusTx object on Serial1 */
bfs::SbusTx sbus_tx(&Serial1);
/* Array for storing SBUS data */
std::array<int16_t, bfs::SbusRx::NUM_CH()> sbus_data;

void setup() {
  /* Serial to display the received data */
  Serial.begin(115200);
  while (!Serial) {}
  /* Begin the SBUS communication */
  sbus_rx.Begin();
  sbus_tx.Begin();
}

void loop() {
  if (sbus_rx.Read()) {
    /* Grab the received data */
    sbus_data = sbus_rx.ch();
    /* Display the received data */
    for (int8_t i = 0; i < bfs::SbusRx::NUM_CH(); i++) {
      Serial.print(sbus_data[i]);
      Serial.print("\t");
    }
    /* Display lost frames and failsafe data */
    Serial.print(sbus_rx.lost_frame());
    Serial.print("\t");
    Serial.println(sbus_rx.failsafe());
    /* Set the SBUS TX data to the received data */
    sbus_tx.ch(sbus_data);
    /* Write the data to the servos */
    sbus_tx.Write();
  }
}

