/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
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

#include "sbus.h"
#include "ppm.h"

#define PIN_INPUT  33
#define PIN_OUTPUT 32

/* SBUS object, reading SBUS */
bfs::SbusRx rx(&Serial, PIN_INPUT, PIN_OUTPUT, false);
/* PPMEncoder object, writing PPM */
PPMEncoder tx;
/* SBUS data */
bfs::SbusData data;

void setup() {
  /* Serial to display data */
  Serial.begin(115200);
  while (!Serial) {}
  /* Begin the SBUS communication */
  rx.Begin();
  /* and the PPM as well */
  tx.begin(PIN_OUTPUT, 16, false);
}

void loop () {
  if (rx.Read()) {
    /* Grab the received data */
    data = rx.data();
    /* Display the received data */
    for (int8_t i = 0; i < data.NUM_CH; i++) {
      Serial.print(data.ch[i]);
      Serial.print("\t");

      tx.setChannel(i, data.ch[i]);
    }
    
    /* Display lost frames and failsafe data */
    //Serial.print(data.lost_frame);
    //Serial.print("\t");
    //Serial.println(data.failsafe);
  }
}
