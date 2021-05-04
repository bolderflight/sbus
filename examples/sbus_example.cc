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
bfs::SbusTx<16> sbus_tx;

/* Sbus RX data */
bfs::InceptorData data;

int main() {
  /* Serial to display data */
  Serial.begin(115200);
  while(!Serial) {}
  /* RX Config */
  bfs::InceptorConfig rx_config = {
    .hw = &Serial2,
    .throttle = {
      .ch = 0,
      .num_coef = 2,
      .poly_coef = {0.0012203, -1.2098841}
    }
  };
  if (!sbus_rx.Init(rx_config)) {
    Serial.println("Unable to establish communication with SBUS receiver");
    while (1) {}
  }
  /* TX Config */
  bfs::EffectorConfig<16> tx_config = {
    .hw = &Serial2,
    .effectors = {
      {
        .type = bfs::SERVO,
        .ch = 1,
        .min = -20,
        .max = 20,
        .failsafe = 0,
        .num_coef = 2,
        .poly_coef = {819.50, 991.50}
      }
    }
  };
  if (!sbus_tx.Init(tx_config)) {
    Serial.println("Unable to init SBUS transmitter");
    while (1) {}
  }
  sbus_tx.EnableMotors();
  sbus_tx.EnableServos();
  std::array<float, 1> cmds;
  while (1) {
    if (sbus_rx.Read(&data)) {
      Serial.println(data.throttle);
      cmds[0] = data.throttle;
      sbus_tx.Cmd(cmds);
      sbus_tx.Write();
    }
  }
}


