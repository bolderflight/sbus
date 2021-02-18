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

#ifndef INCLUDE_SBUS_SBUS_H_
#define INCLUDE_SBUS_SBUS_H_

#include <array>
#include "Arduino.h"

class SbusRx {
 public:
  explicit SbusRx(HardwareSerial *bus);
  void Begin();
  bool Read();
  std::array<uint16_t, 16> rx_channels();
  bool failsafe();
  bool lost_frame();

 private:
  /* Communication */
  HardwareSerial *bus_;
  static constexpr uint32_t BAUD_ = 100000;
  /* Parsing */
  static constexpr uint8_t SBUS_HEADER_ = 0x0F;
  static constexpr uint8_t SBUS_FOOTER_ = 0x00;
  static constexpr uint8_t SBUS2_FOOTER_ = 0x04;
  static constexpr uint8_t SBUS_LENGTH_ = 25;
  static constexpr uint8_t SBUS_LOST_FRAME_ = 0x04;
  static constexpr uint8_t SBUS_FAILSAFE_ = 0x08;
  unsigned int parser_state_ = 0;
  uint8_t previous_byte_ = SBUS_FOOTER_;
  uint8_t rx_buffer_[SBUS_LENGTH_];
  /* Data */
  std::array<uint16_t, 16> rx_channels_;
  bool failsafe_ = false, lost_frame_ = false;
  bool Parse();
};

class SbusTx {
 public:
  explicit SbusTx(HardwareSerial *bus);
  void Begin();
  void Write();
  std::array<uint16_t, 16> tx_channels();
  void tx_channels(const std::array<uint16_t, 16> &val);

 private:
  /* Communication */
  HardwareSerial *bus_;
  static constexpr uint32_t BAUD_ = 100000;
  /* Parsing */
  static constexpr uint8_t SBUS_HEADER_ = 0x0F;
  static constexpr uint8_t SBUS_FOOTER_ = 0x00;
  static constexpr uint8_t SBUS_LENGTH_ = 25;
  uint8_t tx_buffer_[SBUS_LENGTH_];
  /* Data */
  std::array<uint16_t, 16> tx_channels_;
};

#endif  // INCLUDE_SBUS_SBUS_H_
