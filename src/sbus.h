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


#if defined(__AVR__)
#define ETL_NO_STL
#include "Arduino.h"
#include <Embedded_Template_Library.h>
#include <etl/array.h>
namespace sbus = etl;
#else
#include "Arduino.h"
#include <array>
namespace sbus = std;
#endif


class SbusRx {
 public:

  explicit SbusRx(HardwareSerial *bus);

#ifdef ESP32
  void Begin(uint8_t rxpin, uint8_t txpin);
#else
  void Begin();
#endif

  bool Read();
  sbus::array<uint16_t, 16> rx_channels();
  inline bool failsafe() const {return failsafe_;}
  inline bool lost_frame() const {return lost_frame_;}
  inline bool ch17() const {return ch17_;}
  inline bool ch18() const {return ch18_;}

 private:
  /* Communication */
  HardwareSerial *bus_;

  static constexpr uint32_t BAUD_ = 100000;
  /* Parsing */
  static constexpr uint8_t SBUS_HEADER_ = 0x0F;
  static constexpr uint8_t SBUS_FOOTER_ = 0x00;
  static constexpr uint8_t SBUS2_FOOTER_ = 0x04;
  static constexpr uint8_t SBUS_LENGTH_ = 25;
  static constexpr uint8_t SBUS_CH17_ = 0x01;
  static constexpr uint8_t SBUS_CH18_ = 0x02;
  static constexpr uint8_t SBUS_LOST_FRAME_ = 0x04;
  static constexpr uint8_t SBUS_FAILSAFE_ = 0x08;
  unsigned int parser_state_ = 0;
  uint8_t previous_byte_ = SBUS_FOOTER_;
  uint8_t rx_buffer_[SBUS_LENGTH_];
  /* Data */
  sbus::array<uint16_t, 16> rx_channels_;
  bool failsafe_ = false, lost_frame_ = false, ch17_ = false, ch18_ = false;
  bool Parse();
};

class SbusTx {
 public:

  explicit SbusTx(HardwareSerial *bus);

#ifdef ESP32
  void Begin(uint8_t rxpin, uint8_t txpin);
#else
  void Begin();
#endif

  void Write();
  void failsafe(bool val) {failsafe_ = val;}
  void lost_frame(bool val) {lost_frame_ = val;}
  void ch17(bool val) {ch17_ = val;}
  void ch18(bool val) {ch18_ = val;}
  void tx_channels(const sbus::array<uint16_t, 16> &val);
  inline bool failsafe() const {return failsafe_;}
  inline bool lost_frame() const {return lost_frame_;}
  inline bool ch17() const {return ch17_;}
  inline bool ch18() const {return ch18_;}
  sbus::array<uint16_t, 16> tx_channels();

 private:
  /* Communication */
  HardwareSerial *bus_;

  static constexpr uint32_t BAUD_ = 100000;
  /* Parsing */
  static constexpr uint8_t SBUS_HEADER_ = 0x0F;
  static constexpr uint8_t SBUS_FOOTER_ = 0x00;
  static constexpr uint8_t SBUS_LENGTH_ = 25;
  static constexpr uint8_t SBUS_CH17_ = 0x01;
  static constexpr uint8_t SBUS_CH18_ = 0x02;
  static constexpr uint8_t SBUS_LOST_FRAME_ = 0x04;
  static constexpr uint8_t SBUS_FAILSAFE_ = 0x08;
  uint8_t tx_buffer_[SBUS_LENGTH_];
  /* Data */
  sbus::array<uint16_t, 16> tx_channels_;
  bool failsafe_ = false, lost_frame_ = false, ch17_ = false, ch18_ = false;
};

#endif  // INCLUDE_SBUS_SBUS_H_
