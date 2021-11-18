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
#include "core/core.h"

namespace bfs {

class SbusRx {
 public:
  bool Init(HardwareSerial *uart);
  bool Read();
  static constexpr int8_t NUM_CH = 16;
  inline std::array<int16_t, NUM_CH> ch() const {return ch_;}
  inline bool ch17() const {return ch17_;}
  inline bool ch18() const {return ch18_;}
  inline bool lost_frame() const {return lost_frame_;}
  inline bool failsafe() const {return failsafe_;}

 private:
  /* Communication */
  HardwareSerial *uart_;
  static constexpr int32_t BAUD_ = 100000;
  static constexpr int16_t TIMEOUT_MS_ = 5000;
  /* Message len */
  static constexpr int8_t BUF_LEN_ = 25;
  /* SBUS message defs */
  static constexpr uint8_t HEADER_ = 0x0F;
  static constexpr uint8_t FOOTER_ = 0x00;
  static constexpr uint8_t FOOTER2_ = 0x04;
  static constexpr uint8_t CH17_MASK_ = 0x01;
  static constexpr uint8_t CH18_MASK_ = 0x02;
  static constexpr uint8_t LOST_FRAME_MASK_ = 0x04;
  static constexpr uint8_t FAILSAFE_MASK_ = 0x08;
  /* Parsing state tracking */
  int8_t state_ = 0;
  uint8_t prev_byte_ = FOOTER_;
  uint8_t cur_byte_;
  /* Buffer for storing messages */
  uint8_t buf_[BUF_LEN_];
  /* Data */
  bool new_data_;
  std::array<int16_t, NUM_CH> ch_ = {0};
  bool ch17_ = false, ch18_ = false;
  bool failsafe_ = false, lost_frame_ = false;
  bool Parse();
};

class SbusTx {
 public:
  void Init(HardwareSerial *uart);
  void Write();
  static constexpr int8_t NUM_CH = 16;
  inline void ch(const std::array<int16_t, NUM_CH> &ref) {ch_ = ref;}
  inline void ch17(const bool val) {ch17_ = val;}
  inline void ch18(const bool val) {ch18_ = val;}
  inline void lost_frame(const bool val) {lost_frame_ = val;}
  inline void failsafe(const bool val) {failsafe_ = val;}
  inline std::array<int16_t, NUM_CH> ch() const {return ch_;}
  inline bool ch17() const {return ch17_;}
  inline bool ch18() const {return ch18_;}
  inline bool lost_frame() const {return lost_frame_;}
  inline bool failsafe() const {return failsafe_;}

 private:
  /* Communication */
  HardwareSerial *uart_;
  static constexpr int32_t BAUD_ = 100000;
  /* Message len */
  static constexpr int8_t BUF_LEN_ = 25;
  /* SBUS message defs */
  static constexpr uint8_t HEADER_ = 0x0F;
  static constexpr uint8_t FOOTER_ = 0x00;
  static constexpr uint8_t CH17_MASK_ = 0x01;
  static constexpr uint8_t CH18_MASK_ = 0x02;
  static constexpr uint8_t LOST_FRAME_MASK_ = 0x04;
  static constexpr uint8_t FAILSAFE_MASK_ = 0x08;
  /* Buffer for storing messages */
  uint8_t buf_[BUF_LEN_];
  /* Data */
  std::array<int16_t, NUM_CH> ch_ = {0};
  bool ch17_ = false, ch18_ = false;
  bool failsafe_ = false, lost_frame_ = false;
};

}  // namespace bfs

#endif  // INCLUDE_SBUS_SBUS_H_
