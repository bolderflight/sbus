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

#include <span>
#include <array>
#include <algorithm>
#include "core/core.h"
#include "inceptor/inceptor.h"
#include "effector/effector.h"
#include "polytools/polytools.h"

namespace bfs {

class SbusRx {
 public:
  bool Init(const InceptorConfig &cfg);
  bool Read(InceptorData * const data);

 private:
  /* Configuration */
  InceptorConfig config_;
  /* Communication */
  static constexpr uint32_t BAUD_ = 100000;
  static constexpr unsigned int TIMEOUT_MS_ = 5000;
  /* Parsing */
  static constexpr uint8_t HEADER_ = 0x0F;
  static constexpr uint8_t FOOTER_ = 0x00;
  static constexpr uint8_t FOOTER2_ = 0x04;
  static constexpr uint8_t LEN_ = 25;
  static constexpr uint8_t CH17_ = 0x01;
  static constexpr uint8_t CH18_ = 0x02;
  static constexpr uint8_t LOST_FRAME_ = 0x04;
  static constexpr uint8_t FAILSAFE_ = 0x08;
  unsigned int state_ = 0;
  uint8_t prev_byte_ = FOOTER_;
  uint8_t buf_[LEN_];
  /* Data */
  std::array<uint16_t, 16> ch_;
  bool Parse();
  bool ReadSbus();
};

template<std::size_t N>
class SbusTx {
 public:
  static_assert(N < 17, "Only up to 16 channels supported");
  bool Init(const EffectorConfig<N> &cfg) {
    /* Copy the configuration */
    config_ = cfg;
    /* Initialize the bus */
    if (std::holds_alternative<HardwareSerial *>(config_.hw)) {
      bus_ = std::get<HardwareSerial *>(config_.hw);
      bus_->begin(BAUD_, SERIAL_8E2_RXINV_TXINV);
      return true;
    } else {
      return false;
    }
  }
  void Cmd(std::span<float> cmds) {
    std::size_t len = std::min(cmds.size(), N);
    for (std::size_t i = 0; i < len; i++) {
      /* Saturation */
      if (cmds[i] > config_.effectors[i].max) {
        val_ = config_.effectors[i].max;
      } else if (cmds[i] < config_.effectors[i].min) {
        val_ = config_.effectors[i].min;
      } else {
        val_ = cmds[i];
      }
      /* Motor check */
      if ((config_.effectors[i].type == MOTOR) && (!motors_enabled_)) {
        val_ = config_.effectors[i].failsafe;
      }
      /* Servo check */
      if ((config_.effectors[i].type == SERVO) && (!servos_enabled_)) {
        val_ = config_.effectors[i].failsafe;
      }
      /* polyval */
      std::span<float> coef{config_.effectors[i].poly_coef,
        static_cast<std::size_t>(config_.effectors[i].num_coef)};
      ch_[config_.effectors[i].ch] = static_cast<uint16_t>(
                                     polyval<float>(coef, val_));
    }
  }
  void Write() {
    buf_[0] = HEADER_;
    buf_[1] =  static_cast<uint8_t>((ch_[0]  & 0x07FF));
    buf_[2] =  static_cast<uint8_t>((ch_[0]  & 0x07FF) >> 8  |
              (ch_[1]  & 0x07FF) << 3);
    buf_[3] =  static_cast<uint8_t>((ch_[1]  & 0x07FF) >> 5  |
              (ch_[2]  & 0x07FF) << 6);
    buf_[4] =  static_cast<uint8_t>((ch_[2]  & 0x07FF) >> 2);
    buf_[5] =  static_cast<uint8_t>((ch_[2]  & 0x07FF) >> 10 |
              (ch_[3]  & 0x07FF) << 1);
    buf_[6] =  static_cast<uint8_t>((ch_[3]  & 0x07FF) >> 7  |
              (ch_[4]  & 0x07FF) << 4);
    buf_[7] =  static_cast<uint8_t>((ch_[4]  & 0x07FF) >> 4  |
              (ch_[5]  & 0x07FF) << 7);
    buf_[8] =  static_cast<uint8_t>((ch_[5]  & 0x07FF) >> 1);
    buf_[9] =  static_cast<uint8_t>((ch_[5]  & 0x07FF) >> 9  |
              (ch_[6]  & 0x07FF) << 2);
    buf_[10] = static_cast<uint8_t>((ch_[6]  & 0x07FF) >> 6  |
              (ch_[7]  & 0x07FF) << 5);
    buf_[11] = static_cast<uint8_t>((ch_[7]  & 0x07FF) >> 3);
    buf_[12] = static_cast<uint8_t>((ch_[8]  & 0x07FF));
    buf_[13] = static_cast<uint8_t>((ch_[8]  & 0x07FF) >> 8  |
              (ch_[9]  & 0x07FF) << 3);
    buf_[14] = static_cast<uint8_t>((ch_[9]  & 0x07FF) >> 5  |
              (ch_[10] & 0x07FF) << 6);
    buf_[15] = static_cast<uint8_t>((ch_[10] & 0x07FF) >> 2);
    buf_[16] = static_cast<uint8_t>((ch_[10] & 0x07FF) >> 10 |
              (ch_[11] & 0x07FF) << 1);
    buf_[17] = static_cast<uint8_t>((ch_[11] & 0x07FF) >> 7  |
              (ch_[12] & 0x07FF) << 4);
    buf_[18] = static_cast<uint8_t>((ch_[12] & 0x07FF) >> 4  |
              (ch_[13] & 0x07FF) << 7);
    buf_[19] = static_cast<uint8_t>((ch_[13] & 0x07FF) >> 1);
    buf_[20] = static_cast<uint8_t>((ch_[13] & 0x07FF) >> 9  |
              (ch_[14] & 0x07FF) << 2);
    buf_[21] = static_cast<uint8_t>((ch_[14] & 0x07FF) >> 6  |
              (ch_[15] & 0x07FF) << 5);
    buf_[22] = static_cast<uint8_t>((ch_[15] & 0x07FF) >> 3);
    buf_[23] = 0x00 | (ch17_ * CH17_) | (ch18_ * CH18_) |
              (failsafe_ * FAILSAFE_) | (lost_frame_ * LOST_FRAME_);
    buf_[24] = FOOTER_;
    bus_->write(buf_, sizeof(buf_));
  }
  void EnableMotors() {motors_enabled_ = true;}
  void DisableMotors() {motors_enabled_ = false;}
  void EnableServos() {servos_enabled_ = true;}
  void DisableServos() {servos_enabled_ = false;}

 private:
  /* Configuration */
  EffectorConfig<N> config_;
  /* Communication */
  HardwareSerial *bus_;
  static constexpr uint32_t BAUD_ = 100000;
  /* Parsing */
  static constexpr uint8_t HEADER_ = 0x0F;
  static constexpr uint8_t FOOTER_ = 0x00;
  static constexpr uint8_t LEN_ = 25;
  static constexpr uint8_t CH17_ = 0x01;
  static constexpr uint8_t CH18_ = 0x02;
  static constexpr uint8_t LOST_FRAME_ = 0x04;
  static constexpr uint8_t FAILSAFE_ = 0x08;
  uint8_t buf_[LEN_];
  /* Data */
  bool motors_enabled_ = false, servos_enabled_ = false;
  bool lost_frame_ = false, failsafe_ = false;
  bool ch17_ = false, ch18_ = false;
  float val_;
  std::array<uint16_t, 16> ch_ = {0};
};

}  // namespace bfs

#endif  // INCLUDE_SBUS_SBUS_H_
