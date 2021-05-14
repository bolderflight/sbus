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
#include "inceptor/inceptor.h"
#include "polytools/polytools.h"

namespace bfs {

bool SbusRx::Init(const InceptorConfig &cfg) {
  /* Copy the config */
  config_ = cfg;
  /* Start the bus */
  config_.hw->begin(BAUD_, SERIAL_8E2_RXINV_TXINV);
  /* flush the bus */
  config_.hw->flush();
  /* check communication */
  elapsedMillis timer_ms = 0;
  while (timer_ms < TIMEOUT_MS_) {
    if (Parse()) {
      bool lost_frame = buf_[23] & LOST_FRAME_;
      bool failsafe = buf_[23] & FAILSAFE_;
      if (!failsafe && !lost_frame) {
        return true;
      }
    }
  }
  return false;
}
bool SbusRx::Read(InceptorData * const ptr) {
  if (!ptr) {return false;}
  /* Read through all available packets to get the newest */
  ptr->new_data = false;
  do {
    if (Parse()) {
      ptr->new_data = true;
    }
  } while (config_.hw->available());
  /* Parse new data, if available */
  if (ptr->new_data) {
    /* Grab the channel data */
    ch_[0]  = static_cast<uint16_t>(buf_[1]       | buf_[2]  << 8 & 0x07FF);
    ch_[1]  = static_cast<uint16_t>(buf_[2]  >> 3 | buf_[3]  << 5 & 0x07FF);
    ch_[2]  = static_cast<uint16_t>(buf_[3]  >> 6 | buf_[4]  << 2  |
              buf_[5] << 10 & 0x07FF);
    ch_[3]  = static_cast<uint16_t>(buf_[5]  >> 1 | buf_[6]  << 7 & 0x07FF);
    ch_[4]  = static_cast<uint16_t>(buf_[6]  >> 4 | buf_[7]  << 4 & 0x07FF);
    ch_[5]  = static_cast<uint16_t>(buf_[7]  >> 7 | buf_[8]  << 1  |
              buf_[9] << 9 & 0x07FF);
    ch_[6]  = static_cast<uint16_t>(buf_[9]  >> 2 | buf_[10] << 6 & 0x07FF);
    ch_[7]  = static_cast<uint16_t>(buf_[10] >> 5 | buf_[11] << 3 & 0x07FF);
    ch_[8]  = static_cast<uint16_t>(buf_[12]      | buf_[13] << 8 & 0x07FF);
    ch_[9]  = static_cast<uint16_t>(buf_[13] >> 3 | buf_[14] << 5 & 0x07FF);
    ch_[10] = static_cast<uint16_t>(buf_[14] >> 6 | buf_[15] << 2  |
              buf_[16] << 10 & 0x07FF);
    ch_[11] = static_cast<uint16_t>(buf_[16] >> 1 | buf_[17] << 7 & 0x07FF);
    ch_[12] = static_cast<uint16_t>(buf_[17] >> 4 | buf_[18] << 4 & 0x07FF);
    ch_[13] = static_cast<uint16_t>(buf_[18] >> 7 | buf_[19] << 1  |
              buf_[20] << 9 & 0x07FF);
    ch_[14] = static_cast<uint16_t>(buf_[20] >> 2 | buf_[21] << 6 & 0x07FF);
    ch_[15] = static_cast<uint16_t>(buf_[21] >> 5 | buf_[22] << 3 & 0x07FF);
    /* Grab the lost frame */
    ptr->lost_frame = buf_[23] & LOST_FRAME_;
    /* Grab the failsafe */
    ptr->failsafe = buf_[23] & FAILSAFE_;
    /* Throttle enable */
    std::span<float> thr_en_coef{config_.throttle_en.poly_coef,
        static_cast<std::size_t>(config_.throttle_en.num_coef)};
    ptr->throttle_en = static_cast<bool>(polyval<float>(thr_en_coef,
                                         ch_[config_.throttle_en.ch]));
    /* mode0 */
    std::span<float> mode0_coef{config_.mode0.poly_coef,
        static_cast<std::size_t>(config_.mode0.num_coef)};
    ptr->mode0 = static_cast<int8_t>(polyval<float>(mode0_coef,
                                     ch_[config_.mode0.ch]));
    /* mode1 */
    std::span<float> mode1_coef{config_.mode1.poly_coef,
        static_cast<std::size_t>(config_.mode1.num_coef)};
    ptr->mode1 = static_cast<int8_t>(polyval<float>(mode1_coef,
                                     ch_[config_.mode1.ch]));
    /* throttle */
    std::span<float> throttle_coef{config_.throttle.poly_coef,
        static_cast<std::size_t>(config_.throttle.num_coef)};
    ptr->throttle = polyval<float>(throttle_coef, ch_[config_.throttle.ch]);
    /* pitch */
    std::span<float> pitch_coef{config_.pitch.poly_coef,
        static_cast<std::size_t>(config_.pitch.num_coef)};
    ptr->pitch = polyval<float>(pitch_coef, ch_[config_.pitch.ch]);
    /* roll */
    std::span<float> roll_coef{config_.roll.poly_coef,
        static_cast<std::size_t>(config_.roll.num_coef)};
    ptr->roll = polyval<float>(roll_coef, ch_[config_.roll.ch]);
    /* yaw */
    std::span<float> yaw_coef{config_.yaw.poly_coef,
        static_cast<std::size_t>(config_.yaw.num_coef)};
    ptr->yaw = polyval<float>(yaw_coef, ch_[config_.yaw.ch]);
  }
  return ptr->new_data;
}
bool SbusRx::Parse() {
  while (config_.hw->available()) {
    uint8_t c = config_.hw->read();
    if (state_ == 0) {
      if ((c == HEADER_) && ((prev_byte_ == FOOTER_) ||
         ((prev_byte_ & 0x0F) == FOOTER2_))) {
        buf_[state_] = c;
        state_++;
      } else {
        state_ = 0;
      }
    } else {
      if (state_ < LEN_) {
        buf_[state_] = c;
        state_++;
      } else {
        state_ = 0;
        if ((buf_[LEN_ - 1] == FOOTER_) ||
           ((buf_[LEN_ - 1] & 0x0F) == FOOTER2_)) {
          return true;
        } else {
          return false;
        }
      }
    }
    prev_byte_ = c;
  }
  return false;
}

}  // namespace bfs
