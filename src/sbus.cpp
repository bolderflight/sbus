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

#include "sbus.h"  // NOLINT
#if defined(ARDUINO)
#include <Arduino.h>
#else
#include <cstddef>
#include <cstdint>
#include "core/core.h"
#endif

namespace bfs {

void SbusRx::Begin() {
  if (fast_) {
    baud_ = 200000;
  } else {
    baud_ = 100000;
  }
  /* Start the bus */
  /* Teensy 3.0 || Teensy 3.1/3.2 */
  #if defined(__MK20DX128__) || defined(__MK20DX256__)
  if (inv_) {
    uart_->begin(baud_, SERIAL_8E1_RXINV_TXINV);
  } else {
    uart_->begin(baud_, SERIAL_8E1);
  }
  /*
  * Teensy 3.5 || Teensy 3.6 ||
  * Teensy LC  || Teensy 4.0/4.1 ||
  * Teensy 4.0 Beta
  */
  #elif defined(__MK64FX512__) || defined(__MK66FX1M0__) || \
        defined(__MKL26Z64__)  || defined(__IMXRT1062__) || \
        defined(__IMXRT1052__)
  if (inv_){
    uart_->begin(baud_, SERIAL_8E2_RXINV_TXINV);
  } else {
    uart_->begin(baud_, SERIAL_8E2);
  }
  /* STM32L4 */
  #elif defined(STM32L496xx) || defined(STM32L476xx) || \
        defined(STM32L433xx) || defined(STM32L432xx)
  if (inv_) {
    uart_->begin(baud_, SERIAL_8E2 | 0xC000ul);
  } else {
    uart_->begin(baud_, SERIAL_8E2);
  }
  /* ESP32 */
  #elif defined(ESP32)
  uart_->begin(baud_, SERIAL_8E2, rxpin_, txpin_, inv_);
  /* Everything else, with a hardware inverter */
  #else
  uart_->begin(baud_, SERIAL_8E2);
  #endif
  /* flush the bus */
  uart_->flush();
}

bool SbusRx::Read() {
  /* Read through all available packets to get the newest */
  new_data_ = false;
  do {
    if (Parse()) {
      new_data_ = true;
    }
  } while (uart_->available());
  return new_data_;
}

bool SbusRx::Parse() {
  /* Parse messages */
  while (uart_->available()) {
    cur_byte_ = uart_->read();
    if (state_ == 0) {
      if ((cur_byte_ == HEADER_) && ((prev_byte_ == FOOTER_) ||
         ((prev_byte_ & 0x0F) == FOOTER2_))) {
        buf_[state_++] = cur_byte_;
      } else {
        state_ = 0;
      }
    } else if (state_ < PAYLOAD_LEN_ + HEADER_LEN_) {
        buf_[state_++] = cur_byte_;
    } else if (state_ < PAYLOAD_LEN_ + HEADER_LEN_ + FOOTER_LEN_) {
      state_ = 0;
      prev_byte_ = cur_byte_;
      if ((cur_byte_ == FOOTER_) || ((cur_byte_ & 0x0F) == FOOTER2_)) {
        /* Grab the channel data */
        data_.ch[0]  = static_cast<int16_t>(buf_[1] |
                                            ((buf_[2] << 8) & 0x07FF));
        data_.ch[1]  = static_cast<int16_t>((buf_[2] >> 3) |
                                            ((buf_[3] << 5) & 0x07FF));
        data_.ch[2]  = static_cast<int16_t>((buf_[3] >> 6) |
                                            (buf_[4] << 2) |
                                            ((buf_[5] << 10) & 0x07FF));
        data_.ch[3]  = static_cast<int16_t>((buf_[5] >> 1) |
                                            ((buf_[6] << 7) & 0x07FF));
        data_.ch[4]  = static_cast<int16_t>((buf_[6] >> 4) |
                                            ((buf_[7] << 4) & 0x07FF));
        data_.ch[5]  = static_cast<int16_t>((buf_[7] >> 7) |
                                            (buf_[8] << 1) |
                                            ((buf_[9] << 9) & 0x07FF));
        data_.ch[6]  = static_cast<int16_t>((buf_[9] >> 2) |
                                            ((buf_[10] << 6) & 0x07FF));
        data_.ch[7]  = static_cast<int16_t>((buf_[10] >> 5) |
                                            ((buf_[11] << 3) & 0x07FF));
        data_.ch[8]  = static_cast<int16_t>(buf_[12] |
                                            ((buf_[13] << 8) & 0x07FF));
        data_.ch[9]  = static_cast<int16_t>((buf_[13] >> 3) |
                                            ((buf_[14] << 5) & 0x07FF));
        data_.ch[10] = static_cast<int16_t>((buf_[14] >> 6) |
                                            (buf_[15] << 2) |
                                            ((buf_[16] << 10) & 0x07FF));
        data_.ch[11] = static_cast<int16_t>((buf_[16] >> 1) |
                                            ((buf_[17] << 7) & 0x07FF));
        data_.ch[12] = static_cast<int16_t>((buf_[17] >> 4) |
                                            ((buf_[18] << 4) & 0x07FF));
        data_.ch[13] = static_cast<int16_t>((buf_[18] >> 7) |
                                            (buf_[19] << 1) |
                                            ((buf_[20] << 9) & 0x07FF));
        data_.ch[14] = static_cast<int16_t>((buf_[20] >> 2) |
                                            ((buf_[21] << 6) & 0x07FF));
        data_.ch[15] = static_cast<int16_t>((buf_[21] >> 5) |
                                            ((buf_[22] << 3) & 0x07FF));
        /* CH 17 */
        data_.ch17 = buf_[23] & CH17_MASK_;
        /* CH 18 */
        data_.ch18 = buf_[23] & CH18_MASK_;
        /* Grab the lost frame */
        data_.lost_frame = buf_[23] & LOST_FRAME_MASK_;
        /* Grab the failsafe */
        data_.failsafe = buf_[23] & FAILSAFE_MASK_;
        return true;
      } else {
        return false;
      }
    } else {
      state_ = 0;
    }
    prev_byte_ = cur_byte_;
  }
  return false;
}

/* Needed for emulating two stop bytes on Teensy 3.0 and 3.1/3.2 */
#if defined(__MK20DX128__) || defined(__MK20DX256__)
namespace {
  IntervalTimer serial_timer;
  HardwareSerial *sbus_bus;
  uint8_t sbus_packet[25];
  volatile int send_index = 0;
  void SendByte() {
    if (send_index < 25) {
      sbus_bus->write(sbus_packet[send_index]);
      send_index++;
    } else {
      serial_timer.end();
      send_index = 0;
    }
  }
}  // namespace
#endif

void SbusTx::Begin() {
  if (fast_) {
    baud_ = 200000;
  } else {
    baud_ = 100000;
  }
  /* Start the bus */
  /* Teensy 3.0 || Teensy 3.1/3.2 */
  #if defined(__MK20DX128__) || defined(__MK20DX256__)
  sbus_bus = uart_;
  if (inv_) {
    uart_->begin(baud_, SERIAL_8E1_RXINV_TXINV);
  } else {
    uart_->begin(baud_, SERIAL_8E1);
  }
  /*
  * Teensy 3.5 || Teensy 3.6 ||
  * Teensy LC  || Teensy 4.0/4.1 ||
  * Teensy 4.0 Beta
  */
  #elif defined(__MK64FX512__) || defined(__MK66FX1M0__) || \
        defined(__MKL26Z64__)  || defined(__IMXRT1062__) || \
        defined(__IMXRT1052__)
  if (inv_){
    uart_->begin(baud_, SERIAL_8E2_RXINV_TXINV);
  } else {
    uart_->begin(baud_, SERIAL_8E2);
  }
  /* STM32L4 */
  #elif defined(STM32L496xx) || defined(STM32L476xx) || \
        defined(STM32L433xx) || defined(STM32L432xx)
  if (inv_) {
    uart_->begin(baud_, SERIAL_8E2 | 0xC000ul);
  } else {
    uart_->begin(baud_, SERIAL_8E2);
  }
  /* ESP32 */
  #elif defined(ESP32)
  uart_->begin(baud_, SERIAL_8E2, rxpin_, txpin_, inv_);
  /* Everything else, with a hardware inverter */
  #else
  uart_->begin(baud_, SERIAL_8E2);
  #endif
}

void SbusTx::Write() {
  /* Assemble packet */
  buf_[0] = HEADER_;
  buf_[1] = static_cast<uint8_t>((data_.ch[0] & 0x07FF));
  buf_[2] = static_cast<uint8_t>((data_.ch[0] & 0x07FF) >> 8 |
            (data_.ch[1] & 0x07FF) << 3);
  buf_[3] = static_cast<uint8_t>((data_.ch[1] & 0x07FF) >> 5 |
            (data_.ch[2] & 0x07FF) << 6);
  buf_[4] = static_cast<uint8_t>((data_.ch[2] & 0x07FF) >> 2);
  buf_[5] = static_cast<uint8_t>((data_.ch[2] & 0x07FF) >> 10 |
            (data_.ch[3] & 0x07FF) << 1);
  buf_[6] = static_cast<uint8_t>((data_.ch[3] & 0x07FF) >> 7 |
            (data_.ch[4] & 0x07FF) << 4);
  buf_[7] = static_cast<uint8_t>((data_.ch[4] & 0x07FF) >> 4 |
            (data_.ch[5] & 0x07FF) << 7);
  buf_[8] = static_cast<uint8_t>((data_.ch[5] & 0x07FF) >> 1);
  buf_[9] = static_cast<uint8_t>((data_.ch[5] & 0x07FF) >> 9  |
            (data_.ch[6] & 0x07FF) << 2);
  buf_[10] = static_cast<uint8_t>((data_.ch[6] & 0x07FF) >> 6  |
             (data_.ch[7] & 0x07FF) << 5);
  buf_[11] = static_cast<uint8_t>((data_.ch[7] & 0x07FF) >> 3);
  buf_[12] = static_cast<uint8_t>((data_.ch[8] & 0x07FF));
  buf_[13] = static_cast<uint8_t>((data_.ch[8] & 0x07FF) >> 8 |
             (data_.ch[9]  & 0x07FF) << 3);
  buf_[14] = static_cast<uint8_t>((data_.ch[9] & 0x07FF) >> 5 |
             (data_.ch[10] & 0x07FF) << 6);
  buf_[15] = static_cast<uint8_t>((data_.ch[10] & 0x07FF) >> 2);
  buf_[16] = static_cast<uint8_t>((data_.ch[10] & 0x07FF) >> 10 |
             (data_.ch[11] & 0x07FF) << 1);
  buf_[17] = static_cast<uint8_t>((data_.ch[11] & 0x07FF) >> 7  |
             (data_.ch[12] & 0x07FF) << 4);
  buf_[18] = static_cast<uint8_t>((data_.ch[12] & 0x07FF) >> 4  |
             (data_.ch[13] & 0x07FF) << 7);
  buf_[19] = static_cast<uint8_t>((data_.ch[13] & 0x07FF) >> 1);
  buf_[20] = static_cast<uint8_t>((data_.ch[13] & 0x07FF) >> 9  |
             (data_.ch[14] & 0x07FF) << 2);
  buf_[21] = static_cast<uint8_t>((data_.ch[14] & 0x07FF) >> 6  |
             (data_.ch[15] & 0x07FF) << 5);
  buf_[22] = static_cast<uint8_t>((data_.ch[15] & 0x07FF) >> 3);
  buf_[23] = 0x00 | (data_.ch17 * CH17_MASK_) | (data_.ch18 * CH18_MASK_) |
             (data_.failsafe * FAILSAFE_MASK_) |
             (data_.lost_frame * LOST_FRAME_MASK_);
  buf_[24] = FOOTER_;
  /* Send packet to servos */
  #if defined(__MK20DX128__) || defined(__MK20DX256__)
  /*
  * Use ISR to send byte at a time,
  * 130 us between bytes to emulate 2 stop bits
  */
  __disable_irq();
  memcpy(sbus_packet, buf_, sizeof(buf_));
  __enable_irq();
  serial_timer.priority(255);
  serial_timer.begin(SendByte, 130);
  #else
  uart_->write(buf_, sizeof(buf_));
  #endif
}

}  // namespace bfs
