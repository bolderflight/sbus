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

#include "Arduino.h"
#include "sbus.h"

#ifdef ESP32
SbusRx::SbusRx(HardwareSerial *bus, uint8_t rxpin, uint8_t txpin) {
  bus_ = bus;
  rxpin_ = rxpin;
  txpin_ = txpin;
}
#else
SbusRx::SbusRx(HardwareSerial *bus) {
  bus_ = bus;
}
#endif

void SbusRx::Begin() {
  parser_state_ = 0;
  previous_byte_ = SBUS_FOOTER_;
  /* Teensy 3.0 || Teensy 3.1/3.2 */
  #if defined(__MK20DX128__) || defined(__MK20DX256__)
    bus_->begin(BAUD_, SERIAL_8E1_RXINV_TXINV);
  /* Teensy 3.5 || Teensy 3.6 || Teensy 4.0/4.1 || Teensy 4.0 Beta */
  #elif defined(__MK64FX512__) || defined(__MK66FX1M0__) || \
        defined(__MKL26Z64__)  || defined(__IMXRT1062__) || \
        defined(__IMXRT1052__)
    bus_->begin(BAUD_, SERIAL_8E2_RXINV_TXINV);
  /* STM32L4 */
  #elif defined(STM32L496xx) || defined(STM32L476xx) || \
        defined(STM32L433xx) || defined(STM32L432xx)
    bus_->begin(BAUD_, SERIAL_SBUS);
  /* ESP32 */
  #elif defined(ESP32)
    bus_->begin(BAUD_, SERIAL_8E2, rxpin_, txpin_, true);
   /* Everything else, with a hardware inverter */
  #else
    bus_->begin(BAUD_, SERIAL_8E2);
  #endif
  /* flush the bus */
  bus_->flush();
}
bool SbusRx::Read() {
  bool status = false;
  /*  Parse through to the most recent packet if we've fallen behind */
  while (bus_->available()) {
    if (Parse()) {
      /* Grab the channel data */
      rx_channels_[0]  = (uint16_t) ((rx_buffer_[1]        | rx_buffer_[2]  << 8)                         & 0x07FF);
      rx_channels_[1]  = (uint16_t) ((rx_buffer_[2]  >> 3  | rx_buffer_[3]  << 5)                         & 0x07FF);
      rx_channels_[2]  = (uint16_t) ((rx_buffer_[3]  >> 6  | rx_buffer_[4]  << 2  | rx_buffer_[5] << 10)  & 0x07FF);
      rx_channels_[3]  = (uint16_t) ((rx_buffer_[5]  >> 1  | rx_buffer_[6]  << 7)                         & 0x07FF);
      rx_channels_[4]  = (uint16_t) ((rx_buffer_[6]  >> 4  | rx_buffer_[7]  << 4)                         & 0x07FF);
      rx_channels_[5]  = (uint16_t) ((rx_buffer_[7]  >> 7  | rx_buffer_[8]  << 1  | rx_buffer_[9] << 9)   & 0x07FF);
      rx_channels_[6]  = (uint16_t) ((rx_buffer_[9]  >> 2  | rx_buffer_[10] << 6)                         & 0x07FF);
      rx_channels_[7]  = (uint16_t) ((rx_buffer_[10] >> 5  | rx_buffer_[11] << 3)                         & 0x07FF);
      rx_channels_[8]  = (uint16_t) ((rx_buffer_[12]       | rx_buffer_[13] << 8)                         & 0x07FF);
      rx_channels_[9]  = (uint16_t) ((rx_buffer_[13] >> 3  | rx_buffer_[14] << 5)                         & 0x07FF);
      rx_channels_[10] = (uint16_t) ((rx_buffer_[14] >> 6  | rx_buffer_[15] << 2  | rx_buffer_[16] << 10) & 0x07FF);
      rx_channels_[11] = (uint16_t) ((rx_buffer_[16] >> 1  | rx_buffer_[17] << 7)                         & 0x07FF);
      rx_channels_[12] = (uint16_t) ((rx_buffer_[17] >> 4  | rx_buffer_[18] << 4)                         & 0x07FF);
      rx_channels_[13] = (uint16_t) ((rx_buffer_[18] >> 7  | rx_buffer_[19] << 1  | rx_buffer_[20] << 9)  & 0x07FF);
      rx_channels_[14] = (uint16_t) ((rx_buffer_[20] >> 2  | rx_buffer_[21] << 6)                         & 0x07FF);
      rx_channels_[15] = (uint16_t) ((rx_buffer_[21] >> 5  | rx_buffer_[22] << 3)                         & 0x07FF);
      /* CH 17 */
      ch17_ = rx_buffer_[23] & SBUS_CH17_;
      /* CH 18 */
      ch18_ = rx_buffer_[23] & SBUS_CH18_;
      /* Grab the lost frame */
      lost_frame_ = rx_buffer_[23] & SBUS_LOST_FRAME_;
      /* Grab the failsafe */
      failsafe_ = rx_buffer_[23] & SBUS_FAILSAFE_;
      /* Set the status */
      status = true;
    }
  }
  return status;
}
sbus::array<uint16_t, 16> SbusRx::rx_channels() {
  return rx_channels_;
}
bool SbusRx::Parse() {
  while (bus_->available()) {
    uint8_t c = bus_->read();
    if (parser_state_ == 0) {
      if ((c == SBUS_HEADER_) && ((previous_byte_ == SBUS_FOOTER_) || ((previous_byte_ & 0x0F) == SBUS2_FOOTER_))) {
        rx_buffer_[parser_state_] = c;
        parser_state_++;
      } else {
        parser_state_ = 0;
      }
    } else {
      if (parser_state_ < SBUS_LENGTH_) {
        rx_buffer_[parser_state_] = c;
        parser_state_++;
      } else {
        parser_state_ = 0;
        if ((rx_buffer_[SBUS_LENGTH_ - 1] == SBUS_FOOTER_) || ((rx_buffer_[SBUS_LENGTH_ - 1] & 0x0F) == SBUS2_FOOTER_)) {
          return true;
        } else {
          return false;
        }
      }
    }
    previous_byte_ = c;
  }
  return false;
}

/* Needed for emulating two stop bytes on Teensy 3.0 and 3.1/3.2 */
#if defined(__MK20DX128__) || defined(__MK20DX256__)
namespace {
  IntervalTimer serial_timer;
  HardwareSerial *sbus_bus;
  uint8_t sbus_packet[25];
  volatile int send_index;
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

#ifdef ESP32
SbusTx::SbusTx(HardwareSerial *bus, uint8_t rxpin, uint8_t txpin) {
  bus_ = bus;
  rxpin_ = rxpin;
  txpin_ = txpin;
}
#else
SbusTx::SbusTx(HardwareSerial *bus) {
  bus_ = bus;
}
#endif

void SbusTx::Begin() {
  /* Teensy 3.0 || Teensy 3.1/3.2 */
  #if defined(__MK20DX128__) || defined(__MK20DX256__)
    bus_->begin(BAUD_, SERIAL_8E1_RXINV_TXINV);
  /* Teensy 3.5 || Teensy 3.6 || Teensy 4.0/4.1 || Teensy 4.0 Beta */
  #elif defined(__MK64FX512__) || defined(__MK66FX1M0__) || \
        defined(__MKL26Z64__)  || defined(__IMXRT1062__) || \
        defined(__IMXRT1052__)
    bus_->begin(BAUD_, SERIAL_8E2_RXINV_TXINV);
  /* STM32L4 */
  #elif defined(STM32L496xx) || defined(STM32L476xx) || \
        defined(STM32L433xx) || defined(STM32L432xx)
    bus_->begin(BAUD_, SERIAL_SBUS);
  /* ESP32 */
  #elif defined(ESP32)
    bus_->begin(BAUD_, SERIAL_8E2, rxpin_, txpin_, true);
  /* Everything else, with a hardware inverter */
  #else
    bus_->begin(BAUD_, SERIAL_8E2);
  #endif
}
void SbusTx::Write() {
  tx_buffer_[0] = SBUS_HEADER_;
  tx_buffer_[1] =   (uint8_t) ((tx_channels_[0]   & 0x07FF));
  tx_buffer_[2] =   (uint8_t) ((tx_channels_[0]   & 0x07FF) >> 8  | (tx_channels_[1]  & 0x07FF) << 3);
  tx_buffer_[3] =   (uint8_t) ((tx_channels_[1]   & 0x07FF) >> 5  | (tx_channels_[2]  & 0x07FF) << 6);
  tx_buffer_[4] =   (uint8_t) ((tx_channels_[2]   & 0x07FF) >> 2);
  tx_buffer_[5] =   (uint8_t) ((tx_channels_[2]   & 0x07FF) >> 10 | (tx_channels_[3]  & 0x07FF) << 1);
  tx_buffer_[6] =   (uint8_t) ((tx_channels_[3]   & 0x07FF) >> 7  | (tx_channels_[4]  & 0x07FF) << 4);
  tx_buffer_[7] =   (uint8_t) ((tx_channels_[4]   & 0x07FF) >> 4  | (tx_channels_[5]  & 0x07FF) << 7);
  tx_buffer_[8] =   (uint8_t) ((tx_channels_[5]   & 0x07FF) >> 1);
  tx_buffer_[9] =   (uint8_t) ((tx_channels_[5]   & 0x07FF) >> 9  | (tx_channels_[6]  & 0x07FF) << 2);
  tx_buffer_[10] =  (uint8_t) ((tx_channels_[6]   & 0x07FF) >> 6  | (tx_channels_[7]  & 0x07FF) << 5);
  tx_buffer_[11] =  (uint8_t) ((tx_channels_[7]   & 0x07FF) >> 3);
  tx_buffer_[12] =  (uint8_t) ((tx_channels_[8]   & 0x07FF));
  tx_buffer_[13] =  (uint8_t) ((tx_channels_[8]   & 0x07FF) >> 8  | (tx_channels_[9]  & 0x07FF) << 3);
  tx_buffer_[14] =  (uint8_t) ((tx_channels_[9]   & 0x07FF) >> 5  | (tx_channels_[10] & 0x07FF) << 6);
  tx_buffer_[15] =  (uint8_t) ((tx_channels_[10]  & 0x07FF) >> 2);
  tx_buffer_[16] =  (uint8_t) ((tx_channels_[10]  & 0x07FF) >> 10 | (tx_channels_[11] & 0x07FF) << 1);
  tx_buffer_[17] =  (uint8_t) ((tx_channels_[11]  & 0x07FF) >> 7  | (tx_channels_[12] & 0x07FF) << 4);
  tx_buffer_[18] =  (uint8_t) ((tx_channels_[12]  & 0x07FF) >> 4  | (tx_channels_[13] & 0x07FF) << 7);
  tx_buffer_[19] =  (uint8_t) ((tx_channels_[13]  & 0x07FF) >> 1);
  tx_buffer_[20] =  (uint8_t) ((tx_channels_[13]  & 0x07FF) >> 9  | (tx_channels_[14] & 0x07FF) << 2);
  tx_buffer_[21] =  (uint8_t) ((tx_channels_[14]  & 0x07FF) >> 6  | (tx_channels_[15] & 0x07FF) << 5);
  tx_buffer_[22] =  (uint8_t) ((tx_channels_[15]  & 0x07FF) >> 3);
  tx_buffer_[23] = 0x00 | (ch17_ * SBUS_CH17_) | (ch18_ * SBUS_CH18_) | (failsafe_ * SBUS_FAILSAFE_) | (lost_frame_ * SBUS_LOST_FRAME_);
  tx_buffer_[24] = SBUS_FOOTER_;

  #if defined(__MK20DX128__) || defined(__MK20DX256__)
    /* 
    * Use ISR to send byte at a time,
    * 130 us between bytes to emulate 2 stop bits
    */
    __disable_irq();
    memcpy(sbus_packet, tx_buffer_, sizeof(sbus_packet));
    __enable_irq();
    serial_timer.priority(255);
    serial_timer.begin(SendByte, 130);
  #else
    bus_->write(tx_buffer_, sizeof(tx_buffer_));
  #endif
}
sbus::array<uint16_t, 16> SbusTx::tx_channels() {
  return tx_channels_;
}
void SbusTx::tx_channels(const sbus::array<uint16_t, 16> &val) {
  tx_channels_ = val;
}
