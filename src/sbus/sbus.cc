/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "sbus/sbus.h"

Sbus::Sbus(HardwareSerial &bus) {
  bus_ = &bus;
}
void Sbus::Begin() {
  parser_state_ = 0;
  previous_byte_ = SBUS_FOOTER_;
  bus_->begin(BAUD_, SERIAL_8E2_RXINV_TXINV);
}
bool Sbus::Read() {
  if (Parse()) {
    /* Grab the channel data */
    rx_channels_[0]  = (uint16_t) ((rx_buffer_[1]        | rx_buffer_[2]  << 8)                         & 0x07FF);
    rx_channels_[1]  = (uint16_t) ((rx_buffer_[2]  >> 3  | rx_buffer_[3]  << 5)                         & 0x07FF);
    rx_channels_[2]  = (uint16_t) ((rx_buffer_[3]  >> 6  | rx_buffer_[4]  << 2  | rx_buffer_[5] << 10)  & 0x07FF);
    rx_channels_[3]  = (uint16_t) ((rx_buffer_[5]  >> 1  | rx_buffer_[6]  << 7)                         & 0x07FF);
    rx_channels_[4]  = (uint16_t) ((rx_buffer_[6]  >> 4  | rx_buffer_[7]  << 4)                         & 0x07FF);
    rx_channels_[5]  = (uint16_t) ((rx_buffer_[7]  >> 7  | rx_buffer_[8]  << 1  | rx_buffer_[9] << 9)   & 0x07FF);
    rx_channels_[6]  = (uint16_t) ((rx_buffer_[9]  >> 2  | rx_buffer_[10] << 6)                         & 0x07FF);
    rx_channels_[7]  = (uint16_t) ((rx_buffer_[10]  >> 5 | rx_buffer_[11] << 3)                         & 0x07FF);
    rx_channels_[8]  = (uint16_t) ((rx_buffer_[12]       | rx_buffer_[13] << 8)                         & 0x07FF);
    rx_channels_[9]  = (uint16_t) ((rx_buffer_[13] >> 3  | rx_buffer_[14] << 5)                         & 0x07FF);
    rx_channels_[10] = (uint16_t) ((rx_buffer_[14] >> 6  | rx_buffer_[15] << 2  | rx_buffer_[16] << 10) & 0x07FF);
    rx_channels_[11] = (uint16_t) ((rx_buffer_[16] >> 1  | rx_buffer_[17] << 7)                         & 0x07FF);
    rx_channels_[12] = (uint16_t) ((rx_buffer_[17] >> 4  | rx_buffer_[18] << 4)                         & 0x07FF);
    rx_channels_[13] = (uint16_t) ((rx_buffer_[18] >> 7  | rx_buffer_[19] << 1  | rx_buffer_[20] << 9)  & 0x07FF);
    rx_channels_[14] = (uint16_t) ((rx_buffer_[20] >> 2  | rx_buffer_[21] << 6)                         & 0x07FF);
    rx_channels_[15] = (uint16_t) ((rx_buffer_[21] >> 5  | rx_buffer_[22] << 3)                         & 0x07FF);
    /* Grab the lost frame */
    lost_frame_ = rx_buffer_[23] & SBUS_LOST_FRAME_;
    /* Grab the failsafe */
    failsafe_ = rx_buffer_[23] & SBUS_FAILSAFE_;
    return true;
  } else {
    return false;
  }
}
void Sbus::Write() {
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
  tx_buffer_[23] = 0x00;
  tx_buffer_[24] = SBUS_FOOTER_;
  bus_->write(tx_buffer_, sizeof(tx_buffer_));
}
std::array<uint16_t, 16> Sbus::rx_channels() {
  return rx_channels_;
}
std::array<uint16_t, 16> Sbus::tx_channels() {
  return tx_channels_;
}
void Sbus::tx_channels(const std::array<uint16_t, 16> &val) {
  tx_channels_ = val;
}
bool Sbus::failsafe() {
  return failsafe_;
}
bool Sbus::lost_frame() {
  return lost_frame_;
}
void Sbus::End() {
  bus_->end();
}
bool Sbus::Parse() {
  while (bus_->available()) {
    uint8_t c = bus_->read();
    if (parser_state_ == 0) {
      if ((c == SBUS_HEADER_) && (previous_byte_ == SBUS_FOOTER_)) {
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
        if (rx_buffer_[SBUS_LENGTH_ - 1] == SBUS_FOOTER_) {
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
