/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_SBUS_SBUS_H_
#define INCLUDE_SBUS_SBUS_H_

#include <array>
#include "core/core.h"

class Sbus {
 public:
  explicit Sbus(HardwareSerial *bus);
  void Begin();
  bool Read();
  void Write();
  std::array<uint16_t, 16> rx_channels();
  std::array<uint16_t, 16> tx_channels();
  void tx_channels(const std::array<uint16_t, 16> &val);
  bool failsafe();
  bool lost_frame();
  void End();

 private:
  /* Communication */
  HardwareSerial *bus_;
  static constexpr uint32_t BAUD_ = 100000;
  /* Parsing */
  static constexpr uint8_t SBUS_HEADER_ = 0x0F;
  static constexpr uint8_t SBUS_FOOTER_ = 0x00;
  static constexpr uint8_t SBUS_LENGTH_ = 25;
  static constexpr uint8_t SBUS_LOST_FRAME_ = 0x04;
  static constexpr uint8_t SBUS_FAILSAFE_ = 0x08;
  unsigned int parser_state_ = 0;
  uint8_t previous_byte_ = SBUS_FOOTER_;
  uint8_t rx_buffer_[SBUS_LENGTH_];
  uint8_t tx_buffer_[SBUS_LENGTH_];
  /* Data */
  std::array<uint16_t, 16> rx_channels_;
  std::array<uint16_t, 16> tx_channels_;
  bool failsafe_ = false, lost_frame_ = false;
  bool Parse();
};

#endif  // INCLUDE_SBUS_SBUS_H_
