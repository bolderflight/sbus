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

namespace sensors {

class Sbus {
 public:
  explicit Sbus(HardwareSerial *bus) : bus_(bus) {}
  void Begin();
  bool Read();
  inline std::array<uint16_t, 16> rx_channels() const {return rx_channels_;}
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
  std::array<uint16_t, 16> rx_channels_;
  bool failsafe_ = false, lost_frame_ = false, ch17_ = false, ch18_ = false;
  bool Parse();
};

}  // namespace sensors

namespace actuators {

class Sbus {
 public:
  explicit Sbus(HardwareSerial *bus) : bus_(bus) {}
  void Begin();
  void Write();
  inline void failsafe(bool val) {failsafe_ = val;}
  inline void lost_frame(bool val) {lost_frame_ = val;}
  inline void ch17(bool val) {ch17_ = val;}
  inline void ch18(bool val) {ch18_ = val;}
  inline void tx_channels(const std::array<uint16_t, 16> &val) {tx_channels_ = val;}
  inline bool failsafe() const {return failsafe_;}
  inline bool lost_frame() const {return lost_frame_;}
  inline bool ch17() const {return ch17_;}
  inline bool ch18() const {return ch18_;}
  inline std::array<uint16_t, 16> tx_channels() const {return tx_channels_;}

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
  std::array<uint16_t, 16> tx_channels_;
  bool failsafe_ = false, lost_frame_ = false, ch17_ = false, ch18_ = false;
};

}  // namespace actuators

#endif  // INCLUDE_SBUS_SBUS_H_
