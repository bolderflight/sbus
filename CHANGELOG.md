# Changelog

## v8.1.4
- Fixed a bug where trying to write SBUS packets with Teensy 3.1/3.2 would hang the processor by de-referencing a null pointer

## v8.1.3
- Updated core to v3.1.3

## v8.1.2
- Updated core to v3.1.2

## v8.1.1
- Updated core to support MMOD

## v8.1.0
- Added option for fast SBUS (200000 baud)

## v8.0.1
- Enabling ESP32 to use non-inverted SBUS

## v8.0.0
- Fixed bug in SbusRx timing where the last packet waited for the start of the next packet before returning true
- Added option to specify a non-inverted signal for cases where that is a hardware option
- Removed std::array dependency to facilitate use on Arduino AVR
- Updated ESP32 implementation for greater consistency to other processors
- Implemented data struct to ease reading and setting SBUS values without needing std::array

## v7.0.0
- Using std::array for passing around SBUS data and commands

## v6.0.3
- Cleaned up folder structure after merge
- Pulling in mcu-support repo for CMake builds
- Fixed pointer constness for SbusRx

## v6.0.2
- Removed the tools folder to fix Arduino linting

## v6.0.1
- Fixing library.properties version number

## v6.0.0
- Merging CMake and Arduino SBUS libraries. Version 6 is the next available version number common between the two.

## v2.1.2
- Added Embedded Template Library support for AVR boards.

## v2.1.1
- Added CH17 and 18 support for read and write
- Added capability to write lost_frame or failsafe

## v2.0.1
- Added back in support for SBUS2 footers

## v2.0.0
- Updated to match our [SBUS](https://github.com/bolderflight/sbus) library for flight software
- Updated license to MIT

## v1.0.1
- Updated license to GPLV3.

## v1.0.0
- Updated to Arduino 1.5 format and setting a baseline release here.
