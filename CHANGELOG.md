# Changelog

## v2.1.1
- Added a check for lost-link / failsafe in SbusRx.Begin()

## v2.1.0
- Added a timeout to the SbusRx.Begin() method to indicate whether a packet was received

## v2.0.0
- Updated to namespace *bfs*
- Updated sensors::Sbus to SbusRx and actuators::Sbus to SbusTx

## v1.1.6
- Added Channel 17 and 18 support
- Added ability to set lost_frame and failsafe for transmitting
- Added suport for SBUS2 with telemetry enabled

## v1.0.6
- Updated for v2.0.4 of the core library

## v1.0.5
- Updated for v2.0.3 of the core library

## v1.0.4
- Added support for Teensy 4.x

## v1.0.3
- Updated CONTRIBUTING
- Updated *fetch_content* to use https instead of ssh
- Updated *flash_mcu.cmake* to use local loader on Linux

## v1.0.2
- Updated to MIT license.
- Specified version of core lib.

## v1.0.1
- Modified the sensors::Sbus::Begin method to flush the serial buffer and the sensors::Sbus::Read method to read through multiple packets up to the most recent. This should help the code keep up with packets when running at slower rates. 

## v1.0.0
- Initial commit
