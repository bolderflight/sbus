# Changelog

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
