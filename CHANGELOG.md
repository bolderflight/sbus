# Changelog

## v1.0.1

- Modified the sensors::Sbus::Begin method to flush the serial buffer and the sensors::Sbus::Read method to read through multiple packets up to the most recent. This should help the code keep up with packets when running at slower rates. 

## v1.0.0

- Initial commit
