[![Pipeline](https://gitlab.com/bolderflight/software/sbus/badges/main/pipeline.svg)](https://gitlab.com/bolderflight/software/sbus/) [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

![Bolder Flight Systems Logo](img/logo-words_75.png) &nbsp; &nbsp; ![Arduino Logo](img/arduino_logo_75.png)

# Sbus
This library communicates with SBUS receivers and servos and is compatible with Arduino and CMake build systems.
   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)

# Description
SBUS is a bus protocol for receivers to send commands to servos. Unlike PWM, SBUS uses a bus architecture where a single serial line can be connected with up to 16 servos with each receiving a unique command.

The SBUS protocol uses an inverted serial logic with a baud rate of 100000, 8 data bits, even parity, and 2 stop bits. The SBUS packet is 25 bytes long consisting of:
   * Byte[0]: SBUS header, 0x0F
   * Byte[1 -22]: 16 servo channels, 11 bits each
   * Byte[23]
      * Bit 7: channel 17 (0x80)
      * Bit 6: channel 18 (0x40)
      * Bit 5: frame lost (0x20)
      * Bit 4: failsafe activated (0x10)
   * Byte[24]: SBUS footer

Note that lost frame is indicated when a frame is lost between the transmitter and receiver. Failsafe activation typically requires that many frames are lost in a row and indicates that the receiver has moved into failsafe mode. Packets are sent approximately every 10 ms or 20 ms. 

**Note on CH17 and CH18:** Channel 17 and channel 18 are digital on/off channels. These are not universally available on all SBUS receivers and servos.

FrSky receivers will output a range of 172 - 1811 with channels set to a range of -100% to +100%. Using extended limits of -150% to +150% outputs a range of 0 to 2047, which is the maximum range acheivable with 11 bits of data.

Because SBUS is a digital bus format, it is an excellent means of receiving pilot commands from a transmitter and an SBUS capable receiver. If SBUS servos are used in the aircraft, SBUS is also an excellent means of sending actuator commands - servo commands can often be sent with lower latency and, by only using a single pin to command up to 16 servos, additional microcontroller pins are freed for other uses.

# Inverted Serial
SBUS uses an inverted serial protocol, which is not commonly supported in Arduino. This library is able to use inverted serial for the following microcontrollers:
   * Teensy 3.x
   * Teensy 4.x
   * Teensy LC
   * STM32L496xx
   * STM32L476xx
   * STM32L433xx
   * STM32L432xx
   * ESP32

For all other microcontrollers, you **must** use a serial inverter.

# Installation

## Arduino
Simply clone or download and extract the zipped library into your Arduino/libraries folder. The library is added as:

```C++
#include "sbus.h"
```

## CMake
CMake is used to build this library, which is exported as a library target called *sbus*. The header is added as:

```C++
#include "sbus.h"
```

The library can be also be compiled stand-alone using the CMake idiom of creating a *build* directory and then, from within that directory issuing:

```
cmake .. -DMCU=MK66FX1M0
make
```

This will build the library and example executable called *sbus_example*. The example executable source file is located at *examples/cmake/sbus_example.cc*. Notice that the *cmake* command includes a define specifying the microcontroller the code is being compiled for. This is required to correctly configure the code, CPU frequency, and compile/linker options. The available MCUs are:
   * MK64FX512
   * MK66FX1M0
   * MKL26Z64
   * IMXRT1062_T40
   * IMXRT1062_T41

These are known to work with the same packages used in Teensy products. Also switching packages is known to work well, as long as it's only a package change.

The *sbus_example* target creates an executable for communicating with sbus receivers and servos. This target also has a *_hex* for creating the hex file and an *_upload* for using the [Teensy CLI Uploader](https://www.pjrc.com/teensy/loader_cli.html) to flash the Teensy. Please note that the CMake build tooling is expected to be run under Linux or WSL, instructions for setting up your build environment can be found in our [build-tools repo](https://github.com/bolderflight/build-tools). 

# Namespace
This library is within the namespace *bfs*.

# SbusRx
This class is used for receiving SBUS data from an SBUS capable receiver.

**SbusRx(HardwareSerial &ast;bus)** Creates an SbusRx object. A pointer to the Serial object corresponding to the serial port used is passed. The RX pin of the serial port will receive SBUS packets.

```C++
SbusRx sbus(&Serial1);
```

**void Begin()** Initializes SBUS communication.

```C++
sbus.Begin();
```

**(ESP-32 ONLY) void Begin(const int8_t rxpin, const int8_t txpin)** Initialized SBUS communication, given the Serial RX and TX pins.

**bool Read()** Parses SBUS packets, returns true on successfully receiving an SBUS packet.

```C++
if (sbus.Read()) {
   // Do something with the received data
}
```

**static constexpr int8_t NUM_CH()** A constant defining the number of SBUS channels (i.e. 16), useful for defining arrays to read the data into.

**(Non-AVR ONLY) std::array<uint16_t, 16> ch()** Returns the array of received channel data.

```C++
std::array<uint16_t, 16> sbus_data = sbus.ch();
```

**int8_t ch(int16_t * data, const int8_t len)** Copys the array of received channel data given a pointer to a destination, *data*, and length of the destination array *len*. Returns the number of channels copied on success or -1 on failure. Note that the maximum number of channels is the smaller of the *len* or *NUM_CH* (i.e. 16).

```C++
int16_t rx_ch[bfs::SbusRx::NUM_CH];
sbus.ch(rx_ch, bfs::SbusRx::NUM_CH);
```

**int16_t ch(const int8_t idx)** Returns received channel data given the channel index.

```C++
int16_t ch3_data = sbus.ch(3);
```

**bool ch17()** Returns the value of channel 17.

```C++
bool ch17 = sbus.ch17();
```

**bool ch18()** Returns the value of channel 18.

```C++
bool ch18 = sbus.ch18();
```

**bool lost_frame()** Returns true if a frame has been lost.

```C++
bool lost_frame = sbus.lost_frame();
```

**bool failsafe()** Returns true if the receiver has entered failsafe mode.

```C++
bool failsafe = sbus.failsafe();
```

# SbusTx
This class is used for transmitting SBUS data to SBUS capable servos.

**SbusTx(HardwareSerial &ast;bus)** Creates an SbusTx object. A pointer to the Serial object corresponding to the serial port used is passed. The TX pin of the serial port will transmit SBUS packets.

```C++
SbusTx sbus(&Serial1);
```

**void Begin()** Initializes SBUS communication.

```C++
sbus.Begin();
```

**(ESP-32 ONLY) void Begin(const int8_t rxpin, const int8_t txpin)** Initialized SBUS communication, given the Serial RX and TX pins.

**void Write()** Writes an SBUS packet. The packet is written immediately, you should regulate timing of sending packets to servos to maintain a frequency of approximately 100 Hz or 50 Hz, depending on the setup of the SBUS system.

```C++
sbus.Write();
```

**static constexpr int8_t NUM_CH()** A constant defining the number of SBUS channels (i.e. 16), useful for defining arrays to write the data from.

**void ch17(bool val)** Sets the value of channel 17 to be transmitted.

```C++
sbus.ch17(true);
```

**void ch18(bool val)** Sets the value of channel 18 to be transmitted.

```C++
sbus.ch18(true);
```

**void lost_frame(bool val)** Sets whether to transmit the lost frame flag.

```C++
sbus.lost_frame(true);
```

**void failsafe(bool val)** Sets whether to transmit the failsafe flag.

```C++
sbus.failsafe(true);
```

**(Non-AVR ONLY) void ch(const std::array<uint16_t, 16> &val)** Sets the channel data to be transmitted.

```C++
sbus.ch(sbus_tx_data);
```

**bool ch(const int8_t idx, const int16_t val)** Sets the channel data to be transmitted, given a channel index and corresponding value. Returns true on success and false on failure.

```C++
/* Set channel 3 to a value of 1200 */
sbus.ch(3, 1200);
```

**int8_t ch(int16_t const * const data, const int8_t len)** Sets the channel data to be transmitted given a pointer to an array of commands, *data*, and the array length, *len*. Returns the number of channels copied on success or -1 on failure. Note that the maximum number of channels is the smaller of the *len* or *NUM_CH* (i.e. 16).

```C++
int16_t cmd[bfs::SbusTx::NUM_CH()];
sbus.ch(cmd, bfs::SbusTx::NUM_CH());
```

**bool ch17()** Returns the value of channel 17 to be transmitted.

```C++
bool ch17 = sbus.ch17();
```

**bool ch18()** Returns the value of channel 18 to be transmitted.

```C++
bool ch18 = sbus.ch18();
```

**bool lost_frame()** Returns the lost frame flag value to be transmitted.

```C++
bool lost_frame = sbus.lost_frame();
```

**bool failsafe()** Returns the failsafe flag value to be transmitted.

```C++
bool failsafe = sbus.failsafe();
```

**(Non-AVR ONLY) std::array<uint16_t, 16> ch()** Returns the array of channel data to be transmitted.

```C++
std::array<uint16_t, 16> sbus_tx_data = sbus.ch();
```

**int16_t ch(const int8_t idx)** Returns the channel data to be transmitted, given an index.

```C++
/* Get the command for channel 3 */
int16_t data = sbus.ch(3);
```
