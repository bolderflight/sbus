[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

![Bolder Flight Systems Logo](img/logo-words_75.png) &nbsp; &nbsp; ![Arduino Logo](img/arduino_logo_75.png)

# Sbus
This library communicates with SBUS receivers and servos and is compatible with Arduino and CMake build systems.
   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)
   * [Contributing guide](CONTRIBUTING.md)

# Description
SBUS is a bus protocol for receivers to send commands to servos. Unlike PWM, SBUS uses a bus architecture where a single serial line can be connected with up to 16 servos with each receiving a unique command.

The SBUS protocol uses an inverted serial logic with a baud rate of 100000, 8 data bits, even parity, and 2 stop bits. The SBUS packet is 25 bytes long consisting of:
   * Byte[0]: SBUS header, 0x0F
   * Byte[1 -22]: 16 servo channels, 11 bits each
   * Byte[23]
      * Bit 0: channel 17 (0x01)
      * Bit 1: channel 18 (0x02)
      * Bit 2: frame lost (0x04)
      * Bit 3: failsafe activated (0x08)
   * Byte[24]: SBUS footer

Note that lost frame is indicated when a frame is lost between the transmitter and receiver. Failsafe activation typically requires that many frames are lost in a row and indicates that the receiver has moved into failsafe mode. Packets are sent approximately every 10 ms or 20 ms, depending on the system configuration.

A variation on SBUS called "Fast SBUS" has started to be used. This uses a baudrate of 200000 and a quicker update rate.

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

For all other microcontrollers, you **must** use a serial inverter. If you've modified this library to work with other microcontrollers, please submit a pull request.

# Installation

## Arduino
Simply clone or download and extract the zipped library into your Arduino/libraries folder. The library is added as:

```C++
#include "sbus.h"
```

An example is located in *examples/arduino/sbus_example/sbus_example.ino*. This library is tested with Teensy 3.x, 4.x, and LC devices and should work with other Arduino devices.

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
   * IMXRT1062_MMOD

These are known to work with the same packages used in Teensy products. Also switching packages is known to work well, as long as it's only a package change.

The *sbus_example* target creates an executable for communicating with sbus receivers and servos. This target also has a *_hex* for creating the hex file and an *_upload* for using the [Teensy CLI Uploader](https://www.pjrc.com/teensy/loader_cli.html) to flash the Teensy. Instructions for setting up your build environment can be found in our [build-tools repo](https://github.com/bolderflight/build-tools). 

# Namespace
This library is within the namespace *bfs*.

# SbusData
This struct defines SBUS data that can be read and returned by the *SbusRx* object or set and sent by the *SbusTx* object.

## Data Members

**bool lost_frame** Whether a frame has been lost.

**bool failsafe** Whether the receiver has entered failsafe mode or to command servos to enter failsafe mode.

**bool ch17, ch18** State of channel 17 and channel 18.

**static constexpr int8_t NUM_CH = 16** The number of SBUS channels.

**int16_t ch[NUM_CH]** An array of SBUS channel data.

# SbusRx
This class is used for receiving SBUS data from an SBUS capable receiver.

**SbusRx(HardwareSerial &ast;bus)** Creates an *SbusRx* object. A pointer to the *Serial* object corresponding to the serial port used is passed. The RX pin of the serial port will receive SBUS packets.

```C++
bfs::SbusRx sbus(&Serial1);
```

**SbusRx(HardwareSerial &ast;bus, const bool inv)** Creates an *SbusRx* object. A pointer to the *Serial* object corresponding to the serial port used is passed along with a second parameter, *inv*, which sets whether inverted serial is used. If *inv* is true, the signal is the standard inverted SBUS, otherwise it is non-inverted SBUS.

```C++
bfs::SbusRx sbus(&Serial1, false);
```

**SbusRx(HardwareSerial &ast;bus, const bool inv, const bool fast)** Same as the constructor above, but enables selecting the fast SBUS baudrate (200000) if *fast* is true.

**(ESP32 ONLY) SbusRx(HardwareSerial &ast;bus, const int8_t rxpin, const int8_t txpin, const bool inv)** Creates an *SbusRx* object. A pointer to the *Serial* object corresponding to the serial port used is passed along with the RX pin number (*rxpin*), TX pin number (*txpin*), and whether inverted serial is used (*inv*). If *inv* is true, the signal is the standard inverted SBUS, otherwise it is non-inverted SBUS.

**(ESP32 ONLY) SbusRx(HardwareSerial &ast;bus, const int8_t rxpin, const int8_t txpin, const bool inv, const bool fast)** Same as the constructor above, but enables selecting the fast SBUS baudrate (200000) if *fast* is true.

**void Begin()** Initializes SBUS communication.

```C++
sbus.Begin();
```

**bool Read()** Parses SBUS packets, returns true on successfully receiving an SBUS packet.

```C++
if (sbus.Read()) {
  // Do something with the received data
}
```

**SbusData data()** Returns the *SbusData* structure, populated with data from the last received packet.

```C++
if (sbus.Read()) {
  bfs::SbusData data = sbus.data();
}
```

# SbusTx
This class is used for transmitting SBUS data to SBUS capable servos.

**SbusTx(HardwareSerial &ast;bus)** Creates an *SbusTx* object. A pointer to the *Serial* object corresponding to the serial port used is passed. The TX pin of the serial port will receive SBUS packets.

```C++
bfs::SbusTx sbus(&Serial1);
```

**SbusTx(HardwareSerial &ast;bus, const bool inv)** Creates an *SbusTx* object. A pointer to the *Serial* object corresponding to the serial port used is passed along with a second parameter, *inv*, which sets whether inverted serial is used. If *inv* is true, the signal is the standard inverted SBUS, otherwise it is non-inverted SBUS.

```C++
bfs::SbusTx sbus(&Serial1, false);
```

**(ESP32 ONLY) SbusTx(HardwareSerial &ast;bus, const int8_t rxpin, const int8_t txpin, const bool inv)** Creates an *SbusTx* object. A pointer to the *Serial* object corresponding to the serial port used is passed along with the RX pin number (*rxpin*), TX pin number (*txpin*), and whether inverted serial is used (*inv*). If *inv* is true, the signal is the standard inverted SBUS, otherwise it is non-inverted SBUS.

**void Begin()** Initializes SBUS communication.

```C++
sbus.Begin();
```

**void Write()** Writes an SBUS packet. The packet is written immediately, you should regulate timing of sending packets to servos to maintain a frequency of approximately 100 Hz or 50 Hz, depending on the setup of the SBUS system.

```C++
sbus.Write();
```

**void data(const SbusData &data)** Sets the SBUS data, which will be transmitted on the next *Write* method.

```C++
bfs::SbusData data;
data.ch[0] = 900;
sbus.data(data);
```

**SbusData data()** Returns the SBUS data buffered in the *SbusTx* object, which will be transmitted on the next *Write* method.

```C++
bfs::SbusData data = sbus.data();
```
