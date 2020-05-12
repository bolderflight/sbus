# sbus
This library communicates with SBUS receivers and servos. 
   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)
   * [Contributing guide](CONTRIBUTING.md)

# Description
SBUS is a bus protocol for receivers to send commands to servos. Unlike PWM, SBUS uses a bus architecture where a single serial line can be connected with up to 16 servos with each receiving a unique command. SBUS capable servos are required; each can be programmed with a unique address using an SBUS servo programmer.

SBUS can be used to receive pilot commands from an SBUS capable receiver to use as input to a flight control system. SBUS outputs can be sent to SBUS capable capable servos. Advantages compared to PWM include a reduction in vehicle wiring and ease of parsing SBUS packets.

The SBUS protocol uses an inverted serial logic with a baud rate of 100000, 8 data bits, even parity, and 2 stop bits. The SBUS packet is 25 bytes long consisting of:
   * Byte[0]: SBUS header, 0x0F
   * Byte[1 -22]: 16 servo channels, 11 bits each
   * Byte[23]
      * Bit 5: frame lost (0x20)
      * Bit 4: failsafe activated (0x10)
   * Byte[24]: SBUS footer

Note that lost frame is indicated when a frame is lost between the transmitter and receiver. Failsafe activation typically requires that several frames are lost in a row and indicates that the receiver has moved into failsafe mode. Packets are sent approximately every 10 ms. FrSky receivers will output a range of 172 - 1811 with channels set to a range of -100% to +100%. Using extended limits of -150% to +150% outputs a range of 0 to 2047, which is the maximum range acheivable with 11 bits of data.

## Installation
CMake is used to build this library, which is exported as a library target called *sbus*. The header is added as:

```
#include "sbus/sbus.h"
```
Note that you'll need CMake version 3.13 or above; it is recommended to build and install CMake from source, directions are located in the [CMake GitLab repository](https://github.com/Kitware/CMake).

The library can be also be compiled stand-alone using the CMake idiom of creating a *build* directory and then, from within that directory issuing:

```
cmake .. -DMCU=MK66FX1M0
make
```

This will build the library, an example executable called *sbus_example*, and executables for testing using the Google Test framework. The example executable source file is located at *examples/sbus_example.cc*. This code is built and tested on an AMD64 system running Linux and is likely to build on AMD64 systems running the Windows Subsystem for Linux (WSL). The [arm-none-eabi](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads) toolchain must be installed in your Linux environment.

Notice that the *cmake* command includes a define specifying the microcontroller the code is being compiled for. This is required to correctly configure the code, CPU frequency, and compile/linker options. The available MCUs are:
   * MK20DX128
   * MK20DX256
   * MK64FX512
   * MK66FX1M0
   * MKL26Z64

These are known to work with the same packages used in Teensy products. Also switching the MK66FX1M0 or MK64FX512 from BGA to LQFP packages is known to work well. Swapping packages of other chips is probably fine, as long as it's only a package change.

The *sbus_example* target creates an executable for communicating with sbus receivers and servos. This target also has a *_hex* for creating the hex file and a *_upload* to upload the software to the microcontroller. 

Testing is done using a lightweight Remote Command Protocol (RCP) between a Linux *master* and the microcontroller *slave*. The *slave* registers tests, which the *master* can call and receive a boolean status on the test results. A definition file utility, [mcu_hil_defs](https://gitlab.com/bolderflight/utils/mcu_hil_defs) defines the pins and ports for each sensor and communication method. A seperate utility, *mcu_reset*, cycles power to the microcontroller and sensors to provide a clean environment; it should be used before each test. See *tests/master.cc* and *tests/slave.cc* for how the tests are defined.

# Namespaces
The Sbus object for receiving SBUS data from a receiver is within the namespace *sensors*. The Sbus object for sending SBUS commands to servos is within the namespace *actuators*.

# Methods

## Sensors

**Sbus(HardwareSerial &ast;bus)** Creates an Sbus object. A pointer to the Serial object corresponding to the serial port used is passed. The RX pin of the serial port will receive SBUS packets.

```C++
sensors::Sbus sbus(&Serial1);
```

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

**std::array<uint16_t, 16> rx_channels()** Returns the array of received channel data.

```C++
std::array<uint16_t, 16> sbus_data = sbus.rx_channels();
```

**bool lost_frame()** Returns true if a frame has been lost.

```C++
bool lost_frame = sbus.lost_frame();
```

**bool failsafe()** Returns true if the receiver has entered failsafe mode.

```C++
bool failsafe = sbus.failsafe();
```

## Actuators

**Sbus(HardwareSerial &ast;bus)** Creates an Sbus object. A pointer to the Serial object corresponding to the serial port used is passed. The TX pin of the serial port will transmit SBUS packets.

```C++
actuators::Sbus sbus(&Serial1);
```

**void Begin()** Initializes SBUS communication.

```C++
sbus.Begin();
```

**void Write()** Writes an SBUS packet. The packet is written immediately, you should regulate timing of sending packets to servos to maintain a frequency of approximately 100 Hz.

```C++
sbus.Write();
```

**void tx_channels(const std::array<uint16_t, 16> &val)** Sets the channel data to be transmitted.

```C++
sbus.tx_channels(sbus_tx_data);
```

**std::array<uint16_t, 16> tx_channels()** Returns the array of channel data to be transmitted.

```C++
std::array<uint16_t, 16> sbus_tx_data = sbus.tx_channels();
```
