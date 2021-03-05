# sbus
This library communicates with SBUS receivers and servos. 
   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)
   * [Contributing guide](CONTRIBUTING.md)

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

Note that lost frame is indicated when a frame is lost between the transmitter and receiver. Failsafe activation typically requires that several frames are lost in a row and indicates that the receiver has moved into failsafe mode. Packets are sent approximately every 10 ms or 20 ms. 

**Note on CH17 and CH18:** Channel 17 and channel 18 are digital on/off channels. These are not universally available on all SBUS receivers and servos.

FrSky receivers will output a range of 172 - 1811 with channels set to a range of -100% to +100%. Using extended limits of -150% to +150% outputs a range of 0 to 2047, which is the maximum range acheivable with 11 bits of data.

Because SBUS is a digital bus format, it is an excellent means of receiving pilot commands from a transmitter and an SBUS capable receiver. If SBUS servos are used in the aircraft, SBUS is also an excellent means of sending actuator commands - servo commands can often be sent with lower latency and, by only using a single pin to command up to 16 servos, additional microcontroller pins are freed for other uses.

## Installation
CMake is used to build this library, which is exported as a library target called *sbus*. The header is added as:

```
#include "sbus/sbus.h"
```

The library can be also be compiled stand-alone using the CMake idiom of creating a *build* directory and then, from within that directory issuing:

```
cmake .. -DMCU=MK66FX1M0
make
```

This will build the library and example executable called *sbus_example*. The example executable source file is located at *examples/sbus_example.cc*. Notice that the *cmake* command includes a define specifying the microcontroller the code is being compiled for. This is required to correctly configure the code, CPU frequency, and compile/linker options. The available MCUs are:
   * MK20DX128
   * MK20DX256
   * MK64FX512
   * MK66FX1M0
   * MKL26Z64
   * IMXRT1062_T40
   * IMXRT1062_T41

These are known to work with the same packages used in Teensy products. Also switching packages is known to work well, as long as it's only a package change.

The *sbus_example* target creates an executable for communicating with sbus receivers and servos. This target also has a *_hex* for creating the hex file to upload to the microcontroller. 

# Namespace
This library is within the namespace *bfs*

# SbusRx

**SbusRx(HardwareSerial &ast;bus)** Creates an *SbusRx* object. A pointer to the Serial object corresponding to the serial port used is passed. The RX pin of the serial port will receive SBUS packets.

```C++
bfs::SbusRx sbus(&Serial1);
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

## SbusTx

**SbusTx(HardwareSerial &ast;bus)** Creates an *SbusTx* object. A pointer to the Serial object corresponding to the serial port used is passed. The TX pin of the serial port will transmit SBUS packets.

```C++
bfs::SbusTx sbus(&Serial1);
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

**std::array<uint16_t, 16> tx_channels()** Returns the array of channel data to be transmitted.

```C++
std::array<uint16_t, 16> sbus_tx_data = sbus.tx_channels();
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
