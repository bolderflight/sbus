# sbus-arduino
This library communicates with SBUS receivers and servos and is built for use with the Arduino IDE.
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

Note that lost frame is indicated when a frame is lost between the transmitter and receiver. Failsafe activation typically requires that several frames are lost in a row and indicates that the receiver has moved into failsafe mode. Packets are sent approximately every 10 ms or 20 ms. 

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

For all other microcontrollers, you **must** use a serial inverter.

# Installation
Simply clone or download and extract the zipped library into your Arduino/libraries folder. The library is added as:

```C++
#include "sbus.h"
```

**AVR Boards:** you will also need to clone or download the [Embedded Template Library](https://github.com/ETLCPP/etl-arduino) into your Arduino/libraries folder. This provides *etl::array* functionality as an alternative to *std::array*.

# Methods

## Receiving SBUS

**SbusRx(HardwareSerial &ast;bus)** Creates an SbusRx object. A pointer to the Serial object corresponding to the serial port used is passed. The RX pin of the serial port will receive SBUS packets.

```C++
SbusRx sbus(&Serial1);
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

## Writing SBUS

**SbusTx(HardwareSerial &ast;bus)** Creates an SbusTx object. A pointer to the Serial object corresponding to the serial port used is passed. The TX pin of the serial port will transmit SBUS packets.

```C++
SbusTx sbus(&Serial1);
```

**void Begin()** Initializes SBUS communication.

```C++
sbus.Begin();
```

**void Write()** Writes an SBUS packet. The packet is written immediately, you should regulate timing of sending packets to servos to maintain a frequency of approximately 100 Hz or 50 Hz, depending on the setup of the SBUS system.

```C++
sbus.Write();
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

**void tx_channels(const std::array<uint16_t, 16> &val)** Sets the channel data to be transmitted.

```C++
sbus.tx_channels(sbus_tx_data);
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

**std::array<uint16_t, 16> tx_channels()** Returns the array of channel data to be transmitted.

```C++
std::array<uint16_t, 16> sbus_tx_data = sbus.tx_channels();
```
