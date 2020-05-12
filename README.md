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

## Methods

**Sbus(HardwareSerial &ast;bus)** Creates an Sbus object. A pointer to the Serial object corresponding to the serial port used is passed. The RX pin of the serial port will receive SBUS packets and the TX pin will send SBUS packets.

```C++
Sbus sbus(&Serial1);
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

**void Write()** Writes an SBUS packet. The packet is written immediately, you should regulate timing of sending packets to servos to maintain a frequency of approximately 100 Hz.

```C++
sbus.Write();
```

**std::array<uint16_t, 16> rx_channels()** Returns the array of received channel data.

```C++
std::array<uint16_t, 16> sbus_data = sbus.rx_channels();
```

**std::array<uint16_t, 16> tx_channels()** Returns the array of channel data to be transmitted.

```C++
std::array<uint16_t, 16> sbus_tx_data = sbus.tx_channels();
```

**void tx_channels(const std::array<uint16_t, 16> &val)** Sets the channel data to be transmitted.

```C++
sbus.tx_channels(sbus_tx_data);
```

**bool lost_frame()** Returns true if a frame has been lost.

```C++
bool lost_frame = sbus.lost_frame();
```

**bool failsafe()** Returns true if the receiver has entered failsafe mode.

```C++
bool failsafe = sbus.failsafe();
```

**void End()** Frees the serial port used by the Sbus object.

```C++
sbus.End();
```
