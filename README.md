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
This driver conforms to the [Inceptor interface](https://github.com/bolderflight/inceptor); please refer to those documents for information on the *InceptorConfig* and *InceptorData* structs.

**bool Init(const InceptorConfig &ref)** Initializes communication with the SBUS receiver. Returns true on successfully initializing communication with the receiver. Note that often receivers will not transmit data until they've connected with an SBUS transmitter, so it might be necessary to turn on the transmitter before this method is called.

```C++
/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx;
/* RX Config */
bfs::InceptorConfig rx_config = {
   .hw = &Serial2,
   .throttle = {
   .ch = 0,
   .num_coef = 2,
   .poly_coef = {0.0012203, -1.2098841}
   }
};
if (!sbus_rx.Init(rx_config)) {
   Serial.println("Unable to establish communication with SBUS receiver");
   while (1) {}
}
```

**bool Read(InceptorData * const data)** Reads data from the SBUS receiver adn passes the data to the *InceptorData* struct. Returns true on successfully receiving new data.

```C++
/* Sbus RX data */
bfs::InceptorData data;
if (sbus_rx.Read(&data)) {

}
```

# SbusTx
This driver conforms to the [Effector interface](https://github.com/bolderflight/effector); please refer to those documents for information on the *EffectorConfig* and struct.

**bool Init(const EffectorConfig &ref)** Initializes communication on the SBUS. Returns true on success.

```C++
/* SBUS object, writing SBUS */
bfs::SbusTx<16> sbus_tx;
/* TX Config */
bfs::EffectorConfig<16> tx_config = {
   .hw = &Serial2,
   .effectors = {
   {
      .type = bfs::SERVO,
      .ch = 1,
      .min = -20,
      .max = 20,
      .failsafe = 0,
      .num_coef = 2,
      .poly_coef = {819.50, 991.50}
   }
   }
};
if (!sbus_tx.Init(tx_config)) {
   Serial.println("Unable to init SBUS transmitter");
   while (1) {}
}
```

**void Cmd(std::span<float> cmds)** Issues angle commands, which are converted to SBUS commands and stored.

```C++
cmds[0] = data.throttle;
sbus_tx.Cmd(cmds);
```

**void Write()** Sends the stored SBUS commands to the servos. This method should be called every 10ms to 20ms.

```C++
sbus_tx.Write();
```

**void EnableMotors()** Enables motors to output commands.

```C++
sbus_tx.EnableMotors();
```

**void DisableMotors()** Disables motors from outputting commands, the failsafe command is sent instead.

**void EnableServos()** Enables servos to output commands.

```C++
sbus_tx.EnableServos();
```

**void DisableServos()** Disables servos from outputting commands, the failsafe command is sent instead.
