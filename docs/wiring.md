# Wiring and first-wheel bring-up

## FTDI host UART

Use the development-board USB connector backed by the FTDI USB-UART bridge:

```text
FTDI TX  -> ESP32-S3 GPIO44 (UART0 RX)
FTDI RX  <- ESP32-S3 GPIO43 (UART0 TX)
USB GND  -> ESP32-S3 GND
```

The same COM port is used for flashing and the application, so only one of
`idf.py`, a terminal, or `motor_test_gui.py` may have it open. The GUI releases
DTR and RTS before opening the port to avoid the FTDI auto-program circuit
holding the ESP32-S3 in reset.

## Isolated automatic-direction TTL-to-RS485 module

```text
ESP32-S3 GPIO17 (UART1 TX)  -> module RX
ESP32-S3 GPIO18 (UART1 RX)  <- module TX
ESP32 GND                   -> module TTL-side GND
Module A+                   -> M0601 A+
Module B+                   -> M0601 B+
```

Power the module according to its own terminal marking and manual. The isolated
module switches transmit and receive direction internally, so no RTS/DE/~RE
connection is used. Its onboard 120-ohm termination is enabled by default. Keep
motor power separate from the board supply.

## Safe first test

1. Lift and secure the wheel so it cannot move the robot.
2. Connect exactly one motor to RS485 when querying or changing ID.
3. Flash the firmware, then close the flashing process.
4. Start the GUI and connect the FTDI COM port; wait for HELLO handshake success.
5. For a known ID, run **Query target ID status**. Enable single-motor
   maintenance before **Query unique ID** or changing ID.
6. Set the GUI limit to 20 RPM and briefly move the joystick forward.
7. Release, verify zero RPM, then briefly reverse.
8. While running slowly, unplug USB and verify braking within 300 ms.

If a query fails, check that GPIO17 crosses to module RX and GPIO18 crosses to
module TX, then check A+/B+ polarity, 115200 8N1 configuration, and motor power.

## BMI088 SPI2

| BMI088 signal | ESP32-S3 |
|---|---:|
| SCK | GPIO12 |
| MOSI | GPIO11 |
| MISO | GPIO13 |
| Accelerometer CS | GPIO47 |
| Gyroscope CS | GPIO21 |

The installed sensor axes are the robot axes: X forward, Y left and Z up.
Keep the robot stationary for about 2.5 seconds after boot while gyro bias is
calibrated.

## Wi-Fi

ESP32-S3 supports 2.4 GHz Wi-Fi only. Configure STA credentials with
`idf.py menuconfig`; credentials are stored only in the ignored local
`sdkconfig`. The binary host protocol listens on TCP port 3333 and advertises
`navigation-robot.local` through mDNS.
