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

## RS485 transceiver

```text
ESP32-S3 GPIO17 (UART1 TX)  -> DI
ESP32-S3 GPIO18 (UART1 RX)  <- RO
ESP32-S3 GPIO16 (UART1 RTS) -> DE and ~RE tied together
Transceiver A/B             -> M0601 A/B
ESP32 GND                   -> transceiver GND -> motor signal GND
```

Use a 3.3 V logic-compatible transceiver such as MAX3485. Keep motor power
separate from the board supply. Add 120-ohm termination at the physical bus
ends for longer cables; do not add termination at every motor.

## Safe first test

1. Lift and secure the wheel so it cannot move the robot.
2. Connect exactly one motor to RS485 when querying or changing ID.
3. Flash the firmware, then close the flashing process.
4. Start the GUI and connect the FTDI COM port; wait for HELLO handshake success.
5. Run **Query unique ID**. Do not change ID until it succeeds.
6. Set the GUI limit to 20 RPM and briefly move the joystick forward.
7. Release, verify zero RPM, then briefly reverse.
8. While running slowly, unplug USB and verify braking within 300 ms.

If Query ID fails, first swap RS485 A/B, then check common ground, GPIO16
DE/~RE wiring, 115200 8N1 configuration, and motor power.
