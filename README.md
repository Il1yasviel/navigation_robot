# ESP32-S3 Navigation Robot Lower Controller

ESP32-S3N16R8 下位机工程。目前实现第一阶段：通过板载 FTDI USB-UART
连接 Python 上位机，对一台 M0601 RS485 电机进行速度、停止、ID 修改和
反馈心跳测试。

## Hardware defaults

| Function | ESP32-S3 pin |
|---|---:|
| FTDI UART0 TX | GPIO43 |
| FTDI UART0 RX | GPIO44 |
| RS485 UART1 TX / module RX | GPIO17 |
| RS485 UART1 RX / module TX | GPIO18 |
| Reserved ultrasonic TRIG | GPIO5 |
| Reserved ultrasonic ECHO | GPIO6 |

Use the isolated automatic-direction TTL-to-RS485 module: GPIO17 TX connects to
module RX and GPIO18 RX connects to module TX. No RTS/DE/~RE line is required.
Do not power the motor from the development board.

## ESP-IDF firmware

The project is pinned to ESP-IDF 5.5.4 and the `esp32s3` target. From an
ESP-IDF 5.5.4 PowerShell:

```powershell
idf.py set-target esp32s3
idf.py build
idf.py -p COM_PORT flash
```

The FTDI UART0 channel is reserved for binary application data at runtime. The
ESP-IDF console is disabled, so `idf.py monitor` is not part of the normal
runtime workflow. Close the GUI before flashing and close `idf.py` before
opening the GUI. ROM boot text may appear once after reset; the GUI ignores it
and waits for a binary HELLO acknowledgement before enabling controls.

## Host GUI

```powershell
python -m pip install -r requirements-host.txt
python motor_test_gui.py
```

Test with the wheel lifted from the floor. The initial GUI limit is 30 RPM and
the firmware rejects commands beyond 60 RPM. Releasing the joystick sends a
normal zero-speed command; Emergency Stop and the 300 ms command watchdog use
braking.

The fallback motor ID is `1`. Normal query and control address the target ID, so
multiple uniquely-addressed motors can share the bus. **Query unique ID** and ID
changes remain disabled until single-motor maintenance is explicitly enabled.
The GUI supports speed, current, and position modes; current is capped at
±1000mA and position is entered as 0..360 degrees. Unchanged motion targets are
not repeatedly placed on the RS485 bus: the host uses a lightweight 100 ms
control keepalive while status polling and telemetry remain at 10 Hz.

## Tests

```powershell
python -m unittest tests.host.test_protocol -v
```

Pure C protocol tests can be built independently (CMake is included with an
ESP-IDF tools installation):

```powershell
cmake -S tests/native -B build-native
cmake --build build-native
ctest --test-dir build-native --output-on-failure
```

The custom host wire protocol is documented in
[docs/host_protocol_v1.md](docs/host_protocol_v1.md). Wiring and hardware
bring-up are documented in [docs/wiring.md](docs/wiring.md).
