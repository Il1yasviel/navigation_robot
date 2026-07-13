# Host protocol v1

The same byte-stream protocol is carried over USB UART0 (115200/8N1) and
Wi-Fi TCP port 3333. All multi-byte fields are little-endian.

## Frame

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | `0xAA` |
| 1 | 1 | `0x55` |
| 2 | 1 | version `0x01` |
| 3 | 1 | message type |
| 4 | 1 | sequence |
| 5 | 1 | flags; bit 0 requests a success ACK |
| 6 | 2 | payload length, 0..128 |
| 8 | N | payload |
| 8+N | 2 | CRC16/CCITT-FALSE, low byte first |

CRC parameters are polynomial `0x1021`, initial value `0xFFFF`, no
reflection, no final XOR. CRC covers `version` through the payload. TCP is a
stream: receivers must support fragmented and concatenated frames.

## Commands and responses

| Type | Direction | Payload |
|---:|---|---|
| `0x01` | Host→MCU | HELLO, empty |
| `0x10` | Host→MCU | `id:u8, rpm:i16, accel:u8, brake:u8` |
| `0x11` | Host→MCU | legacy single joystick |
| `0x12` | Host→MCU | `id:u8, brake:u8` |
| `0x13` | Host→MCU | query by ID: `id:u8` |
| `0x14` | Host→MCU | single-device unique-ID query, empty |
| `0x15` | Host→MCU | `old_id:u8, new_id:u8, confirm:u16=0x4D36` |
| `0x16` | Host→MCU | `id:u8, mode:u8` |
| `0x17` | Host→MCU | `id:u8, current_raw:i16, accel:u8, brake:u8` |
| `0x18` | Host→MCU | `id:u8, position_raw:u16, accel:u8, brake:u8` |
| `0x19` | Host→MCU | single control keepalive: `id:u8` |
| `0x1A` | Host→MCU | `left_id:u8, right_id:u8, left_rpm:i16, right_rpm:i16, accel:u8, brake:u8` |
| `0x1B` | Host→MCU | dual keepalive: `left_id:u8, right_id:u8` |
| `0x1C` | Host→MCU | dual stop: `left_id:u8, right_id:u8, brake:u8` |
| `0x80` | MCU→Host | ACK: `request_type:u8, status:u8, detail:i16` |
| `0x90` | MCU→Host | legacy 30-byte selected-motor heartbeat |
| `0x91` | MCU→Host | reserved event |
| `0x92` | MCU→Host | 56-byte chassis telemetry |
| `0x93` | MCU→Host | 44-byte IMU telemetry |

Application speed is limited to ±125 RPM. `0x1A` RPM values use vehicle
logic: positive means the wheel propels the chassis forward. Firmware sends
left RPM unchanged and negates right RPM exactly once.

Brake off is `0x00`, brake on is `0xFF`. ID `0xC8` is reserved. Modes are
current `0x01`, speed `0x02`, position `0x03`.

ACK status: success 0, host CRC 1, bad length 2, range 3, busy 4, motor timeout
5, motor CRC 6, precondition 7, unsupported 8, I/O 9. Failed recognized
requests always return an ACK. Success returns an ACK only when frame flag bit
0 is set. Keepalives normally do not request success ACKs.

USB and TCP may both receive telemetry. The first successful motion command
owns control. Motion from the other transport returns busy. Queries do not
claim control. Dual stop is accepted from either transport. Disconnect or a
300 ms keepalive timeout stops the motors and releases ownership.

## Chassis telemetry `0x92`

Header:

| Offset | Type | Meaning |
|---:|---|---|
| 0 | `u32` | MCU uptime ms |
| 4 | `u8` | owner: 0 none, 1 UART, 2 TCP |
| 5 | `u8` | bit0 active, bit1 estop, bit2 Wi-Fi IP, bit3 left valid, bit4 right valid |
| 6 | `u16` | watchdog stop count |

Left record starts at offset 8 and right record at offset 32. Each record is:

| Relative offset | Type | Meaning |
|---:|---|---|
| 0 | `u8` | motor ID |
| 1 | `u8` | mode |
| 2 | `u8` | state: offline/idle/running/fault/estop = 0..4 |
| 3 | `u8` | M0601 fault bits |
| 4 | `i16` | logical target RPM |
| 6 | `i16` | logical actual RPM |
| 8 | `i16` | motor current raw |
| 10 | `u16` | drive feedback position raw |
| 12 | `u8` | query feedback position raw |
| 13 | `u8` | reserved zero |
| 14 | `u16` | feedback age ms, saturated |
| 16 | `u32` | valid feedback count |
| 20 | `u16` | motor CRC errors |
| 22 | `u16` | motor timeouts |

## IMU telemetry `0x93`

| Offset | Type | Meaning |
|---:|---|---|
| 0 | `u64` | monotonic MCU timestamp, microseconds |
| 8 | `u8` | bit0 online, bit1 gyro bias calibrated, bit2 sample valid |
| 9 | 3 bytes | reserved zero |
| 12 | `3*f32` | acceleration X/Y/Z, m/s² |
| 24 | `3*f32` | angular velocity X/Y/Z, rad/s |
| 36 | `u32` | sample count |
| 40 | `u16` | read error count |
| 42 | `u16` | initialization error count |

Axes are X forward, Y left, Z up. No orientation quaternion is supplied.

## Golden frames

```text
HELLO seq=0x2A
AA 55 01 01 2A 00 00 00 04 BE

dual +25/+25 RPM seq=1, ACK requested
AA 55 01 1A 01 01 08 00 01 02 19 00 19 00 00 00 D2 49

dual keepalive seq=2, no ACK requested
AA 55 01 1B 02 00 02 00 01 02 E2 7F

dual braking stop seq=3, ACK requested
AA 55 01 1C 03 01 03 00 01 02 FF 00 E3
```

Conversions:

```text
current_mA = current_raw * 8000 / 32767
position_deg = drive_position_raw * 360 / 32767
query_position_deg = query_position_u8 * 360 / 256
```
