# Host protocol v1

This protocol is transported over the board FTDI bridge and ESP32-S3 UART0. All
multi-byte values are little-endian.

## Frame

```text
Offset  Size  Field
0       1     0xAA
1       1     0x55
2       1     version = 0x01
3       1     message type
4       1     sequence
5       1     flags (bit 0 = ACK requested)
6       2     payload length, 0..128
8       N     payload
8+N     2     CRC16/CCITT-FALSE, low byte first
```

CRC parameters are polynomial `0x1021`, initial value `0xFFFF`, no reflection,
and no final XOR. The CRC covers bytes from `version` through the end of the
payload. There is no tail marker. A receiver resynchronizes by scanning for
`AA 55`, validating length, then validating CRC.

Golden PING frame, sequence `0x2A`:

```text
AA 55 01 01 2A 00 00 00 04 BE
```

## Commands

| Type | Direction | Payload |
|---:|---|---|
| `0x01` | Host to MCU | HELLO, empty |
| `0x10` | Host to MCU | `id:u8, rpm:i16, accel:u8, brake:u8` |
| `0x11` | Host to MCU | `id:u8, x_permille:i16, y_permille:i16, max_rpm:u16, deadman:u8` |
| `0x12` | Host to MCU | `id:u8, brake:u8` |
| `0x13` | Host to MCU | `id:u8` |
| `0x14` | Host to MCU | Query unique ID, empty |
| `0x15` | Host to MCU | `expected_old:u8, new_id:u8, confirm:u16` |
| `0x16` | Host to MCU | `id:u8, mode:u8` |
| `0x17` | Host to MCU | `id:u8, current_raw:i16, accel:u8, brake:u8` |
| `0x18` | Host to MCU | `id:u8, position_raw:u16, accel:u8, brake:u8` |
| `0x19` | Host to MCU | Control keepalive: `id:u8`; no RS485 transaction |
| `0x80` | MCU to Host | `request_type:u8, status:u8, detail:i16` |
| `0x90` | MCU to Host | 30-byte heartbeat below |
| `0x91` | MCU to Host | Reserved event: `code:u16, severity:u8, argument:u32` |

The ID-change confirmation value is `0x4D36`. Brake off is `0x00`, brake on is
`0xFF`. ID `0xC8` is reserved by M0601 for unique-ID query and cannot be used as
a normal motor ID.

ACK status values: `0` success, `1` host CRC, `2` bad length, `3` range,
`4` busy, `5` motor timeout, `6` motor CRC, `7` failed precondition,
`8` unsupported, and `9` I/O error. ACK uses the request sequence. For unique
ID query and successful ID change, `detail` contains the confirmed motor ID.
Normal control and query commands address the motor ID explicitly. Unique-ID
query and ID change are maintenance operations for a bus containing exactly one
motor; the GUI keeps these controls disabled by default.

Successful requests produce an ACK only when flag bit 0 (`ACK requested`) is
set. Recognized requests that fail validation or execution always produce an
ACK. Motion commands request an ACK; after a
successful nonzero motion command the GUI sends `0x19` every 100 ms without
requesting successful ACKs. A keepalive is accepted only for the confirmed,
actively controlled motor and does not repeat the M0601 drive command.

## Heartbeat payload

```text
0   u32  MCU uptime milliseconds
4   u8   motor ID
5   u8   M0601 mode
6   u8   state: 0 offline, 1 idle, 2 running, 3 fault, 4 emergency stop
7   u8   M0601 fault bits
8   i16  target value: RPM, current raw, or position raw according to mode
10  i16  actual RPM
12  i16  current raw
14  u16  drive feedback position raw
16  u8   query feedback position raw
17  u8   reserved, always zero
18  u16  feedback age milliseconds, saturated at 65535
20  u32  valid feedback count
24  u16  motor CRC error count
26  u16  motor timeout count
28  u16  watchdog stop count
```

Conversions used by the GUI:

```text
current_mA = current_raw * 8000 / 32767
position_deg = drive_position_raw * 360 / 32767
query_position_deg = query_position_u8 * 360 / 256
```
