"""线协议常量、CRC16、帧编码与帧解析。"""
from __future__ import annotations

import struct
from dataclasses import dataclass

SOF = b"\xAA\x55"
VERSION = 1
MAX_PAYLOAD = 128
FLAG_ACK_REQUIRED = 0x01

MSG_HELLO = 0x01
MSG_SET_SINGLE_RPM = 0x10
MSG_JOYSTICK = 0x11
MSG_STOP = 0x12
MSG_QUERY_MOTOR = 0x13
MSG_QUERY_UNIQUE_ID = 0x14
MSG_SET_ID = 0x15
MSG_SET_MODE = 0x16
MSG_SET_CURRENT = 0x17
MSG_SET_POSITION = 0x18
MSG_CONTROL_KEEPALIVE = 0x19
MSG_SET_DUAL_RPM = 0x1A
MSG_DUAL_KEEPALIVE = 0x1B
MSG_STOP_DUAL = 0x1C
MSG_ACK = 0x80
MSG_HEARTBEAT = 0x90
MSG_CHASSIS_TELEMETRY = 0x92
MSG_IMU_TELEMETRY = 0x93


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(msg_type: int, sequence: int, flags: int = 0,
                 payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload exceeds 128 bytes")
    body = struct.pack("<BBBBH", VERSION, msg_type & 0xFF, sequence & 0xFF,
                       flags & 0xFF, len(payload)) + payload
    return SOF + body + struct.pack("<H", crc16_ccitt_false(body))


@dataclass(frozen=True)
class HostFrame:
    msg_type: int
    sequence: int
    flags: int
    payload: bytes


class FrameParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.valid_frames = 0
        self.crc_errors = 0
        self.length_errors = 0

    def feed(self, data: bytes) -> list[HostFrame]:
        self.buffer.extend(data)
        frames: list[HostFrame] = []
        while True:
            position = self.buffer.find(SOF)
            if position < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == SOF[:1] else b""
                break
            if position:
                del self.buffer[:position]
            if len(self.buffer) < 8:
                break
            if self.buffer[2] != VERSION:
                del self.buffer[0]
                continue
            payload_length = struct.unpack_from("<H", self.buffer, 6)[0]
            if payload_length > MAX_PAYLOAD:
                self.length_errors += 1
                del self.buffer[0]
                continue
            total = 10 + payload_length
            if len(self.buffer) < total:
                break
            received_crc = struct.unpack_from("<H", self.buffer, 8 + payload_length)[0]
            expected_crc = crc16_ccitt_false(bytes(self.buffer[2:8 + payload_length]))
            if received_crc != expected_crc:
                self.crc_errors += 1
                del self.buffer[0]
                continue
            frames.append(HostFrame(self.buffer[3], self.buffer[4], self.buffer[5],
                                    bytes(self.buffer[8:8 + payload_length])))
            self.valid_frames += 1
            del self.buffer[:total]
        return frames
