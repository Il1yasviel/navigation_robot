#!/usr/bin/env python3
"""Pseudo-terminal emulator for the ESP32-S3 host protocol (test use only)."""

import argparse
import os
import pty
import select
import struct
import sys
import time


def crc16(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(msg_type, sequence, payload=b'', flags=0):
    body = struct.pack('<BBBBH', 1, msg_type, sequence, flags, len(payload)) + payload
    return b'\xAA\x55' + body + struct.pack('<H', crc16(body))


def try_write(fd, data):
    """Drop simulated telemetry if no host is currently draining the PTY."""
    try:
        os.write(fd, data)
    except (BlockingIOError, OSError):
        pass


class Parser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while True:
            start = self.buffer.find(b'\xAA\x55')
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer.endswith(b'\xAA') else b''
                return frames
            del self.buffer[:start]
            if len(self.buffer) < 8:
                return frames
            length = int.from_bytes(self.buffer[6:8], 'little')
            total = 10 + length
            if length > 128:
                del self.buffer[0]
                continue
            if len(self.buffer) < total:
                return frames
            body = bytes(self.buffer[2:8 + length])
            expected = int.from_bytes(self.buffer[8 + length:total], 'little')
            if crc16(body) != expected:
                del self.buffer[0]
                continue
            frames.append((self.buffer[3], self.buffer[4], self.buffer[5],
                           bytes(self.buffer[8:8 + length])))
            del self.buffer[:total]


def motor_record(motor_id, target, actual, count):
    state = 2 if actual else 1
    return struct.pack('<BBBBhhhHBBHIHH', motor_id, 2, state, 0,
                       target, actual, 0, 0, 0, 0, 0, count, 0, 0)


def main():
    arguments = argparse.ArgumentParser()
    arguments.add_argument('--symlink', help='Optional stable link, e.g. /tmp/navigation_base')
    args = arguments.parse_args()

    master, slave = pty.openpty()
    os.set_blocking(master, False)
    slave_name = os.ttyname(slave)
    if args.symlink:
        try:
            os.unlink(args.symlink)
        except FileNotFoundError:
            pass
        os.symlink(slave_name, args.symlink)
    print(args.symlink or slave_name, flush=True)

    parser = Parser()
    started = time.monotonic()
    last_keepalive = started
    last_chassis = 0.0
    last_imu = 0.0
    target_left = target_right = 0
    telemetry_count = watchdog_count = imu_count = 0
    active = False
    output_sequence = 0
    # Firmware preconditions: a motor must be selected and confirmed by a query
    # before its mode can be set, and dual motion requires speed mode on both.
    confirmed = set()
    selected_id = 1
    modes = {}

    try:
        while True:
            now = time.monotonic()
            readable, _, _ = select.select([master], [], [], 0.005)
            if readable:
                try:
                    data = os.read(master, 4096)
                except BlockingIOError:
                    data = b''
                for msg_type, sequence, flags, payload in parser.feed(data):
                    status = 0
                    if msg_type == 0x01:  # HELLO
                        if payload:
                            status = 2
                    elif msg_type == 0x13:  # QUERY: select and confirm a motor
                        if len(payload) != 1:
                            status = 2
                        elif payload[0] == 0xC8:
                            status = 3
                        else:
                            selected_id = payload[0]
                            confirmed.add(payload[0])
                    elif msg_type == 0x16:  # SET_MODE requires a prior query of this motor
                        if len(payload) != 2:
                            status = 2
                        elif payload[0] == 0xC8 or payload[1] not in (1, 2, 3):
                            status = 3
                        elif payload[0] not in confirmed or selected_id != payload[0]:
                            status = 7
                        else:
                            modes[payload[0]] = payload[1]
                    elif msg_type == 0x1A:  # dual RPM
                        if len(payload) != 8:
                            status = 2
                        else:
                            left_id, right_id, new_left, new_right, _, _ = struct.unpack(
                                '<BBhhBB', payload)
                            if (left_id != 1 or right_id != 2 or
                                    not -125 <= new_left <= 125 or
                                    not -125 <= new_right <= 125):
                                status = 3
                            elif (left_id not in confirmed or right_id not in confirmed or
                                    modes.get(left_id) != 2 or modes.get(right_id) != 2):
                                status = 7
                            else:
                                target_left, target_right = new_left, new_right
                                active = new_left != 0 or new_right != 0
                                last_keepalive = now
                    elif msg_type == 0x1B:  # dual keepalive is only valid while active
                        if payload != b'\x01\x02':
                            status = 2
                        elif not active:
                            status = 7
                        else:
                            last_keepalive = now
                    elif msg_type == 0x1C:  # dual stop
                        if len(payload) != 3:
                            status = 2
                        elif payload[0] != 1 or payload[1] != 2:
                            status = 3
                        else:
                            target_left = target_right = 0
                            active = False
                    else:
                        status = 8
                    if flags & 1 or status:
                        ack = struct.pack('<BBh', msg_type, status, 0)
                        try_write(master, encode(0x80, sequence, ack))

            if active and now - last_keepalive > 0.3:
                target_left = target_right = 0
                active = False
                watchdog_count += 1

            if now - last_chassis >= 0.1:
                telemetry_count += 1
                uptime_ms = int((now - started) * 1000) & 0xFFFFFFFF
                flags = 0x18 | (0x01 if active else 0)
                payload = struct.pack('<IBBH', uptime_ms, 1 if active else 0, flags, watchdog_count)
                payload += motor_record(1, target_left, target_left, telemetry_count)
                payload += motor_record(2, target_right, target_right, telemetry_count)
                try_write(master, encode(0x92, output_sequence, payload))
                output_sequence = (output_sequence + 1) & 0xFF
                last_chassis = now

            if now - last_imu >= 0.01:
                imu_count += 1
                timestamp_us = int((now - started) * 1_000_000)
                payload = struct.pack('<QB3x6fIHH', timestamp_us, 0x07,
                                      0.0, 0.0, 9.80665, 0.0, 0.0, 0.0,
                                      imu_count, 0, 0)
                try_write(master, encode(0x93, output_sequence, payload))
                output_sequence = (output_sequence + 1) & 0xFF
                last_imu = now
    except KeyboardInterrupt:
        pass
    finally:
        if args.symlink:
            try:
                os.unlink(args.symlink)
            except FileNotFoundError:
                pass
        os.close(master)
        os.close(slave)


if __name__ == '__main__':
    sys.exit(main())
