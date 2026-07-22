"""HELLO 握手控制与 ESP32 复位检测。"""
from __future__ import annotations

import time

from host.config import (HANDSHAKE_INITIAL_DELAY_S, HANDSHAKE_RETRY_S,
                         HANDSHAKE_TIMEOUT_S, RESET_MARKER, RESET_WINDOW_S)


class HandshakeController:
    WAIT = 0
    SEND = 1
    TIMEOUT = 2

    def __init__(self) -> None:
        self.active = False
        self.deadline = 0.0
        self.next_send = 0.0

    def start(self, now: float) -> None:
        self.active = True
        self.deadline = now + HANDSHAKE_TIMEOUT_S
        self.next_send = now + HANDSHAKE_INITIAL_DELAY_S

    def complete(self) -> None:
        self.active = False

    def poll(self, now: float) -> int:
        if not self.active:
            return self.WAIT
        if now >= self.deadline:
            self.active = False
            return self.TIMEOUT
        if now >= self.next_send:
            self.next_send = now + HANDSHAKE_RETRY_S
            return self.SEND
        return self.WAIT


class ResetDetector:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.reset_times: list[float] = []

    def feed(self, data: bytes, now: float | None = None) -> int:
        timestamp = time.monotonic() if now is None else now
        self.buffer.extend(data)
        found = False
        while True:
            position = self.buffer.find(RESET_MARKER)
            if position < 0:
                keep = max(0, len(RESET_MARKER) - 1)
                self.buffer[:] = self.buffer[-keep:]
                break
            found = True
            del self.buffer[:position + len(RESET_MARKER)]
            self.reset_times.append(timestamp)
        self.reset_times = [item for item in self.reset_times
                            if timestamp - item <= RESET_WINDOW_S]
        return len(self.reset_times) if found else 0
