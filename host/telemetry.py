"""遥测速率统计与底盘/IMU 遥测载荷的纯解析。"""
from __future__ import annotations

import struct

from host.config import TELEMETRY_RATE_WINDOW_S


class TelemetryRateMeter:
    def __init__(self, window_s: float = TELEMETRY_RATE_WINDOW_S) -> None:
        self.window_s = window_s
        self.samples: list[tuple[float, int, int]] = []
        self.last_sequence: int | None = None
        self.last_feedback: int | None = None
        self.heartbeat_total = 0

    def reset(self) -> None:
        self.samples.clear()
        self.last_sequence = None
        self.last_feedback = None
        self.heartbeat_total = 0

    def update(self, now: float, sequence: int,
               feedback_count: int) -> tuple[float, float]:
        if self.last_feedback is not None and feedback_count < self.last_feedback:
            self.reset()
        if self.last_sequence is not None:
            self.heartbeat_total += (sequence - self.last_sequence) & 0xFF
        self.last_sequence = sequence
        self.last_feedback = feedback_count
        self.samples.append((now, self.heartbeat_total, feedback_count))
        cutoff = now - self.window_s
        while len(self.samples) > 2 and self.samples[1][0] <= cutoff:
            del self.samples[0]
        if len(self.samples) < 2:
            return 0.0, 0.0
        first_time, first_frames, first_feedback = self.samples[0]
        duration = now - first_time
        if duration <= 0.0:
            return 0.0, 0.0
        return ((self.heartbeat_total - first_frames) / duration,
                max(0, feedback_count - first_feedback) / duration)


def _decode_motor_record(payload: bytes, offset: int) -> dict[str, int]:
    motor_id, mode, state, fault = struct.unpack_from("<BBBB", payload, offset)
    target, speed, current = struct.unpack_from("<hhh", payload, offset + 4)
    position = struct.unpack_from("<H", payload, offset + 10)[0]
    query_position = payload[offset + 12]
    age = struct.unpack_from("<H", payload, offset + 14)[0]
    feedback = struct.unpack_from("<I", payload, offset + 16)[0]
    crc_errors, timeouts = struct.unpack_from("<HH", payload, offset + 20)
    return {
        "motor_id": motor_id, "mode": mode, "state": state, "fault": fault,
        "target": target, "speed": speed, "current": current,
        "position": position, "query_position": query_position,
        "age": age, "feedback": feedback,
        "crc_errors": crc_errors, "timeouts": timeouts,
    }


def decode_chassis_telemetry(
        payload: bytes) -> tuple[int, int, int, int, dict[str, int], dict[str, int]]:
    """解析 0x92 底盘遥测载荷，返回头部字段与左右电机记录。"""
    uptime, owner, flags, watchdogs = struct.unpack_from("<IBBH", payload, 0)
    left = _decode_motor_record(payload, 8)
    right = _decode_motor_record(payload, 32)
    return uptime, owner, flags, watchdogs, left, right


def decode_imu_telemetry(
        payload: bytes) -> tuple[int, int, tuple[float, float, float],
                                 tuple[float, float, float], int, int, int]:
    """解析 0x93 IMU 遥测载荷，返回时间戳、标志、加速度和陀螺仪等字段。"""
    timestamp = struct.unpack_from("<Q", payload, 0)[0]
    flags = payload[8]
    accel = struct.unpack_from("<fff", payload, 12)
    gyro = struct.unpack_from("<fff", payload, 24)
    samples = struct.unpack_from("<I", payload, 36)[0]
    read_errors, init_errors = struct.unpack_from("<HH", payload, 40)
    return timestamp, flags, accel, gyro, samples, read_errors, init_errors
