"""运动命令合并与单条在途 ACK 门控。"""
from __future__ import annotations

import struct
from dataclasses import dataclass

from host.config import CONTROL_KEEPALIVE_PERIOD_S, MOTION_ACK_TIMEOUT_S
from host.protocol import MSG_CONTROL_KEEPALIVE


@dataclass(frozen=True)
class MotionCommand:
    msg_type: int
    payload: bytes
    signature: tuple[int, ...]
    motor_id: int
    mode: int
    active: bool
    keepalive_type: int = MSG_CONTROL_KEEPALIVE
    keepalive_payload: bytes = b""


class MotionCommandGate:
    """Coalesce motion updates and allow only one ACKed command in flight."""

    def __init__(self, ack_timeout_s: float = MOTION_ACK_TIMEOUT_S,
                 keepalive_period_s: float = CONTROL_KEEPALIVE_PERIOD_S) -> None:
        self.ack_timeout_s = ack_timeout_s
        self.keepalive_period_s = keepalive_period_s
        self.inflight_sequence: int | None = None
        self.inflight_command: MotionCommand | None = None
        self.pending_command: MotionCommand | None = None
        self.last_acked_signature: tuple[int, ...] | None = None
        self.ack_deadline = 0.0
        self.next_keepalive = 0.0
        self.active_id: int | None = None
        self.active_keepalive_type = MSG_CONTROL_KEEPALIVE
        self.active_keepalive_payload = b""
        self.blocked = False

    def reset(self) -> None:
        self.inflight_sequence = None
        self.inflight_command = None
        self.pending_command = None
        self.last_acked_signature = None
        self.ack_deadline = 0.0
        self.next_keepalive = 0.0
        self.active_id = None
        self.active_keepalive_type = MSG_CONTROL_KEEPALIVE
        self.active_keepalive_payload = b""
        self.blocked = False

    def resume(self) -> None:
        self.blocked = False

    def offer(self, command: MotionCommand) -> MotionCommand | None:
        if self.blocked:
            return None
        if self.inflight_command is not None:
            self.pending_command = (
                None if command.signature == self.inflight_command.signature else command)
            return None
        if command.signature == self.last_acked_signature:
            return None
        return command

    def mark_sent(self, sequence: int, command: MotionCommand, now: float) -> None:
        if self.inflight_command is not None:
            raise RuntimeError("a motion command is already awaiting ACK")
        self.inflight_sequence = sequence
        self.inflight_command = command
        self.ack_deadline = now + self.ack_timeout_s

    def acknowledge(self, sequence: int, success: bool,
                    now: float) -> MotionCommand | None:
        if sequence != self.inflight_sequence or self.inflight_command is None:
            return None
        acknowledged = self.inflight_command
        pending = self.pending_command
        self.inflight_sequence = None
        self.inflight_command = None
        self.pending_command = None
        self.ack_deadline = 0.0
        if not success:
            self.active_id = None
            self.last_acked_signature = None
            self.blocked = True
            return None
        self.last_acked_signature = acknowledged.signature
        self.active_id = acknowledged.motor_id if acknowledged.active else None
        self.active_keepalive_type = acknowledged.keepalive_type
        self.active_keepalive_payload = acknowledged.keepalive_payload
        self.next_keepalive = now + self.keepalive_period_s
        if pending is not None and pending.signature != self.last_acked_signature:
            return pending
        return None

    def check_timeout(self, now: float) -> bool:
        if self.inflight_command is None or now < self.ack_deadline:
            return False
        self.inflight_sequence = None
        self.inflight_command = None
        self.pending_command = None
        self.last_acked_signature = None
        self.active_id = None
        self.ack_deadline = 0.0
        self.blocked = True
        return True

    def keepalive_due(self, now: float) -> int | None:
        if self.blocked or self.active_id is None or now < self.next_keepalive:
            return None
        while self.next_keepalive <= now:
            self.next_keepalive += self.keepalive_period_s
        return self.active_id

    def keepalive_command_due(self, now: float) -> tuple[int, bytes] | None:
        active_id = self.keepalive_due(now)
        if active_id is None:
            return None
        payload = self.active_keepalive_payload or struct.pack("<B", active_id)
        return self.active_keepalive_type, payload

    def reject_keepalive(self) -> None:
        self.active_id = None
        self.pending_command = None
        self.blocked = True
