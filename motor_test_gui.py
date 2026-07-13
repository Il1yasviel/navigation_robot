"""ESP32-S3 navigation robot dual-motor and BMI088 diagnostic GUI."""

from __future__ import annotations

import queue
import socket
import struct
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox, ttk
from tkinter.scrolledtext import ScrolledText
from typing import Callable

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # Unit tests can run without pyserial.
    serial = None
    list_ports = None

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

M0601_BRAKE_OFF = 0x00
M0601_BRAKE_ON = 0xFF
M0601_RESERVED_QUERY_ID = 0xC8
M0601_MODE_CURRENT = 0x01
M0601_MODE_SPEED = 0x02
M0601_MODE_POSITION = 0x03
MAX_CURRENT_MA = 1000.0
MAX_RPM = 125
SPEED_GEARS = (25, 50, 75, 100, 125)
LEFT_MOTOR_ID = 1
RIGHT_MOTOR_ID = 2
SET_ID_CONFIRM = 0x4D36

HANDSHAKE_INITIAL_DELAY_S = 1.0
HANDSHAKE_RETRY_S = 0.3
HANDSHAKE_TIMEOUT_S = 2.5
RESET_WINDOW_S = 5.0
RESET_MARKER = b"ESP-ROM:"
OPEN_TIMEOUT_S = 6.0
STARTUP_PURGE_MAX_S = 1.5
STARTUP_PURGE_QUIET_READS = 3
POLL_EVENT_LIMIT = 48
POLL_TIME_BUDGET_S = 0.008
MOTION_ACK_TIMEOUT_S = 0.250
CONTROL_KEEPALIVE_PERIOD_S = 0.100
TELEMETRY_RATE_WINDOW_S = 2.0

STATUS_TEXT = {
    0: "成功",
    1: "主机CRC错误",
    2: "长度错误",
    3: "参数越界",
    4: "控制权被占用",
    5: "电机响应超时",
    6: "电机CRC错误",
    7: "前置条件不满足",
    8: "不支持的命令",
    9: "I/O错误",
}

MOTOR_STATE_TEXT = {0: "离线", 1: "空闲", 2: "运行", 3: "故障", 4: "急停"}
MODE_TEXT = {
    M0601_MODE_CURRENT: "电流模式",
    M0601_MODE_SPEED: "速度模式",
    M0601_MODE_POSITION: "位置模式",
}
MODE_BY_TEXT = {text: mode for mode, text in MODE_TEXT.items()}
FAULT_NAMES = ("传感器", "过流", "相线过流", "堵转",
               "故障位4", "保留5", "保留6", "保留7")


def current_ma_to_raw(current_ma: float) -> int:
    if not -MAX_CURRENT_MA <= current_ma <= MAX_CURRENT_MA:
        raise ValueError(f"电流目标必须在±{MAX_CURRENT_MA:.0f}mA以内")
    return int(round(current_ma * 32767.0 / 8000.0))


def current_raw_to_ma(current_raw: int) -> float:
    return current_raw * 8000.0 / 32767.0


def degrees_to_position_raw(degrees: float) -> int:
    if not 0.0 <= degrees <= 360.0:
        raise ValueError("位置目标必须在0～360°之间")
    return int(round(degrees * 32767.0 / 360.0))


def position_raw_to_degrees(position_raw: int) -> float:
    return position_raw * 360.0 / 32767.0


def differential_rpm(x_permille: int, y_permille: int,
                     maximum_rpm: int) -> tuple[int, int]:
    maximum = max(1, min(MAX_RPM, int(maximum_rpm)))
    x = max(-1000, min(1000, int(x_permille)))
    y = max(-1000, min(1000, int(y_permille)))
    left = max(-maximum, min(maximum, int((y + x) * maximum / 1000)))
    right = max(-maximum, min(maximum, int((y - x) * maximum / 1000)))
    return left, right


def keyboard_direction_rpm(direction: str, gear: int) -> tuple[int, int]:
    if gear not in SPEED_GEARS:
        raise ValueError("速度档位无效")
    mapping = {
        "Up": (gear, gear),
        "Down": (-gear, -gear),
        "Left": (-gear, gear),
        "Right": (gear, -gear),
    }
    if direction not in mapping:
        raise ValueError("方向键无效")
    return mapping[direction]


def control_state_flags(link_ready: bool, target_confirmed: bool,
                        current_mode: int | None,
                        maintenance_enabled: bool) -> dict[str, bool]:
    motion_ready = link_ready and target_confirmed
    return {
        "query": link_ready,
        "motion": motion_ready,
        "joystick": motion_ready and current_mode == M0601_MODE_SPEED,
        "maintenance": link_ready and maintenance_enabled,
    }


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


def open_ftdi_serial(port: str, serial_factory=None):
    if serial_factory is None and serial is None:
        raise RuntimeError("缺少pyserial，请运行 pip install -r requirements-host.txt")
    factory = serial.Serial if serial_factory is None else serial_factory
    connection = factory(port=None, baudrate=115200, timeout=0.03,
                         write_timeout=0.2, rtscts=False, dsrdtr=False)
    connection.dtr = False
    connection.rts = False
    connection.port = port
    connection.open()
    connection.reset_input_buffer()
    return connection


class SerialWorker:
    def __init__(self, port: str, session_id: int,
                 events: queue.Queue[tuple[int, str, object]],
                 serial_factory=None) -> None:
        self.port = port
        self.session_id = session_id
        self.events = events
        self.tx: queue.Queue[bytes] = queue.Queue()
        self.stop_event = threading.Event()
        self.serial_factory = serial_factory
        self.connection = None
        self.thread = threading.Thread(target=self._run, name="robot-host-uart", daemon=True)
        self.thread.start()

    def send(self, data: bytes) -> None:
        if not self.stop_event.is_set():
            self.tx.put(data)

    def _emit(self, kind: str, value: object) -> None:
        self.events.put((self.session_id, kind, value))

    def _purge_startup_backlog(self) -> None:
        deadline = time.monotonic() + STARTUP_PURGE_MAX_S
        quiet_reads = 0
        while (not self.stop_event.is_set() and time.monotonic() < deadline and
               quiet_reads < STARTUP_PURGE_QUIET_READS):
            self.connection.reset_input_buffer()
            quiet_reads = 0 if self.connection.read(256) else quiet_reads + 1
        self.connection.reset_input_buffer()

    def _run(self) -> None:
        try:
            self.connection = open_ftdi_serial(self.port, self.serial_factory)
            self._purge_startup_backlog()
            if self.stop_event.is_set():
                return
            self._emit("opened", self.port)
            while not self.stop_event.is_set():
                while True:
                    try:
                        data = self.tx.get_nowait()
                    except queue.Empty:
                        break
                    self.connection.write(data)
                    self._emit("tx", data)
                received = self.connection.read(256)
                if received:
                    self._emit("rx", received)
        except Exception as exc:
            self._emit("error", str(exc))
        finally:
            try:
                if self.connection is not None and self.connection.is_open:
                    self.connection.close()
            finally:
                self._emit("closed", None)

    def close(self) -> None:
        self.stop_event.set()
        if self.connection is None:
            return
        cancel_read = getattr(self.connection, "cancel_read", None)
        if callable(cancel_read):
            try:
                cancel_read()
            except Exception:
                pass
        try:
            if self.connection.is_open:
                self.connection.close()
        except Exception:
            pass


class TcpWorker:
    def __init__(self, host: str, port: int, session_id: int,
                 events: queue.Queue[tuple[int, str, object]],
                 socket_factory: Callable[..., socket.socket] | None = None) -> None:
        self.host = host
        self.port = port
        self.session_id = session_id
        self.events = events
        self.tx: queue.Queue[bytes] = queue.Queue()
        self.stop_event = threading.Event()
        self.socket_factory = socket.create_connection if socket_factory is None else socket_factory
        self.connection: socket.socket | None = None
        self.thread = threading.Thread(target=self._run, name="robot-host-tcp", daemon=True)
        self.thread.start()

    def _emit(self, kind: str, value: object) -> None:
        self.events.put((self.session_id, kind, value))

    def send(self, data: bytes) -> None:
        if not self.stop_event.is_set():
            self.tx.put(data)

    def _discard_pending_tx(self) -> None:
        while True:
            try:
                self.tx.get_nowait()
            except queue.Empty:
                return

    def _run(self) -> None:
        retry_delay = 1.0
        while not self.stop_event.is_set():
            try:
                self.connection = self.socket_factory((self.host, self.port), timeout=2.0)
                self.connection.settimeout(0.03)
                retry_delay = 1.0
                self._emit("opened", f"{self.host}:{self.port}")
                while not self.stop_event.is_set():
                    while True:
                        try:
                            data = self.tx.get_nowait()
                        except queue.Empty:
                            break
                        self.connection.sendall(data)
                        self._emit("tx", data)
                    try:
                        received = self.connection.recv(256)
                    except socket.timeout:
                        continue
                    if not received:
                        raise ConnectionError("TCP连接已关闭")
                    self._emit("rx", received)
            except Exception as exc:
                if not self.stop_event.is_set():
                    self._discard_pending_tx()
                    self._emit("retrying", str(exc))
                    self.stop_event.wait(retry_delay)
                    retry_delay = min(10.0, retry_delay * 2.0)
            finally:
                if self.connection is not None:
                    try:
                        self.connection.close()
                    except Exception:
                        pass
                    self.connection = None
        self._emit("closed", None)

    def close(self) -> None:
        self.stop_event.set()
        if self.connection is not None:
            try:
                self.connection.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                self.connection.close()
            except Exception:
                pass


class VirtualJoystick(tk.Canvas):
    def __init__(self, parent: tk.Misc, on_release: Callable[[], None]) -> None:
        super().__init__(parent, width=240, height=240, bg="#17202a", highlightthickness=0)
        self.center = 120.0
        self.radius = 88.0
        self.x_value = 0.0
        self.y_value = 0.0
        self.active = False
        self.on_release = on_release
        self.create_oval(32, 32, 208, 208, fill="#253746", outline="#5d768a", width=2)
        self.create_line(120, 32, 120, 208, fill="#486273")
        self.create_line(32, 120, 208, 120, fill="#486273")
        self.knob = self.create_oval(96, 96, 144, 144, fill="#28a5da", outline="")
        self.bind("<Button-1>", self._move)
        self.bind("<B1-Motion>", self._move)
        self.bind("<ButtonRelease-1>", self._release)

    def _move(self, event: tk.Event) -> None:
        dx = float(event.x) - self.center
        dy = float(event.y) - self.center
        distance = (dx * dx + dy * dy) ** 0.5
        if distance > self.radius:
            dx *= self.radius / distance
            dy *= self.radius / distance
        self.x_value = 0.0 if abs(dx / self.radius) < 0.08 else dx / self.radius
        self.y_value = 0.0 if abs(-dy / self.radius) < 0.08 else -dy / self.radius
        self.active = True
        self.coords(self.knob, self.center + dx - 24, self.center + dy - 24,
                    self.center + dx + 24, self.center + dy + 24)

    def _release(self, _event: tk.Event) -> None:
        self.active = False
        self.x_value = self.y_value = 0.0
        self.coords(self.knob, 96, 96, 144, 144)
        self.on_release()


class MotorTestApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("ESP32-S3 Navigation Robot 双轮与IMU测试")
        self.root.geometry("1240x860")
        self.events: queue.Queue[tuple[int, str, object]] = queue.Queue()
        self.parser = FrameParser()
        self.worker: SerialWorker | TcpWorker | None = None
        self.sequence = 0
        self.session_id = 0
        self.hello_sequence: int | None = None
        self.hello_frame: bytes | None = None
        self.pending_requests: dict[int, dict[str, object]] = {}
        self.motion_gate = MotionCommandGate()
        self.left_rate = TelemetryRateMeter()
        self.right_rate = TelemetryRateMeter()
        self.imu_rate = TelemetryRateMeter()
        self.link_ready = False
        self.disconnect_pending = False
        self.handshake = HandshakeController()
        self.handshake_job: str | None = None
        self.open_timeout_job: str | None = None
        self.reset_detector = ResetDetector()
        self.confirmed_id: int | None = None
        self.current_mode: int | None = None
        self.dual_prepared = False
        self.dual_ready = False
        self.pressed_directions: list[str] = []
        self.latest_imu: tuple[object, ...] | None = None
        self.next_imu_render = 0.0

        self.connection_type_var = tk.StringVar(value="USB串口")
        self.port_var = tk.StringVar()
        self.host_var = tk.StringVar(value="navigation-robot.local")
        self.tcp_port_var = tk.IntVar(value=3333)
        self.connection_var = tk.StringVar(value="未连接")
        self.gear_var = tk.IntVar(value=25)
        self.axis_var = tk.StringVar(value="X=+0.000 Y=+0.000 / 左=0 右=0 RPM")
        self.dual_state_var = tk.StringVar(value="未准备")
        self.owner_var = tk.StringVar(value="无")
        self.motor_id_var = tk.StringVar(value="1")
        self.new_id_var = tk.StringVar(value="2")
        self.maintenance_var = tk.BooleanVar(value=False)
        self.mode_var = tk.StringVar(value=MODE_TEXT[M0601_MODE_SPEED])
        self.target_var = tk.StringVar(value="30")
        self.target_label_var = tk.StringVar(value="目标速度 (RPM，±125)")
        self.single_status_var = tk.StringVar(value="请先按ID查询")
        self.ack_var = tk.StringVar(value="--")
        self.wheel_vars = {
            side: {key: tk.StringVar(value="--") for key in
                   ("id", "mode", "state", "target", "speed", "current",
                    "position", "fault", "age", "rate", "errors")}
            for side in ("left", "right")
        }
        self.imu_vars = {key: tk.StringVar(value="--") for key in
                         ("state", "accel", "gyro", "rate", "samples", "errors", "time")}

        self._build_ui()
        self.motor_id_var.trace_add("write", self._target_id_changed)
        self.connection_type_var.trace_add("write", self._connection_type_changed)
        self.root.bind_all("<KeyPress>", self._key_press, add="+")
        self.root.bind_all("<KeyRelease>", self._key_release, add="+")
        self._set_controls_enabled()
        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(20, self._poll)
        self.root.after(50, self._control_tick)

    def _build_ui(self) -> None:
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill="x")
        ttk.Label(top, text="连接方式").pack(side="left")
        self.connection_type_box = ttk.Combobox(
            top, textvariable=self.connection_type_var,
            values=("USB串口", "WiFi TCP"), state="readonly", width=10)
        self.connection_type_box.pack(side="left", padx=5)
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=15, state="readonly")
        self.port_box.pack(side="left", padx=5)
        self.refresh_button = ttk.Button(top, text="刷新", command=self.refresh_ports)
        self.refresh_button.pack(side="left")
        self.host_entry = ttk.Entry(top, textvariable=self.host_var, width=24)
        self.tcp_port_entry = ttk.Entry(top, textvariable=self.tcp_port_var, width=7)
        self.connect_button = ttk.Button(top, text="连接", command=self.toggle_connection)
        self.connect_button.pack(side="left", padx=8)
        ttk.Label(top, textvariable=self.connection_var).pack(side="left", padx=6)
        ttk.Button(top, text="急停双轮", command=lambda: self.send_dual_stop(True)).pack(side="right")

        body = ttk.Panedwindow(self.root, orient="horizontal")
        body.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        controls = ttk.Frame(body, padding=4)
        monitor = ttk.Frame(body, padding=4)
        body.add(controls, weight=1)
        body.add(monitor, weight=2)

        control_tabs = ttk.Notebook(controls)
        control_tabs.pack(fill="both", expand=True)
        dual_tab = ttk.Frame(control_tabs, padding=10)
        single_tab = ttk.Frame(control_tabs, padding=10)
        control_tabs.add(dual_tab, text="双轮底盘")
        control_tabs.add(single_tab, text="单电机调试")

        self.prepare_button = ttk.Button(
            dual_tab, text="准备双轮控制（ID1 + ID2）", command=self.prepare_dual_control)
        self.prepare_button.pack(fill="x")
        ttk.Label(dual_tab, textvariable=self.dual_state_var).pack(pady=4)
        self.joystick = VirtualJoystick(dual_tab, lambda: self.send_dual_stop(False))
        self.joystick.pack(pady=4)
        ttk.Label(dual_tab, textvariable=self.axis_var).pack()
        gear_frame = ttk.LabelFrame(dual_tab, text="速度档位（数字键1～5）", padding=6)
        gear_frame.pack(fill="x", pady=8)
        self.gear_buttons = []
        for index, gear in enumerate(SPEED_GEARS):
            button = ttk.Radiobutton(gear_frame, text=str(gear), value=gear,
                                     variable=self.gear_var)
            button.grid(row=0, column=index, padx=4)
            self.gear_buttons.append(button)
        ttk.Label(dual_tab, text="方向键：↑前进  ↓后退  ←原地左转  →原地右转").pack(pady=6)
        ttk.Label(dual_tab, text="右轮反向由下位机统一处理，上位机发送车体逻辑RPM。",
                  foreground="#555").pack()

        id_box = ttk.LabelFrame(single_tab, text="目标电机", padding=8)
        id_box.pack(fill="x")
        ttk.Label(id_box, text="ID").grid(row=0, column=0, sticky="w")
        self.motor_id_entry = ttk.Entry(id_box, textvariable=self.motor_id_var, width=8)
        self.motor_id_entry.grid(row=0, column=1, padx=5)
        self.query_target_button = ttk.Button(id_box, text="按ID查询", command=self.query_target_motor)
        self.query_target_button.grid(row=0, column=2)
        ttk.Label(id_box, textvariable=self.single_status_var).grid(
            row=1, column=0, columnspan=3, sticky="w", pady=5)

        mode_box = ttk.LabelFrame(single_tab, text="三模式控制", padding=8)
        mode_box.pack(fill="x", pady=8)
        self.mode_box = ttk.Combobox(mode_box, textvariable=self.mode_var,
                                     values=tuple(MODE_BY_TEXT), state="readonly", width=12)
        self.mode_box.grid(row=0, column=0)
        self.mode_box.bind("<<ComboboxSelected>>", self._mode_selection_changed)
        self.set_mode_button = ttk.Button(mode_box, text="切换模式", command=self.set_motor_mode)
        self.set_mode_button.grid(row=0, column=1, padx=5)
        ttk.Label(mode_box, textvariable=self.target_label_var).grid(
            row=1, column=0, columnspan=2, sticky="w", pady=(8, 2))
        self.target_entry = ttk.Entry(mode_box, textvariable=self.target_var, width=14)
        self.target_entry.grid(row=2, column=0)
        self.send_target_button = ttk.Button(mode_box, text="发送目标", command=self.send_target)
        self.send_target_button.grid(row=2, column=1, padx=5)
        self.single_stop_button = ttk.Button(
            mode_box, text="停止单电机", command=lambda: self.send_single_stop(True))
        self.single_stop_button.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(8, 0))

        maintenance = ttk.LabelFrame(single_tab, text="单电机维护", padding=8)
        maintenance.pack(fill="x")
        self.maintenance_check = ttk.Checkbutton(
            maintenance, text="确认总线仅连接一台电机", variable=self.maintenance_var,
            command=self._maintenance_changed)
        self.maintenance_check.grid(row=0, column=0, columnspan=3, sticky="w")
        self.query_button = ttk.Button(maintenance, text="查询唯一ID", command=self.query_unique_id)
        self.query_button.grid(row=1, column=2, pady=5)
        ttk.Label(maintenance, text="新ID").grid(row=2, column=0)
        self.new_id_entry = ttk.Entry(maintenance, textvariable=self.new_id_var, width=8)
        self.new_id_entry.grid(row=2, column=1, padx=5)
        self.set_id_button = ttk.Button(maintenance, text="修改ID", command=self.set_motor_id)
        self.set_id_button.grid(row=2, column=2)

        monitor_tabs = ttk.Notebook(monitor)
        monitor_tabs.pack(fill="both", expand=True)
        motor_tab = ttk.Frame(monitor_tabs, padding=10)
        imu_tab = ttk.Frame(monitor_tabs, padding=10)
        log_tab = ttk.Frame(monitor_tabs, padding=5)
        monitor_tabs.add(motor_tab, text="左右轮信息")
        monitor_tabs.add(imu_tab, text="BMI088 IMU")
        monitor_tabs.add(log_tab, text="通信日志")

        ttk.Label(motor_tab, text="控制来源").grid(row=0, column=0, sticky="w")
        ttk.Label(motor_tab, textvariable=self.owner_var).grid(row=0, column=1, sticky="w")
        ttk.Label(motor_tab, text="最近ACK").grid(row=1, column=0, sticky="w")
        ttk.Label(motor_tab, textvariable=self.ack_var).grid(row=1, column=1, columnspan=2, sticky="w")
        fields = (("ID", "id"), ("模式", "mode"), ("状态", "state"),
                  ("目标", "target"), ("实际速度", "speed"),
                  ("力矩电流", "current"), ("位置", "position"),
                  ("故障", "fault"), ("反馈年龄", "age"),
                  ("刷新率", "rate"), ("错误", "errors"))
        for column, (side, title) in enumerate((("left", "左轮 ID1"), ("right", "右轮 ID2")), 1):
            frame = ttk.LabelFrame(motor_tab, text=title, padding=8)
            frame.grid(row=2, column=column - 1, sticky="nsew", padx=5, pady=8)
            for row, (label, key) in enumerate(fields):
                ttk.Label(frame, text=label, width=10).grid(row=row, column=0, sticky="w", pady=2)
                ttk.Label(frame, textvariable=self.wheel_vars[side][key]).grid(
                    row=row, column=1, sticky="w", pady=2)
        motor_tab.columnconfigure(0, weight=1)
        motor_tab.columnconfigure(1, weight=1)

        imu_fields = (("状态", "state"), ("线加速度", "accel"), ("角速度", "gyro"),
                      ("数据频率", "rate"), ("样本数", "samples"),
                      ("错误", "errors"), ("MCU时间戳", "time"))
        for row, (label, key) in enumerate(imu_fields):
            ttk.Label(imu_tab, text=label, width=14).grid(row=row, column=0, sticky="w", pady=5)
            ttk.Label(imu_tab, textvariable=self.imu_vars[key]).grid(row=row, column=1, sticky="w", pady=5)
        ttk.Label(imu_tab, text="坐标系：X向前、Y向左、Z向上；单位为m/s²和rad/s。",
                  foreground="#555").grid(row=len(imu_fields), column=0, columnspan=2,
                                            sticky="w", pady=10)

        self.log = ScrolledText(log_tab, font=("Consolas", 9), state="disabled")
        self.log.pack(fill="both", expand=True)
        self._connection_type_changed()

    def _connection_type_changed(self, *_args: object) -> None:
        wifi = self.connection_type_var.get() == "WiFi TCP"
        if wifi:
            self.port_box.pack_forget()
            self.refresh_button.pack_forget()
            self.host_entry.pack(side="left", padx=5, before=self.connect_button)
            self.tcp_port_entry.pack(side="left", padx=2, before=self.connect_button)
        else:
            self.host_entry.pack_forget()
            self.tcp_port_entry.pack_forget()
            self.port_box.pack(side="left", padx=5, before=self.connect_button)
            self.refresh_button.pack(side="left", before=self.connect_button)

    def refresh_ports(self) -> None:
        infos = [] if list_ports is None else list(list_ports.comports())
        ports = [item.device for item in infos]
        self.port_box["values"] = ports
        if ports and self.port_var.get() not in ports:
            ftdi = next((item.device for item in infos
                         if item.vid == 0x0403 and item.pid == 0x6001), None)
            self.port_var.set(ftdi or ports[0])

    def _set_controls_enabled(self) -> None:
        state = "normal" if self.link_ready else "disabled"
        self.prepare_button.configure(state=state)
        joystick_state = "normal" if self.dual_ready else "disabled"
        self.joystick.configure(state=joystick_state)
        for button in self.gear_buttons:
            button.configure(state=joystick_state)
        try:
            target_confirmed = self._motor_id() == self.confirmed_id
        except ValueError:
            target_confirmed = False
        flags = control_state_flags(self.link_ready, target_confirmed,
                                    self.current_mode, self.maintenance_var.get())
        self.motor_id_entry.configure(state="normal" if flags["query"] else "disabled")
        self.query_target_button.configure(state="normal" if flags["query"] else "disabled")
        self.maintenance_check.configure(state="normal" if flags["query"] else "disabled")
        self.query_button.configure(state="normal" if flags["maintenance"] else "disabled")
        self.new_id_entry.configure(state="normal" if flags["maintenance"] else "disabled")
        self.set_id_button.configure(
            state="normal" if flags["maintenance"] and target_confirmed else "disabled")
        motion_state = "normal" if flags["motion"] else "disabled"
        self.mode_box.configure(state="readonly" if flags["motion"] else "disabled")
        self.set_mode_button.configure(state=motion_state)
        self.target_entry.configure(state=motion_state)
        self.send_target_button.configure(state=motion_state)
        self.single_stop_button.configure(state=motion_state)

    def _reset_session_state(self) -> None:
        self.parser = FrameParser()
        self.reset_detector = ResetDetector()
        self.link_ready = False
        self.confirmed_id = None
        self.current_mode = None
        self.dual_prepared = False
        self.dual_ready = False
        self.pending_requests.clear()
        self.motion_gate.reset()
        self.left_rate.reset()
        self.right_rate.reset()
        self.imu_rate.reset()
        self.hello_sequence = None
        self.hello_frame = None
        self.pressed_directions.clear()
        self.dual_state_var.set("未准备")
        self._set_controls_enabled()

    def toggle_connection(self) -> None:
        if self.worker is not None:
            self._schedule_disconnect()
            return
        self.session_id += 1
        self._reset_session_state()
        if self.connection_type_var.get() == "WiFi TCP":
            host = self.host_var.get().strip()
            try:
                port = int(self.tcp_port_var.get())
                if not host or not 1 <= port <= 65535:
                    raise ValueError
            except (ValueError, tk.TclError):
                messagebox.showerror("WiFi连接", "请输入有效主机名/IP和TCP端口")
                return
            self.worker = TcpWorker(host, port, self.session_id, self.events)
            self.connection_var.set(f"正在连接 {host}:{port}")
        else:
            if not self.port_var.get():
                messagebox.showwarning("串口", "没有可用串口")
                return
            self.worker = SerialWorker(self.port_var.get(), self.session_id, self.events)
            self.connection_var.set(f"正在打开 {self.port_var.get()}")
        self.connect_button.configure(text="断开")
        if isinstance(self.worker, SerialWorker):
            self._schedule_open_timeout()

    def _schedule_open_timeout(self) -> None:
        self._cancel_open_timeout()
        self.open_timeout_job = self.root.after(
            int(OPEN_TIMEOUT_S * 1000), self._handle_open_timeout)

    def _cancel_open_timeout(self) -> None:
        if self.open_timeout_job is not None:
            self.root.after_cancel(self.open_timeout_job)
            self.open_timeout_job = None

    def _handle_open_timeout(self) -> None:
        self.open_timeout_job = None
        if self.worker is not None and not self.link_ready:
            messagebox.showerror("连接超时", "未能连接并完成HELLO握手")
            self._finish_disconnect("连接超时")

    def _begin_handshake(self, endpoint: object) -> None:
        self._cancel_open_timeout()
        self.parser = FrameParser()
        self.hello_sequence = None
        self.hello_frame = None
        self.connection_var.set(f"链路已打开，等待ESP32应用：{endpoint}")
        self.handshake.start(time.monotonic())
        self._schedule_handshake_tick()

    def _schedule_handshake_tick(self) -> None:
        if self.handshake_job is None:
            self.handshake_job = self.root.after(50, self._handshake_tick)

    def _handshake_tick(self) -> None:
        self.handshake_job = None
        if self.worker is None or self.link_ready:
            return
        action = self.handshake.poll(time.monotonic())
        if action == HandshakeController.SEND:
            self._send_hello()
        elif action == HandshakeController.TIMEOUT:
            if isinstance(self.worker, TcpWorker):
                self.connection_var.set("TCP已连接但ESP32应用无HELLO响应，等待重连")
            else:
                messagebox.showerror("握手失败", "串口已打开，但ESP32没有返回HELLO ACK")
                self._finish_disconnect("握手失败")
            return
        self._schedule_handshake_tick()

    def _schedule_disconnect(self) -> None:
        if self.worker is None or self.disconnect_pending:
            return
        self.disconnect_pending = True
        if self.link_ready:
            self.send_dual_stop(True)
            self.root.after(150, self._finish_disconnect)
        else:
            self._finish_disconnect()

    def _finish_disconnect(self, status: str = "未连接") -> None:
        worker, self.worker = self.worker, None
        self.session_id += 1
        if worker is not None:
            worker.close()
        self.handshake.complete()
        self._cancel_open_timeout()
        if self.handshake_job is not None:
            self.root.after_cancel(self.handshake_job)
            self.handshake_job = None
        self.disconnect_pending = False
        self._reset_session_state()
        self.connection_var.set(status)
        self.connect_button.configure(text="连接")

    def send(self, msg_type: int, payload: bytes = b"", ack: bool = True,
             context: dict[str, object] | None = None) -> int | None:
        if self.worker is None:
            return None
        sequence = self.sequence
        data = encode_frame(msg_type, sequence,
                            FLAG_ACK_REQUIRED if ack else 0, payload)
        self.sequence = (self.sequence + 1) & 0xFF
        if ack and context is not None:
            self.pending_requests[sequence] = {"type": msg_type, **context}
        self.worker.send(data)
        return sequence

    def _send_hello(self) -> None:
        if self.worker is None:
            return
        if self.hello_frame is None:
            self.hello_sequence = self.sequence
            self.hello_frame = encode_frame(MSG_HELLO, self.sequence, FLAG_ACK_REQUIRED)
            self.sequence = (self.sequence + 1) & 0xFF
        self.worker.send(self.hello_frame)

    def _cancel_motion_flow(self) -> None:
        self.motion_gate.reset()
        for sequence in [seq for seq, item in self.pending_requests.items()
                         if item.get("motion") is True]:
            self.pending_requests.pop(sequence, None)

    def _transmit_motion(self, command: MotionCommand) -> None:
        sequence = self.send(command.msg_type, command.payload,
                             context={"motion": True, "id": command.motor_id,
                                      "mode": command.mode})
        if sequence is not None:
            self.motion_gate.mark_sent(sequence, command, time.monotonic())

    def _queue_motion(self, command: MotionCommand) -> None:
        ready = self.motion_gate.offer(command)
        if ready is not None:
            self._transmit_motion(ready)

    def prepare_dual_control(self) -> None:
        if not self.link_ready:
            return
        self._cancel_motion_flow()
        self.dual_prepared = False
        self.dual_ready = False
        self.dual_state_var.set("正在查询左轮ID1…")
        self._set_controls_enabled()
        self.send(MSG_QUERY_MOTOR, struct.pack("<B", LEFT_MOTOR_ID),
                  context={"prepare": "query_left"})

    def _continue_prepare(self, step: str, status: int) -> None:
        if status != 0:
            self.dual_prepared = self.dual_ready = False
            self.dual_state_var.set(f"准备失败：{STATUS_TEXT.get(status, status)}")
            self._set_controls_enabled()
            return
        if step == "query_left":
            self.dual_state_var.set("正在确认左轮速度模式…")
            self.send(MSG_SET_MODE, struct.pack("<BB", LEFT_MOTOR_ID, M0601_MODE_SPEED),
                      context={"prepare": "mode_left"})
        elif step == "mode_left":
            self.dual_state_var.set("正在查询右轮ID2…")
            self.send(MSG_QUERY_MOTOR, struct.pack("<B", RIGHT_MOTOR_ID),
                      context={"prepare": "query_right"})
        elif step == "query_right":
            self.dual_state_var.set("正在确认右轮速度模式…")
            self.send(MSG_SET_MODE, struct.pack("<BB", RIGHT_MOTOR_ID, M0601_MODE_SPEED),
                      context={"prepare": "mode_right"})
        elif step == "mode_right":
            self.dual_prepared = True
            self.dual_state_var.set("命令验证完成，等待两轮新鲜反馈")

    def send_dual_target(self, left_rpm: int, right_rpm: int) -> None:
        if not self.dual_ready:
            return
        left = max(-MAX_RPM, min(MAX_RPM, int(left_rpm)))
        right = max(-MAX_RPM, min(MAX_RPM, int(right_rpm)))
        command = MotionCommand(
            msg_type=MSG_SET_DUAL_RPM,
            payload=struct.pack("<BBhhBB", LEFT_MOTOR_ID, RIGHT_MOTOR_ID,
                                left, right, 0, M0601_BRAKE_OFF),
            signature=(MSG_SET_DUAL_RPM, left, right),
            motor_id=LEFT_MOTOR_ID,
            mode=M0601_MODE_SPEED,
            active=left != 0 or right != 0,
            keepalive_type=MSG_DUAL_KEEPALIVE,
            keepalive_payload=struct.pack("<BB", LEFT_MOTOR_ID, RIGHT_MOTOR_ID),
        )
        self.motion_gate.resume()
        self._queue_motion(command)

    def send_dual_stop(self, emergency: bool) -> None:
        if self.worker is None or not self.link_ready:
            return
        self._cancel_motion_flow()
        self.pressed_directions.clear()
        self.send(MSG_STOP_DUAL,
                  struct.pack("<BBB", LEFT_MOTOR_ID, RIGHT_MOTOR_ID,
                              M0601_BRAKE_ON if emergency else M0601_BRAKE_OFF))

    def _motor_id(self) -> int:
        value = int(self.motor_id_var.get(), 0)
        if not 0 <= value <= 255 or value == M0601_RESERVED_QUERY_ID:
            raise ValueError("ID必须为0～255，且不能为0xC8")
        return value

    def _target_id_changed(self, *_args: object) -> None:
        try:
            target = self._motor_id()
        except ValueError:
            target = None
        if target != self.confirmed_id:
            self.confirmed_id = None
            self.current_mode = None
            self._cancel_motion_flow()
        self._set_controls_enabled()

    def query_target_motor(self) -> None:
        if not self.link_ready:
            return
        try:
            motor_id = self._motor_id()
        except ValueError as exc:
            messagebox.showerror("ID错误", str(exc))
            return
        self.confirmed_id = None
        self.current_mode = None
        self.single_status_var.set(f"正在查询ID {motor_id}")
        self.send(MSG_QUERY_MOTOR, struct.pack("<B", motor_id),
                  context={"id": motor_id})

    def _mode_selection_changed(self, _event: object | None = None) -> None:
        mode = MODE_BY_TEXT.get(self.mode_var.get(), M0601_MODE_SPEED)
        if mode == M0601_MODE_CURRENT:
            self.target_label_var.set("目标力矩电流 (mA，±1000)")
            self.target_var.set("0")
        elif mode == M0601_MODE_POSITION:
            self.target_label_var.set("目标位置 (0～360°)")
            self.target_var.set("0")
        else:
            self.target_label_var.set("目标速度 (RPM，±125)")
            self.target_var.set("30")

    def set_motor_mode(self) -> None:
        if not self.link_ready or self.confirmed_id is None:
            return
        try:
            motor_id = self._motor_id()
            mode = MODE_BY_TEXT[self.mode_var.get()]
        except (ValueError, KeyError) as exc:
            messagebox.showerror("模式错误", str(exc))
            return
        self._cancel_motion_flow()
        self.send_single_stop(False)
        self.root.after(150, lambda: self.send(
            MSG_SET_MODE, struct.pack("<BB", motor_id, mode),
            context={"id": motor_id, "mode": mode}))

    def send_target(self) -> None:
        if not self.link_ready or self.confirmed_id is None:
            return
        try:
            motor_id = self._motor_id()
            mode = MODE_BY_TEXT[self.mode_var.get()]
            if motor_id != self.confirmed_id or mode != self.current_mode:
                raise ValueError("目标ID或模式尚未查询确认")
            value = float(self.target_var.get())
            if mode == M0601_MODE_SPEED:
                if not -MAX_RPM <= value <= MAX_RPM:
                    raise ValueError("速度目标必须在±125RPM以内")
                target = int(round(value))
                msg_type = MSG_SET_SINGLE_RPM
                payload = struct.pack("<BhBB", motor_id, target, 0, M0601_BRAKE_OFF)
            elif mode == M0601_MODE_CURRENT:
                target = current_ma_to_raw(value)
                msg_type = MSG_SET_CURRENT
                payload = struct.pack("<BhBB", motor_id, target, 0, M0601_BRAKE_OFF)
            else:
                target = degrees_to_position_raw(value)
                msg_type = MSG_SET_POSITION
                payload = struct.pack("<BHBB", motor_id, target, 0, M0601_BRAKE_OFF)
        except (ValueError, tk.TclError) as exc:
            messagebox.showerror("目标值错误", str(exc))
            return
        self.motion_gate.resume()
        self._queue_motion(MotionCommand(
            msg_type=msg_type, payload=payload,
            signature=(msg_type, motor_id, target), motor_id=motor_id,
            mode=mode, active=mode == M0601_MODE_POSITION or target != 0))

    def send_single_stop(self, emergency: bool) -> None:
        if not self.link_ready or self.confirmed_id is None:
            return
        self._cancel_motion_flow()
        self.send(MSG_STOP, struct.pack("<BB", self.confirmed_id,
                                       M0601_BRAKE_ON if emergency else M0601_BRAKE_OFF))

    def _maintenance_changed(self) -> None:
        if self.maintenance_var.get() and not messagebox.askyesno(
                "单电机维护", "确认RS485总线上只连接了一台电机吗？"):
            self.maintenance_var.set(False)
        self._set_controls_enabled()

    def query_unique_id(self) -> None:
        if self.link_ready and self.maintenance_var.get():
            self.send(MSG_QUERY_UNIQUE_ID, context={"maintenance": True})

    def set_motor_id(self) -> None:
        if not self.link_ready or not self.maintenance_var.get() or self.confirmed_id is None:
            return
        try:
            old_id = self._motor_id()
            new_id = int(self.new_id_var.get(), 0)
            if not 0 <= new_id <= 255 or new_id == M0601_RESERVED_QUERY_ID:
                raise ValueError("新ID必须为0～255，且不能为0xC8")
        except ValueError as exc:
            messagebox.showerror("ID错误", str(exc))
            return
        if old_id != self.confirmed_id:
            messagebox.showwarning("ID不一致", "旧ID与刚查询确认的ID不一致")
            return
        if not messagebox.askyesno("确认修改ID", "确认总线仅一台电机且电机已经停止？"):
            return
        self.send_single_stop(True)
        self.root.after(350, lambda: self.send(
            MSG_SET_ID, struct.pack("<BBH", old_id, new_id, SET_ID_CONFIRM),
            context={"old_id": old_id, "new_id": new_id}))

    @staticmethod
    def _key_input_allowed(widget: tk.Misc | None) -> bool:
        if widget is None:
            return True
        return widget.winfo_class() not in {"Entry", "TEntry", "Spinbox", "TSpinbox", "TCombobox"}

    def _key_press(self, event: tk.Event) -> None:
        if not self._key_input_allowed(self.root.focus_get()):
            return
        if event.keysym in ("1", "2", "3", "4", "5"):
            self.gear_var.set(SPEED_GEARS[int(event.keysym) - 1])
            return
        if event.keysym in ("Up", "Down", "Left", "Right") and self.dual_ready:
            if event.keysym not in self.pressed_directions:
                self.pressed_directions.append(event.keysym)

    def _key_release(self, event: tk.Event) -> None:
        if event.keysym not in ("Up", "Down", "Left", "Right"):
            return
        was_active = bool(self.pressed_directions and
                          self.pressed_directions[-1] == event.keysym)
        if event.keysym in self.pressed_directions:
            self.pressed_directions.remove(event.keysym)
        if was_active and not self.pressed_directions:
            self.send_dual_stop(False)

    def _control_tick(self) -> None:
        now = time.monotonic()
        if self.motion_gate.check_timeout(now):
            self.ack_var.set("运动命令ACK超过250ms，等待下位机看门狗安全停止")
        left = right = 0
        if self.dual_ready and self.pressed_directions:
            left, right = keyboard_direction_rpm(
                self.pressed_directions[-1], int(self.gear_var.get()))
            self.send_dual_target(left, right)
        elif self.dual_ready and self.joystick.active:
            left, right = differential_rpm(
                int(round(self.joystick.x_value * 1000)),
                int(round(self.joystick.y_value * 1000)),
                int(self.gear_var.get()))
            self.send_dual_target(left, right)
        self.axis_var.set(
            f"X={self.joystick.x_value:+.3f} Y={self.joystick.y_value:+.3f} / "
            f"左={left:+d} 右={right:+d} RPM")
        if self.worker is not None and self.link_ready:
            keepalive = self.motion_gate.keepalive_command_due(now)
            if keepalive is not None:
                self.send(keepalive[0], keepalive[1], ack=False)
        if self.latest_imu is not None and now >= self.next_imu_render:
            self._render_imu()
            self.next_imu_render = now + 0.05
        self.root.after(50, self._control_tick)

    def _poll(self) -> None:
        deadline = time.monotonic() + POLL_TIME_BUDGET_S
        handled = 0
        while handled < POLL_EVENT_LIMIT and time.monotonic() < deadline:
            try:
                session_id, kind, value = self.events.get_nowait()
            except queue.Empty:
                break
            handled += 1
            if session_id != self.session_id:
                continue
            if kind == "opened":
                self._begin_handshake(value)
            elif kind == "rx":
                self._process_rx(bytes(value))
            elif kind == "tx":
                data = bytes(value)
                if len(data) < 4 or data[3] not in (MSG_CONTROL_KEEPALIVE, MSG_DUAL_KEEPALIVE):
                    self._log("TX", data.hex(" ").upper())
            elif kind == "retrying":
                self._reset_session_state()
                self.connection_var.set(f"WiFi连接中断，后台重连：{value}")
            elif kind == "error":
                self._finish_disconnect(f"通信错误：{value}")
            elif kind == "closed" and not self.disconnect_pending and self.worker is not None:
                self._finish_disconnect()
        self.root.after(20, self._poll)

    def _process_rx(self, data: bytes) -> None:
        reset_count = self.reset_detector.feed(data)
        if reset_count:
            self._reset_session_state()
            self.connection_var.set("检测到ESP32复位，等待重新握手")
            self.handshake.start(time.monotonic())
            self._schedule_handshake_tick()
        latest_chassis = None
        latest_imu = None
        latest_legacy = None
        for frame in self.parser.feed(data):
            if frame.msg_type == MSG_CHASSIS_TELEMETRY and len(frame.payload) == 56:
                latest_chassis = frame
            elif frame.msg_type == MSG_IMU_TELEMETRY and len(frame.payload) == 44:
                latest_imu = frame
            elif frame.msg_type == MSG_HEARTBEAT and len(frame.payload) == 30:
                latest_legacy = frame
            else:
                self._handle_frame(frame)
        if latest_legacy is not None:
            self._handle_legacy(latest_legacy.payload)
        if latest_chassis is not None:
            self._handle_chassis(latest_chassis.payload, latest_chassis.sequence)
        if latest_imu is not None:
            self._handle_imu(latest_imu.payload, latest_imu.sequence)

    def _handle_frame(self, frame: HostFrame) -> None:
        if frame.msg_type != MSG_ACK or len(frame.payload) != 4:
            return
        request_type, status, detail = struct.unpack("<BBh", frame.payload)
        context = self.pending_requests.pop(frame.sequence, {})
        self.ack_var.set(
            f"0x{request_type:02X} {STATUS_TEXT.get(status, status)} detail={detail}")
        self._log("ACK", self.ack_var.get())
        if (request_type == MSG_HELLO and status == 0 and not self.link_ready and
                frame.sequence == self.hello_sequence):
            self.handshake.complete()
            self.link_ready = True
            self.hello_frame = None
            endpoint = (f"{self.host_var.get()}:{self.tcp_port_var.get()}"
                        if isinstance(self.worker, TcpWorker) else self.port_var.get())
            self.connection_var.set(f"已连接并完成握手：{endpoint}")
        elif context.get("motion") is True:
            next_command = self.motion_gate.acknowledge(
                frame.sequence, status == 0, time.monotonic())
            if next_command is not None:
                self._transmit_motion(next_command)
        elif "prepare" in context:
            self._continue_prepare(str(context["prepare"]), status)
        elif request_type == MSG_QUERY_MOTOR:
            queried_id = int(context.get("id", -1))
            if status == 0:
                self.confirmed_id = queried_id
                self.single_status_var.set(f"已确认ID {queried_id}")
            else:
                self.confirmed_id = None
                self.current_mode = None
                self.single_status_var.set("查询失败")
        elif request_type == MSG_QUERY_UNIQUE_ID and context.get("maintenance") is True:
            if status == 0:
                detected = detail & 0xFF
                self.motor_id_var.set(str(detected))
                self.confirmed_id = detected
        elif request_type == MSG_SET_MODE and status == 0:
            mode = int(context.get("mode", -1))
            if mode in MODE_TEXT:
                self.current_mode = mode
                self.mode_var.set(MODE_TEXT[mode])
                self._mode_selection_changed()
        elif request_type == MSG_SET_ID and status == 0:
            detected = detail & 0xFF
            self.motor_id_var.set(str(detected))
            self.confirmed_id = detected
        if request_type in (MSG_CONTROL_KEEPALIVE, MSG_DUAL_KEEPALIVE) and status != 0:
            self.motion_gate.reject_keepalive()
        self._set_controls_enabled()

    def _handle_legacy(self, payload: bytes) -> None:
        motor_id, mode = struct.unpack_from("<BB", payload, 4)
        if motor_id == self.confirmed_id and mode in MODE_TEXT:
            self.current_mode = mode
            self.mode_var.set(MODE_TEXT[mode])
            self._set_controls_enabled()

    @staticmethod
    def _decode_motor_record(payload: bytes, offset: int) -> dict[str, int]:
        motor_id, mode, state, fault = struct.unpack_from("<BBBB", payload, offset)
        target, speed, current = struct.unpack_from("<hhh", payload, offset + 4)
        position = struct.unpack_from("<H", payload, offset + 10)[0]
        query_position = payload[offset + 12]
        age = struct.unpack_from("<H", payload, offset + 14)[0]
        feedback = struct.unpack_from("<I", payload, offset + 16)[0]
        crc_errors, timeouts = struct.unpack_from("<HH", payload, offset + 20)
        return locals()

    def _handle_chassis(self, payload: bytes, sequence: int) -> None:
        uptime, owner, flags, watchdogs = struct.unpack_from("<IBBH", payload, 0)
        left = self._decode_motor_record(payload, 8)
        right = self._decode_motor_record(payload, 32)
        now = time.monotonic()
        left_rates = self.left_rate.update(now, sequence, left["feedback"])
        right_rates = self.right_rate.update(now, sequence, right["feedback"])
        self.owner_var.set({0: "无", 1: "USB串口", 2: "WiFi TCP"}.get(owner, str(owner)))
        for side, record, rates in (("left", left, left_rates),
                                    ("right", right, right_rates)):
            values = self.wheel_vars[side]
            values["id"].set(str(record["motor_id"]))
            values["mode"].set(f"{MODE_TEXT.get(record['mode'], '未知')} (0x{record['mode']:02X})")
            values["state"].set(MOTOR_STATE_TEXT.get(record["state"], str(record["state"])))
            values["target"].set(f"{record['target']} RPM")
            values["speed"].set(f"{record['speed']} RPM（车体逻辑）")
            values["current"].set(
                f"{record['current']} raw / {current_raw_to_ma(record['current']):.1f} mA")
            values["position"].set(
                f"{record['position']} / {position_raw_to_degrees(record['position']):.2f}°")
            values["fault"].set("无" if record["fault"] == 0 else ", ".join(
                name for bit, name in enumerate(FAULT_NAMES)
                if record["fault"] & (1 << bit)))
            values["age"].set(f"{record['age']} ms")
            values["rate"].set(f"遥测{rates[0]:.1f}Hz / 反馈{rates[1]:.1f}Hz")
            values["errors"].set(
                f"CRC {record['crc_errors']} / 超时 {record['timeouts']}")
        valid = (flags & 0x18) == 0x18
        speed_modes = left["mode"] == M0601_MODE_SPEED and right["mode"] == M0601_MODE_SPEED
        fresh = left["age"] <= 500 and right["age"] <= 500
        self.dual_ready = self.dual_prepared and valid and speed_modes and fresh
        if self.dual_ready:
            self.dual_state_var.set(
                f"双轮已就绪，档位{self.gear_var.get()}RPM / 运行{uptime / 1000:.1f}s / 看门狗{watchdogs}")
        elif self.dual_prepared:
            self.dual_state_var.set("双轮命令已验证，但反馈无效、过期或模式不正确")
        if not valid or not fresh:
            self.dual_prepared = False
            self._cancel_motion_flow()
        self._set_controls_enabled()

    def _handle_imu(self, payload: bytes, sequence: int) -> None:
        timestamp = struct.unpack_from("<Q", payload, 0)[0]
        flags = payload[8]
        accel = struct.unpack_from("<fff", payload, 12)
        gyro = struct.unpack_from("<fff", payload, 24)
        samples = struct.unpack_from("<I", payload, 36)[0]
        read_errors, init_errors = struct.unpack_from("<HH", payload, 40)
        rate = self.imu_rate.update(time.monotonic(), sequence, samples)
        self.latest_imu = (timestamp, flags, accel, gyro, samples,
                           read_errors, init_errors, rate)

    def _render_imu(self) -> None:
        if self.latest_imu is None:
            return
        timestamp, flags, accel, gyro, samples, read_errors, init_errors, rate = self.latest_imu
        if not flags & 0x01:
            state = "离线，后台每秒重试"
        elif not flags & 0x02:
            state = "在线，正在静止校准"
        elif flags & 0x04:
            state = "在线且已校准"
        else:
            state = "在线但样本无效"
        self.imu_vars["state"].set(state)
        self.imu_vars["accel"].set(
            f"X={accel[0]:+.4f}  Y={accel[1]:+.4f}  Z={accel[2]:+.4f} m/s²")
        self.imu_vars["gyro"].set(
            f"X={gyro[0]:+.5f}  Y={gyro[1]:+.5f}  Z={gyro[2]:+.5f} rad/s")
        self.imu_vars["rate"].set(f"帧{rate[0]:.1f}Hz / 新样本{rate[1]:.1f}Hz（界面20Hz）")
        self.imu_vars["samples"].set(str(samples))
        self.imu_vars["errors"].set(f"读取 {read_errors} / 初始化 {init_errors}")
        self.imu_vars["time"].set(f"{timestamp / 1_000_000.0:.6f} s")

    def _log(self, direction: str, text: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", f"{time.strftime('%H:%M:%S')} {direction} {text}\n")
        self.log.see("end")
        lines = int(self.log.index("end-1c").split(".")[0])
        if lines > 500:
            self.log.delete("1.0", f"{lines - 400}.0")
        self.log.configure(state="disabled")

    def close(self) -> None:
        if self.worker is not None:
            self.send_dual_stop(True)
            self.root.after(150, self._close_now)
        else:
            self.root.destroy()

    def _close_now(self) -> None:
        self._cancel_open_timeout()
        if self.worker is not None:
            self.worker.close()
            self.worker = None
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    MotorTestApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
