"""ESP32-S3/M0601 single-wheel FTDI UART test GUI.

The FTDI COM port is an exclusive binary channel at runtime. Close idf.py and
any serial terminal before connecting this application.
"""

from __future__ import annotations

import queue
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
except ImportError:  # Protocol tests can run without pyserial.
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
MSG_ACK = 0x80
MSG_HEARTBEAT = 0x90

M0601_BRAKE_OFF = 0x00
M0601_BRAKE_ON = 0xFF
M0601_RESERVED_QUERY_ID = 0xC8
SET_ID_CONFIRM = 0x4D36

HANDSHAKE_INITIAL_DELAY_S = 1.0
HANDSHAKE_RETRY_S = 0.3
HANDSHAKE_TIMEOUT_S = 2.5
RESET_WINDOW_S = 5.0
RESET_MARKER = b"ESP-ROM:"
OPEN_TIMEOUT_S = 3.0
STARTUP_PURGE_MAX_S = 1.5
STARTUP_PURGE_QUIET_READS = 3
POLL_EVENT_LIMIT = 32
POLL_TIME_BUDGET_S = 0.008

STATUS_TEXT = {
    0: "成功",
    1: "帧 CRC 错误",
    2: "长度错误",
    3: "参数越界",
    4: "设备忙",
    5: "电机响应超时",
    6: "电机 CRC 错误",
    7: "前置条件不满足",
    8: "不支持的命令",
    9: "I/O 错误",
}

MOTOR_STATE_TEXT = {
    0: "离线",
    1: "空闲",
    2: "运行",
    3: "故障",
    4: "急停",
}

FAULT_NAMES = (
    "传感器",
    "过流",
    "相线过流",
    "堵转",
    "过温",
    "保留5",
    "保留6",
    "保留7",
)


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(msg_type: int, sequence: int, flags: int = 0, payload: bytes = b"") -> bytes:
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
        raise RuntimeError("缺少 pyserial，请运行 pip install -r requirements-host.txt")
    factory = serial.Serial if serial_factory is None else serial_factory
    connection = factory(port=None,
                         baudrate=115200,
                         timeout=0.03,
                         write_timeout=0.2,
                         rtscts=False,
                         dsrdtr=False)
    connection.dtr = False
    connection.rts = False
    connection.port = port
    connection.open()
    connection.reset_input_buffer()
    return connection


class SerialWorker:
    def __init__(self, port: str,
                 session_id: int,
                 events: queue.Queue[tuple[int, str, object]],
                 serial_factory=None) -> None:
        self.port = port
        self.session_id = session_id
        self.events = events
        self.tx: queue.Queue[bytes] = queue.Queue()
        self.stop_event = threading.Event()
        self.serial_factory = serial_factory
        self.connection = None
        self.thread = threading.Thread(target=self._run, name="motor-host-uart", daemon=True)
        self.thread.start()

    def send(self, data: bytes) -> None:
        if not self.stop_event.is_set():
            self.tx.put(data)

    def _emit(self, kind: str, value: object) -> None:
        self.events.put((self.session_id, kind, value))

    def _purge_startup_backlog(self) -> None:
        assert self.connection is not None
        deadline = time.monotonic() + STARTUP_PURGE_MAX_S
        quiet_reads = 0
        while (
            not self.stop_event.is_set()
            and time.monotonic() < deadline
            and quiet_reads < STARTUP_PURGE_QUIET_READS
        ):
            self.connection.reset_input_buffer()
            discarded = self.connection.read(256)
            if discarded:
                quiet_reads = 0
            else:
                quiet_reads += 1
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
        except Exception as exc:  # Serial failures must reach the Tk thread.
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
        if self.connection.is_open:
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
        self.x_value = dx / self.radius
        self.y_value = -dy / self.radius  # Up is positive vehicle speed.
        if abs(self.x_value) < 0.08:
            self.x_value = 0.0
        if abs(self.y_value) < 0.08:
            self.y_value = 0.0
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
        self.root.title("ESP32-S3 M0601 单轮测试")
        self.root.geometry("980x720")
        self.events: queue.Queue[tuple[int, str, object]] = queue.Queue()
        self.parser = FrameParser()
        self.worker: SerialWorker | None = None
        self.sequence = 0
        self.session_id = 0
        self.hello_sequence: int | None = None
        self.hello_frame: bytes | None = None
        self.confirmed_id: int | None = None
        self.link_ready = False
        self.disconnect_pending = False
        self.handshake = HandshakeController()
        self.handshake_job: str | None = None
        self.open_timeout_job: str | None = None
        self.reset_detector = ResetDetector()

        self.port_var = tk.StringVar()
        self.connection_var = tk.StringVar(value="未连接")
        self.motor_id_var = tk.StringVar(value="1")
        self.new_id_var = tk.StringVar(value="2")
        self.max_rpm_var = tk.IntVar(value=30)
        self.reverse_var = tk.BooleanVar(value=False)
        self.axis_var = tk.StringVar(value="X=0.000  Y=0.000")
        self.status_vars = {name: tk.StringVar(value="--") for name in (
            "state", "mode", "target", "speed", "current", "position", "temperature",
            "fault", "age", "feedback", "errors", "uptime", "ack")}

        self._build_ui()
        self._set_controls_enabled(False)
        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(20, self._poll)
        self.root.after(50, self._control_tick)

    def _build_ui(self) -> None:
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill="x")
        ttk.Label(top, text="USB串口").pack(side="left")
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=24, state="readonly")
        self.port_box.pack(side="left", padx=6)
        ttk.Button(top, text="刷新", command=self.refresh_ports).pack(side="left")
        self.connect_button = ttk.Button(top, text="连接", command=self.toggle_connection)
        self.connect_button.pack(side="left", padx=8)
        ttk.Label(top, textvariable=self.connection_var).pack(side="left", padx=8)
        self.emergency_button = ttk.Button(
            top, text="急停", command=lambda: self.send_stop(True))
        self.emergency_button.pack(side="right")

        body = ttk.Frame(self.root, padding=(10, 0, 10, 8))
        body.pack(fill="both", expand=True)
        left = ttk.LabelFrame(body, text="单轮摇杆（Y轴控制，X轴预留）", padding=10)
        left.pack(side="left", fill="y")
        self.joystick = VirtualJoystick(left, lambda: self.send_stop(False))
        self.joystick.pack()
        ttk.Label(left, textvariable=self.axis_var).pack(pady=5)
        speed_row = ttk.Frame(left)
        speed_row.pack(fill="x", pady=4)
        ttk.Label(speed_row, text="最大RPM").pack(side="left")
        ttk.Spinbox(speed_row, from_=1, to=60, textvariable=self.max_rpm_var, width=8).pack(side="left", padx=8)
        ttk.Checkbutton(left, text="反转方向", variable=self.reverse_var).pack(anchor="w")

        id_box = ttk.LabelFrame(left, text="电机ID", padding=8)
        id_box.pack(fill="x", pady=12)
        ttk.Label(id_box, text="当前/期望").grid(row=0, column=0, sticky="w")
        ttk.Entry(id_box, textvariable=self.motor_id_var, width=8).grid(row=0, column=1, padx=5)
        self.query_button = ttk.Button(
            id_box, text="查询唯一ID", command=self.query_unique_id)
        self.query_button.grid(row=0, column=2)
        ttk.Label(id_box, text="新ID").grid(row=1, column=0, sticky="w", pady=5)
        ttk.Entry(id_box, textvariable=self.new_id_var, width=8).grid(row=1, column=1, padx=5)
        self.set_id_button = ttk.Button(id_box, text="修改ID", command=self.set_motor_id)
        self.set_id_button.grid(row=1, column=2)

        right = ttk.Frame(body)
        right.pack(side="left", fill="both", expand=True, padx=(12, 0))
        telemetry = ttk.LabelFrame(right, text="实时心跳", padding=10)
        telemetry.pack(fill="x")
        labels = (
            ("状态", "state"), ("模式", "mode"), ("目标RPM", "target"),
            ("实际RPM", "speed"), ("电流", "current"), ("位置", "position"),
            ("温度原始值", "temperature"), ("故障", "fault"),
            ("反馈年龄", "age"), ("有效反馈", "feedback"),
            ("错误计数", "errors"), ("MCU运行时间", "uptime"), ("最近ACK", "ack"),
        )
        for row, (title, key) in enumerate(labels):
            ttk.Label(telemetry, text=title, width=14).grid(row=row, column=0, sticky="w", pady=2)
            ttk.Label(telemetry, textvariable=self.status_vars[key]).grid(row=row, column=1, sticky="w", pady=2)

        log_box = ttk.LabelFrame(right, text="十六进制收发", padding=5)
        log_box.pack(fill="both", expand=True, pady=(8, 0))
        self.log = ScrolledText(log_box, height=14, font=("Consolas", 9), state="disabled")
        self.log.pack(fill="both", expand=True)

    def refresh_ports(self) -> None:
        port_infos = [] if list_ports is None else list(list_ports.comports())
        ports = [item.device for item in port_infos]
        self.port_box["values"] = ports
        if ports and self.port_var.get() not in ports:
            ftdi = next((item.device for item in port_infos
                         if item.vid == 0x0403 and item.pid == 0x6001), None)
            self.port_var.set(ftdi or ports[0])

    def _set_controls_enabled(self, enabled: bool) -> None:
        state = "normal" if enabled else "disabled"
        self.joystick.configure(state=state)
        self.query_button.configure(state=state)
        self.set_id_button.configure(state=state)
        self.emergency_button.configure(state=state)

    def toggle_connection(self) -> None:
        if self.worker is None:
            if not self.port_var.get():
                messagebox.showwarning("串口", "未找到可用串口")
                return
            self.session_id += 1
            self.worker = SerialWorker(self.port_var.get(), self.session_id, self.events)
            self.parser = FrameParser()
            self.reset_detector = ResetDetector()
            self.link_ready = False
            self.confirmed_id = None
            self.hello_sequence = None
            self.hello_frame = None
            self._set_controls_enabled(False)
            self.connection_var.set(f"正在打开串口 {self.port_var.get()}")
            self.connect_button.configure(text="断开")
            self._schedule_open_timeout()
        else:
            self._schedule_disconnect()

    def _schedule_open_timeout(self) -> None:
        if self.open_timeout_job is None:
            self.open_timeout_job = self.root.after(
                int(OPEN_TIMEOUT_S * 1000), self._handle_open_timeout)

    def _cancel_open_timeout(self) -> None:
        if self.open_timeout_job is not None:
            self.root.after_cancel(self.open_timeout_job)
            self.open_timeout_job = None

    def _handle_open_timeout(self) -> None:
        self.open_timeout_job = None
        if self.worker is None or self.link_ready:
            return
        messagebox.showerror("连接超时", f"串口 {self.port_var.get()} 打开或初始化超时")
        self._finish_disconnect("连接超时")

    def _schedule_handshake_tick(self) -> None:
        if self.handshake_job is None:
            self.handshake_job = self.root.after(50, self._handshake_tick)

    def _handshake_tick(self) -> None:
        self.handshake_job = None
        if self.worker is None or self.link_ready:
            return
        action = self.handshake.poll(time.monotonic())
        if action == HandshakeController.SEND:
            self.connection_var.set(f"正在握手 {self.port_var.get()}")
            self._send_hello()
        elif action == HandshakeController.TIMEOUT:
            messagebox.showerror(
                "握手失败",
                "串口能够打开，但ESP32应用程序没有返回HELLO ACK。\n"
                "请确认已烧录最新固件，并关闭烧录器或其他串口工具。")
            self._finish_disconnect("握手失败：ESP32应用程序无响应")
            return
        self._schedule_handshake_tick()

    def _restart_handshake_after_reset(self, repeated_resets: int) -> None:
        if self.worker is None:
            return
        self.link_ready = False
        self.confirmed_id = None
        self.hello_sequence = None
        self.hello_frame = None
        self._set_controls_enabled(False)
        self.handshake.start(time.monotonic())
        if repeated_resets >= 2:
            self.connection_var.set("ESP32连续复位：检查DTR/RTS和供电")
        else:
            self.connection_var.set("检测到ESP32复位，等待重新握手")
        self._schedule_handshake_tick()

    def _schedule_disconnect(self) -> None:
        if self.worker is None or self.disconnect_pending:
            return
        self.disconnect_pending = True
        if self.link_ready:
            self.send_stop(True)
            self.root.after(120, self._finish_disconnect)
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
        self.link_ready = False
        self.disconnect_pending = False
        self.confirmed_id = None
        self.hello_sequence = None
        self.hello_frame = None
        self._set_controls_enabled(False)
        self.connection_var.set(status)
        self.connect_button.configure(text="连接")

    def send(self, msg_type: int, payload: bytes = b"", ack: bool = True) -> None:
        if self.worker is None:
            return
        frame = encode_frame(msg_type, self.sequence,
                             FLAG_ACK_REQUIRED if ack else 0, payload)
        self.sequence = (self.sequence + 1) & 0xFF
        self.worker.send(frame)

    def _send_hello(self) -> None:
        if self.worker is None:
            return
        if self.hello_frame is None:
            self.hello_sequence = self.sequence
            self.hello_frame = encode_frame(
                MSG_HELLO, self.sequence, FLAG_ACK_REQUIRED)
            self.sequence = (self.sequence + 1) & 0xFF
        self.worker.send(self.hello_frame)

    def _motor_id(self) -> int:
        value = int(self.motor_id_var.get(), 0)
        if not 0 <= value <= 255 or value == M0601_RESERVED_QUERY_ID:
            raise ValueError("ID必须为0～255，且不能为0xC8")
        return value

    def send_stop(self, emergency: bool) -> None:
        if self.worker is None or not self.link_ready:
            return
        try:
            payload = struct.pack("<BB", self._motor_id(),
                                  M0601_BRAKE_ON if emergency else M0601_BRAKE_OFF)
        except ValueError as exc:
            messagebox.showerror("ID错误", str(exc))
            return
        self.send(MSG_STOP, payload)

    def query_unique_id(self) -> None:
        if self.link_ready:
            self.send(MSG_QUERY_UNIQUE_ID)

    def set_motor_id(self) -> None:
        if not self.link_ready or self.confirmed_id is None:
            messagebox.showwarning("禁止修改", "请先查询并确认总线上唯一电机的ID")
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
            messagebox.showwarning("ID不一致", "输入的旧ID与刚查询到的ID不一致")
            return
        if not messagebox.askyesno("确认修改ID", "确认RS485总线上只连接了一台电机，并且电机已经停止？"):
            return
        self.send_stop(True)
        self.root.after(350, lambda: self.send(
            MSG_SET_ID, struct.pack("<BBH", old_id, new_id, SET_ID_CONFIRM)))

    def _control_tick(self) -> None:
        x_value, y_value = self.joystick.x_value, self.joystick.y_value
        self.axis_var.set(f"X={x_value:+.3f}  Y={y_value:+.3f}")
        if self.worker is not None and self.link_ready and self.joystick.active:
            try:
                motor_id = self._motor_id()
                max_rpm = max(1, min(60, int(self.max_rpm_var.get())))
                y_permille = int(round(y_value * 1000))
                if self.reverse_var.get():
                    y_permille = -y_permille
                payload = struct.pack("<BhhHB", motor_id,
                                      int(round(x_value * 1000)),
                                      y_permille, max_rpm, 1)
                self.send(MSG_JOYSTICK, payload)
            except (ValueError, tk.TclError):
                pass
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
                self._cancel_open_timeout()
                self.connection_var.set(f"串口已打开，等待ESP32启动 {value}")
                self.handshake.start(time.monotonic())
                self._schedule_handshake_tick()
            elif kind == "rx":
                self._process_rx(bytes(value))
            elif kind == "tx":
                self._log("TX", bytes(value))
            elif kind == "error":
                self._finish_disconnect(f"通信错误: {value}")
            elif kind == "closed" and not self.disconnect_pending and self.worker is not None:
                self._finish_disconnect()
        self.root.after(20, self._poll)

    def _process_rx(self, data: bytes) -> None:
        self._log("RX", data)
        reset_count = self.reset_detector.feed(data)
        if reset_count:
            self._restart_handshake_after_reset(reset_count)
        latest_heartbeat = None
        for frame in self.parser.feed(data):
            if frame.msg_type == MSG_HEARTBEAT and len(frame.payload) == 30:
                latest_heartbeat = frame
            else:
                self._handle_frame(frame)
        if latest_heartbeat is not None:
            self._handle_frame(latest_heartbeat)

    def _handle_frame(self, frame: HostFrame) -> None:
        if frame.msg_type == MSG_ACK and len(frame.payload) == 4:
            request_type, status, detail = struct.unpack("<BBh", frame.payload)
            self.status_vars["ack"].set(
                f"命令0x{request_type:02X}: {STATUS_TEXT.get(status, status)}，detail={detail}")
            if (request_type == MSG_HELLO and status == 0 and not self.link_ready and
                    frame.sequence == self.hello_sequence):
                self.handshake.complete()
                self.link_ready = True
                self.hello_frame = None
                self.connection_var.set(f"已连接并完成握手 {self.port_var.get()}")
                self._set_controls_enabled(True)
                self.root.after(100, self.query_unique_id)
            elif request_type == MSG_QUERY_UNIQUE_ID and status == 0:
                self.confirmed_id = detail & 0xFF
                self.motor_id_var.set(str(self.confirmed_id))
            elif request_type == MSG_QUERY_UNIQUE_ID:
                self.confirmed_id = None
            elif request_type == MSG_SET_ID and status == 0:
                self.confirmed_id = detail & 0xFF
                self.motor_id_var.set(str(self.confirmed_id))
        elif frame.msg_type == MSG_HEARTBEAT and len(frame.payload) == 30:
            self._handle_heartbeat(frame.payload)

    def _handle_heartbeat(self, payload: bytes) -> None:
        uptime, = struct.unpack_from("<I", payload, 0)
        motor_id, mode, state, fault = struct.unpack_from("<BBBB", payload, 4)
        target, speed, current = struct.unpack_from("<hhh", payload, 8)
        drive_position, = struct.unpack_from("<H", payload, 14)
        query_position, temperature = struct.unpack_from("<BB", payload, 16)
        age, = struct.unpack_from("<H", payload, 18)
        feedback, = struct.unpack_from("<I", payload, 20)
        crc_errors, timeouts, watchdogs = struct.unpack_from("<HHH", payload, 24)
        current_ma = current * 8000.0 / 32767.0
        position_deg = drive_position * 360.0 / 32768.0
        faults = "无" if fault == 0 else ", ".join(
            name for bit, name in enumerate(FAULT_NAMES) if fault & (1 << bit))
        self.status_vars["state"].set(f"{MOTOR_STATE_TEXT.get(state, state)} / ID {motor_id}")
        self.status_vars["mode"].set(str(mode))
        self.status_vars["target"].set(str(target))
        self.status_vars["speed"].set(str(speed))
        self.status_vars["current"].set(f"{current} raw / {current_ma:.1f} mA")
        self.status_vars["position"].set(
            f"{drive_position} raw / {position_deg:.2f}°（查询值 {query_position}）")
        self.status_vars["temperature"].set(f"{temperature}（仅供参考）")
        self.status_vars["fault"].set(faults)
        self.status_vars["age"].set(f"{age} ms")
        self.status_vars["feedback"].set(str(feedback))
        self.status_vars["errors"].set(
            f"CRC {crc_errors} / 超时 {timeouts} / 看门狗 {watchdogs}")
        self.status_vars["uptime"].set(f"{uptime / 1000.0:.1f} s")

    def _log(self, direction: str, data: bytes) -> None:
        self.log.configure(state="normal")
        timestamp = time.strftime("%H:%M:%S")
        self.log.insert("end", f"{timestamp} {direction} {data.hex(' ').upper()}\n")
        self.log.see("end")
        line_count = int(self.log.index("end-1c").split(".")[0])
        if line_count > 500:
            self.log.delete("1.0", f"{line_count - 400}.0")
        self.log.configure(state="disabled")

    def close(self) -> None:
        if self.worker is not None:
            self.send_stop(True)
            self.root.after(120, self._close_now)
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
