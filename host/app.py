"""MotorTestApp 主界面与程序入口。"""
from __future__ import annotations

import json
import queue
import struct
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import messagebox, ttk
from tkinter.scrolledtext import ScrolledText

from host.config import (FAULT_NAMES, LEFT_MOTOR_ID, MAX_RPM, MODE_BY_TEXT,
                         MODE_TEXT, MOTOR_STATE_TEXT, M0601_BRAKE_OFF,
                         M0601_BRAKE_ON, M0601_MODE_CURRENT,
                         M0601_MODE_POSITION, M0601_MODE_SPEED,
                         M0601_RESERVED_QUERY_ID, OPEN_TIMEOUT_S,
                         POLL_EVENT_LIMIT, POLL_TIME_BUDGET_S, RIGHT_MOTOR_ID,
                         SET_ID_CONFIRM, SPEED_GEARS, STATUS_TEXT)
from host.joystick import VirtualJoystick
from host.link import HandshakeController, ResetDetector
from host.mapping import (control_state_flags, current_ma_to_raw,
                          current_raw_to_ma, degrees_to_position_raw,
                          differential_rpm, keyboard_direction_rpm,
                          position_raw_to_degrees)
from host.motion import MotionCommand, MotionCommandGate
from host.protocol import (FLAG_ACK_REQUIRED, MSG_ACK, MSG_CHASSIS_TELEMETRY,
                           MSG_CONTROL_KEEPALIVE, MSG_DUAL_KEEPALIVE,
                           MSG_HEARTBEAT, MSG_HELLO, MSG_IMU_TELEMETRY,
                           MSG_QUERY_MOTOR, MSG_QUERY_UNIQUE_ID,
                           MSG_SET_CURRENT, MSG_SET_DUAL_RPM, MSG_SET_ID,
                           MSG_SET_MODE, MSG_SET_POSITION, MSG_SET_SINGLE_RPM,
                           MSG_STOP, MSG_STOP_DUAL, FrameParser, HostFrame,
                           encode_frame)
from host.telemetry import (TelemetryRateMeter, decode_chassis_telemetry,
                            decode_imu_telemetry)
from host.transport import SerialWorker, TcpWorker, list_ports


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
        self.latest_chassis_data: dict[str, object] | None = None
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
        ttk.Button(top, text="诊断快照", command=self._take_snapshot).pack(side="right", padx=6)

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

    def _handle_chassis(self, payload: bytes, sequence: int) -> None:
        uptime, owner, flags, watchdogs, left, right = decode_chassis_telemetry(payload)
        now = time.monotonic()
        left_rates = self.left_rate.update(now, sequence, left["feedback"])
        right_rates = self.right_rate.update(now, sequence, right["feedback"])
        self.latest_chassis_data = {
            "uptime_ms": uptime,
            "owner": owner,
            "flags": flags,
            "watchdog_stops": watchdogs,
            "left": {**left, "telemetry_hz": left_rates[0], "feedback_hz": left_rates[1]},
            "right": {**right, "telemetry_hz": right_rates[0], "feedback_hz": right_rates[1]},
        }
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
        timestamp, flags, accel, gyro, samples, read_errors, init_errors = (
            decode_imu_telemetry(payload))
        rate = self.imu_rate.update(time.monotonic(), sequence, samples)
        self.latest_imu = (timestamp, flags, accel, gyro, samples,
                           read_errors, init_errors, rate)

    def _take_snapshot(self) -> None:
        """把当前连接、底盘、IMU 的全部最新状态写入 logs/ 下的 JSON 文件。"""
        try:
            path = self._write_snapshot()
        except Exception as exc:
            messagebox.showerror("诊断快照", f"保存失败：{type(exc).__name__}: {exc}")
            return
        messagebox.showinfo("诊断快照", f"已保存：{path}")

    def _write_snapshot(self) -> Path:
        stamp = datetime.now()
        directory = Path(__file__).resolve().parents[1] / "logs"
        directory.mkdir(exist_ok=True)
        path = directory / f"snapshot_{stamp:%Y%m%d_%H%M%S}.json"
        imu_data: dict[str, object] | None = None
        if self.latest_imu is not None:
            timestamp, flags, accel, gyro, samples, read_errors, init_errors, rate = (
                self.latest_imu)
            imu_data = {
                "timestamp_us": timestamp,
                "online": bool(flags & 0x01),
                "calibrated": bool(flags & 0x02),
                "sample_valid": bool(flags & 0x04),
                "accel_mps2": list(accel),
                "gyro_rads": list(gyro),
                "samples": samples,
                "read_errors": read_errors,
                "init_errors": init_errors,
                "frame_hz": rate[0],
                "sample_hz": rate[1],
            }
        payload = {
            "created": stamp.isoformat(),
            "connection": {
                "type": self.connection_type_var.get(),
                "endpoint": (f"{self.host_var.get()}:{self.tcp_port_var.get()}"
                             if self.connection_type_var.get() == "WiFi TCP"
                             else self.port_var.get()),
                "status": self.connection_var.get(),
                "link_ready": self.link_ready,
            },
            "latest_ack": self.ack_var.get(),
            "control_owner": self.owner_var.get(),
            "dual_prepared": self.dual_prepared,
            "dual_ready": self.dual_ready,
            "chassis": self.latest_chassis_data,
            "imu": imu_data,
        }
        path.write_text(json.dumps(payload, indent=2, ensure_ascii=False),
                        encoding="utf-8")
        return path

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
