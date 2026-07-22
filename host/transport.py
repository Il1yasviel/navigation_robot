"""FTDI 串口打开助手与串口/TCP 收发工作线程。"""
from __future__ import annotations

import queue
import socket
import threading
import time
from typing import Callable

from host.config import STARTUP_PURGE_MAX_S, STARTUP_PURGE_QUIET_READS

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # Unit tests can run without pyserial.
    serial = None
    list_ports = None


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
