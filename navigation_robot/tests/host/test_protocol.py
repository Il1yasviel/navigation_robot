import unittest
import queue
import socket
import time
from pathlib import Path

from host.config import SPEED_GEARS
from host.link import HandshakeController, ResetDetector
from host.mapping import (
    control_state_flags,
    current_ma_to_raw,
    current_raw_to_ma,
    degrees_to_position_raw,
    differential_rpm,
    keyboard_direction_rpm,
    position_raw_to_degrees,
)
from host.motion import MotionCommand, MotionCommandGate
from host.protocol import FrameParser, crc16_ccitt_false, encode_frame
from host.telemetry import TelemetryRateMeter
from host.transport import SerialWorker, TcpWorker, open_ftdi_serial


class ProtocolTests(unittest.TestCase):
    def test_crc_known_vector(self):
        self.assertEqual(crc16_ccitt_false(bytes.fromhex("01 01 2A 00 00 00")), 0xBE04)

    def test_ping_golden_frame(self):
        self.assertEqual(
            encode_frame(0x01, 0x2A),
            bytes.fromhex("AA 55 01 01 2A 00 00 00 04 BE"),
        )

    def test_stop_golden_frame(self):
        self.assertEqual(
            encode_frame(0x12, 1, 1, bytes.fromhex("01 FF")),
            bytes.fromhex("AA 55 01 12 01 01 02 00 01 FF 2D 0E"),
        )

    def test_keepalive_golden_frame_without_ack_request(self):
        self.assertEqual(
            encode_frame(0x19, 0x2A, 0, bytes((1,))),
            bytes.fromhex("AA 55 01 19 2A 00 01 00 01 C2 72"),
        )

    def test_dual_motor_golden_frames(self):
        self.assertEqual(
            encode_frame(0x1A, 1, 1, bytes.fromhex("01 02 19 00 19 00 00 00")),
            bytes.fromhex(
                "AA 55 01 1A 01 01 08 00 01 02 19 00 19 00 00 00 D2 49"),
        )
        self.assertEqual(
            encode_frame(0x1B, 2, 0, bytes.fromhex("01 02")),
            bytes.fromhex("AA 55 01 1B 02 00 02 00 01 02 E2 7F"),
        )
        self.assertEqual(
            encode_frame(0x1C, 3, 1, bytes.fromhex("01 02 FF")),
            bytes.fromhex("AA 55 01 1C 03 01 03 00 01 02 FF 00 E3"),
        )

    def test_maximum_payload(self):
        parser = FrameParser()
        payload = bytes(range(128))
        frames = parser.feed(encode_frame(0x10, 7, payload=payload))
        self.assertEqual(frames[0].payload, payload)

    def test_fragmented_frame(self):
        parser = FrameParser()
        encoded = encode_frame(0x12, 1, 1, bytes.fromhex("01 FF"))
        frames = []
        for byte in encoded:
            frames.extend(parser.feed(bytes([byte])))
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].msg_type, 0x12)
        self.assertEqual(frames[0].payload, bytes.fromhex("01 FF"))

    def test_sticky_frames(self):
        parser = FrameParser()
        frames = parser.feed(encode_frame(1, 1) + encode_frame(1, 2))
        self.assertEqual([frame.sequence for frame in frames], [1, 2])

    def test_bad_crc_resynchronizes(self):
        parser = FrameParser()
        broken = bytearray(encode_frame(1, 1))
        broken[-1] ^= 0xFF
        frames = parser.feed(b"noise" + broken + encode_frame(1, 2))
        self.assertEqual(parser.crc_errors, 1)
        self.assertEqual([frame.sequence for frame in frames], [2])

    def test_rejects_oversized_length_and_recovers(self):
        parser = FrameParser()
        invalid = bytes.fromhex("AA 55 01 01 00 00 81 00")
        frames = parser.feed(invalid + encode_frame(1, 3))
        self.assertEqual(parser.length_errors, 1)
        self.assertEqual([frame.sequence for frame in frames], [3])

    def test_rom_text_and_binary_frame_resynchronize(self):
        parser = FrameParser()
        encoded = encode_frame(0x80, 4, payload=bytes.fromhex("01 00 00 00"))
        frames = parser.feed(b"ESP-ROM:esp32s3\r\n" + encoded)
        self.assertEqual([frame.sequence for frame in frames], [4])
        self.assertEqual(parser.crc_errors, 0)
        self.assertEqual(parser.length_errors, 0)

    def test_rom_text_fragmented_before_frame(self):
        parser = FrameParser()
        self.assertEqual(parser.feed(b"ESP-RO"), [])
        frames = parser.feed(b"M:boot\r\n" + encode_frame(1, 5))
        self.assertEqual([frame.sequence for frame in frames], [5])


class MotorControlUiTests(unittest.TestCase):
    @staticmethod
    def _motion(target: int) -> MotionCommand:
        return MotionCommand(
            msg_type=0x10,
            payload=bytes((1, target & 0xFF)),
            signature=(0x10, 1, target),
            motor_id=1,
            mode=2,
            active=target != 0,
        )

    def test_current_conversion_and_limit(self):
        self.assertEqual(current_ma_to_raw(1000.0), 4096)
        self.assertEqual(current_ma_to_raw(-1000.0), -4096)
        self.assertAlmostEqual(current_raw_to_ma(4096), 1000.03, places=2)
        with self.assertRaises(ValueError):
            current_ma_to_raw(1000.1)

    def test_position_conversion_and_limit(self):
        self.assertEqual(degrees_to_position_raw(0.0), 0)
        self.assertEqual(degrees_to_position_raw(90.0), 8192)
        self.assertEqual(degrees_to_position_raw(360.0), 32767)
        self.assertAlmostEqual(position_raw_to_degrees(8192), 90.0027, places=3)
        with self.assertRaises(ValueError):
            degrees_to_position_raw(360.1)

    def test_single_motor_maintenance_is_closed_by_default(self):
        flags = control_state_flags(True, True, 2, False)
        self.assertTrue(flags["motion"])
        self.assertTrue(flags["joystick"])
        self.assertFalse(flags["maintenance"])

    def test_motion_requires_confirmed_target_and_speed_joystick_requires_mode(self):
        self.assertFalse(control_state_flags(True, False, 2, False)["motion"])
        self.assertFalse(control_state_flags(True, True, 1, False)["joystick"])
        self.assertTrue(control_state_flags(True, True, 2, False)["joystick"])

    def test_gui_has_no_temperature_display(self):
        host_dir = Path(__file__).parents[2] / "host"
        for source_path in sorted(host_dir.glob("*.py")):
            source = source_path.read_text(encoding="utf-8")
            self.assertNotIn("温度", source)
            self.assertNotIn("temperature", source.lower())

    def test_five_speed_gears_and_keyboard_directions(self):
        self.assertEqual(SPEED_GEARS, (25, 50, 75, 100, 125))
        self.assertEqual(keyboard_direction_rpm("Up", 25), (25, 25))
        self.assertEqual(keyboard_direction_rpm("Down", 50), (-50, -50))
        self.assertEqual(keyboard_direction_rpm("Left", 75), (-75, 75))
        self.assertEqual(keyboard_direction_rpm("Right", 125), (125, -125))

    def test_joystick_differential_mapping_and_limit(self):
        self.assertEqual(differential_rpm(0, 1000, 125), (125, 125))
        self.assertEqual(differential_rpm(-1000, 0, 125), (-125, 125))
        self.assertEqual(differential_rpm(1000, 0, 125), (125, -125))
        self.assertEqual(differential_rpm(1000, 1000, 200), (125, 0))

    def test_unchanged_motion_sends_once_then_ten_keepalives(self):
        gate = MotionCommandGate()
        command = self._motion(30)
        self.assertEqual(gate.offer(command), command)
        gate.mark_sent(7, command, 0.0)
        self.assertIsNone(gate.acknowledge(7, True, 0.0))

        keepalives = 0
        for step in range(1, 11):
            self.assertIsNone(gate.offer(command))
            if gate.keepalive_due(step * 0.1001) == 1:
                keepalives += 1
        self.assertEqual(keepalives, 10)

    def test_fast_motion_updates_keep_only_the_latest_target(self):
        gate = MotionCommandGate()
        first = self._motion(10)
        middle = self._motion(20)
        latest = self._motion(30)
        gate.mark_sent(1, first, 0.0)
        self.assertIsNone(gate.offer(middle))
        self.assertIsNone(gate.offer(latest))
        self.assertEqual(gate.inflight_sequence, 1)
        self.assertEqual(gate.pending_command, latest)
        self.assertEqual(gate.acknowledge(1, True, 0.01), latest)

    def test_motion_ack_timeout_blocks_retries_until_resumed(self):
        gate = MotionCommandGate()
        command = self._motion(30)
        gate.mark_sent(1, command, 0.0)
        self.assertFalse(gate.check_timeout(0.249))
        self.assertTrue(gate.check_timeout(0.251))
        self.assertIsNone(gate.offer(command))
        gate.resume()
        self.assertEqual(gate.offer(command), command)

    def test_reset_cancels_inflight_pending_and_keepalive(self):
        gate = MotionCommandGate()
        first = self._motion(10)
        latest = self._motion(30)
        gate.mark_sent(1, first, 0.0)
        gate.offer(latest)
        gate.reset()
        self.assertIsNone(gate.inflight_command)
        self.assertIsNone(gate.pending_command)
        self.assertIsNone(gate.keepalive_due(1.0))

    def test_telemetry_rate_meter_reports_ten_hertz(self):
        meter = TelemetryRateMeter()
        meter.update(0.0, 0, 100)
        heartbeat_hz = feedback_hz = 0.0
        for step in range(1, 21):
            heartbeat_hz, feedback_hz = meter.update(
                step * 0.1, step & 0xFF, 100 + step)
        self.assertAlmostEqual(heartbeat_hz, 10.0, places=2)
        self.assertAlmostEqual(feedback_hz, 10.0, places=2)


class FirmwareSourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = Path(__file__).parents[2]

    def test_keepalive_has_no_m0601_bus_transaction(self):
        source = self.root.joinpath(
            "components/services/src/motor_service.c").read_text(encoding="utf-8")
        start = source.index("m0601_status_t motor_service_keepalive")
        end = source.index("void motor_service_note_watchdog_stop", start)
        keepalive = source[start:end]
        self.assertNotIn("m0601_drive_", keepalive)
        self.assertNotIn("m0601_query", keepalive)

    def test_keepalive_and_ack_flag_are_dispatched(self):
        messages = self.root.joinpath(
            "components/protocols/include/protocols/host_messages.h").read_text(
                encoding="utf-8")
        service = self.root.joinpath(
            "components/services/src/host_link_service.c").read_text(encoding="utf-8")
        self.assertIn("HOST_MSG_CONTROL_KEEPALIVE 0x19u", messages)
        self.assertIn("case HOST_MSG_CONTROL_KEEPALIVE:", service)
        self.assertIn("frame->flags & HOST_FLAG_ACK_REQUIRED", service)
        self.assertIn("status != HOST_STATUS_OK", service)

    def test_dual_protocol_wifi_and_imu_are_wired(self):
        messages = self.root.joinpath(
            "components/protocols/include/protocols/host_messages.h").read_text(
                encoding="utf-8")
        host = self.root.joinpath(
            "components/services/src/host_link_service.c").read_text(encoding="utf-8")
        motor = self.root.joinpath(
            "components/services/src/motor_service.c").read_text(encoding="utf-8")
        self.assertIn("HOST_MSG_SET_DUAL_RPM       0x1Au", messages)
        self.assertIn("HOST_MSG_CHASSIS_TELEMETRY 0x92u", messages)
        self.assertIn("HOST_MSG_IMU_TELEMETRY     0x93u", messages)
        self.assertIn("esp32_wifi_tcp_start", host)
        self.assertIn("bmi088_service_get_snapshot", host)
        self.assertIn("request->right_target_value * ROBOT_RIGHT_DIRECTION", motor)
        config = self.root.joinpath(
            "components/config/Kconfig.projbuild").read_text(encoding="utf-8")
        self.assertIn('default 125', config)

    def test_motor_queries_use_fixed_deadlines(self):
        source = self.root.joinpath(
            "components/services/src/motor_service.c").read_text(encoding="utf-8")
        self.assertIn("next_query_ms += ROBOT_MOTOR_QUERY_MS", source)
        self.assertNotIn("last_query_ms = now", source)


class ResetDetectorTests(unittest.TestCase):
    def test_detects_fragmented_rom_marker_and_repeated_reset(self):
        detector = ResetDetector()
        self.assertEqual(detector.feed(b"ESP-", now=1.0), 0)
        self.assertEqual(detector.feed(b"ROM:esp32s3", now=1.1), 1)
        self.assertEqual(detector.feed(b"ESP-ROM:esp32s3", now=3.0), 2)
        self.assertEqual(detector.feed(b"ordinary binary", now=9.0), 0)
        self.assertEqual(detector.reset_times, [])


class HandshakeTests(unittest.TestCase):
    def test_initial_delay_retry_and_timeout(self):
        handshake = HandshakeController()
        handshake.start(10.0)
        self.assertEqual(handshake.poll(10.99), HandshakeController.WAIT)
        self.assertEqual(handshake.poll(11.0), HandshakeController.SEND)
        self.assertEqual(handshake.poll(11.29), HandshakeController.WAIT)
        self.assertEqual(handshake.poll(11.3), HandshakeController.SEND)
        self.assertEqual(handshake.poll(12.5), HandshakeController.TIMEOUT)
        self.assertEqual(handshake.poll(12.6), HandshakeController.WAIT)

    def test_complete_stops_retries(self):
        handshake = HandshakeController()
        handshake.start(0.0)
        handshake.complete()
        self.assertEqual(handshake.poll(2.0), HandshakeController.WAIT)


class SerialOpenTests(unittest.TestCase):
    def test_dtr_and_rts_are_released_before_open(self):
        events = []

        class FakeSerial:
            def __init__(self, **kwargs):
                events.append(("create", kwargs))
                self.is_open = False

            @property
            def dtr(self):
                return False

            @dtr.setter
            def dtr(self, value):
                events.append(("dtr", value))

            @property
            def rts(self):
                return False

            @rts.setter
            def rts(self, value):
                events.append(("rts", value))

            @property
            def port(self):
                return None

            @port.setter
            def port(self, value):
                events.append(("port", value))

            def open(self):
                events.append(("open", None))
                self.is_open = True

            def reset_input_buffer(self):
                events.append(("flush", None))

        connection = open_ftdi_serial("COM9", FakeSerial)
        names = [item[0] for item in events]
        self.assertTrue(connection.is_open)
        self.assertEqual(events[0][1]["port"], None)
        self.assertLess(names.index("dtr"), names.index("open"))
        self.assertLess(names.index("rts"), names.index("open"))
        self.assertLess(names.index("port"), names.index("open"))

    def test_worker_open_is_async(self):
        events = queue.Queue()

        class SlowSerial:
            def __init__(self, **_kwargs):
                self.is_open = False

            @property
            def dtr(self):
                return False

            @dtr.setter
            def dtr(self, _value):
                pass

            @property
            def rts(self):
                return False

            @rts.setter
            def rts(self, _value):
                pass

            @property
            def port(self):
                return None

            @port.setter
            def port(self, _value):
                pass

            def open(self):
                time.sleep(0.2)
                self.is_open = True

            def reset_input_buffer(self):
                pass

            def read(self, _size):
                time.sleep(0.01)
                return b""

            def close(self):
                self.is_open = False

        start = time.monotonic()
        worker = SerialWorker("COM9", 7, events, serial_factory=SlowSerial)
        elapsed = time.monotonic() - start
        self.assertLess(elapsed, 0.1)
        session_id, kind, value = events.get(timeout=1.0)
        self.assertEqual((session_id, kind, value), (7, "opened", "COM9"))
        worker.close()
        worker.thread.join(timeout=1.0)
        self.assertFalse(worker.thread.is_alive())

    def test_worker_purges_startup_backlog_before_opened(self):
        events = queue.Queue()

        class BacklogSerial:
            def __init__(self, **_kwargs):
                self.is_open = False
                self.reads = [b"old-1", b"old-2", b""]

            @property
            def dtr(self):
                return False

            @dtr.setter
            def dtr(self, _value):
                pass

            @property
            def rts(self):
                return False

            @rts.setter
            def rts(self, _value):
                pass

            @property
            def port(self):
                return None

            @port.setter
            def port(self, _value):
                pass

            def open(self):
                self.is_open = True

            def reset_input_buffer(self):
                pass

            def read(self, _size):
                if self.reads:
                    return self.reads.pop(0)
                time.sleep(0.01)
                return b""

            def close(self):
                self.is_open = False

        worker = SerialWorker("COM9", 9, events, serial_factory=BacklogSerial)
        self.assertEqual(events.get(timeout=1.0), (9, "opened", "COM9"))
        with self.assertRaises(queue.Empty):
            while True:
                item = events.get_nowait()
                if item[1] == "rx":
                    self.fail("startup backlog should not be emitted as rx events")
        worker.close()
        worker.thread.join(timeout=1.0)
        self.assertFalse(worker.thread.is_alive())


class TcpWorkerTests(unittest.TestCase):
    def test_tcp_open_send_and_close_are_async(self):
        events = queue.Queue()

        class FakeSocket:
            def __init__(self):
                self.closed = False
                self.sent = []

            def settimeout(self, _timeout):
                pass

            def sendall(self, data):
                self.sent.append(data)

            def recv(self, _size):
                if self.closed:
                    return b""
                time.sleep(0.01)
                raise socket.timeout

            def shutdown(self, _how):
                self.closed = True

            def close(self):
                self.closed = True

        fake = FakeSocket()

        def factory(address, timeout):
            self.assertEqual(address, ("robot.local", 3333))
            self.assertEqual(timeout, 2.0)
            return fake

        start = time.monotonic()
        worker = TcpWorker("robot.local", 3333, 11, events, socket_factory=factory)
        self.assertLess(time.monotonic() - start, 0.1)
        self.assertEqual(events.get(timeout=1.0),
                         (11, "opened", "robot.local:3333"))
        worker.send(b"frame")
        deadline = time.monotonic() + 1.0
        while not fake.sent and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertEqual(fake.sent, [b"frame"])
        worker.close()
        worker.thread.join(timeout=1.0)
        self.assertFalse(worker.thread.is_alive())


if __name__ == "__main__":
    unittest.main()
