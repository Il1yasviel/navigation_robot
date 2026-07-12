import unittest

from motor_test_gui import (
    FrameParser,
    HandshakeController,
    ResetDetector,
    crc16_ccitt_false,
    encode_frame,
    open_ftdi_serial,
)


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


if __name__ == "__main__":
    unittest.main()
