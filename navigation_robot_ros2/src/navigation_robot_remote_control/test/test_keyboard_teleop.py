from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus

from navigation_robot_remote_control.keyboard_teleop import base_status_ready
from navigation_robot_remote_control.keyboard_teleop import command_must_stop
from navigation_robot_remote_control.keyboard_teleop import command_for_key
from navigation_robot_remote_control.keyboard_teleop import diagnostic_level_value
from navigation_robot_remote_control.keyboard_teleop import normalize_key


def test_arrow_keys_are_normalized():
    assert normalize_key('\x1b[A') == 'w'
    assert normalize_key('\x1b[B') == 's'
    assert normalize_key('\x1b[D') == 'a'
    assert normalize_key('\x1b[C') == 'd'


def test_five_speed_gears_scale_twist():
    assert command_for_key('w', 1, 0.15, 0.60) == (0.03, 0.0)
    assert command_for_key('w', 5, 0.15, 0.60) == (0.15, 0.0)
    assert command_for_key('a', 1, 0.15, 0.60) == (0.0, 0.12)
    assert command_for_key('d', 5, 0.15, 0.60) == (0.0, -0.60)
    assert command_for_key('x', 1, 0.15, 0.60) is None


def test_only_explicit_ready_base_diagnostic_unlocks_motion():
    array = DiagnosticArray()
    status = DiagnosticStatus()
    status.name = 'navigation_robot/base_driver'
    status.level = DiagnosticStatus.WARN
    status.message = 'sensor-only motion lock'
    array.status.append(status)
    assert not base_status_ready(array)

    status.level = DiagnosticStatus.OK
    status.message = 'ready'
    assert base_status_ready(array)


def test_diagnostic_level_accepts_int_and_ros_uint8_byte():
    assert diagnostic_level_value(0) == 0
    assert diagnostic_level_value(b'\x00') == 0
    assert diagnostic_level_value(b'\x01') == 1


def test_command_stops_on_stale_key_or_lost_readiness():
    assert not command_must_stop(True, True, 1.0, 1.40, 0.45)
    assert command_must_stop(True, True, 1.0, 1.46, 0.45)
    assert command_must_stop(True, False, 1.0, 1.01, 0.45)
    assert not command_must_stop(False, False, 1.0, 10.0, 0.45)


def test_zero_key_timeout_latches_until_explicit_stop():
    assert not command_must_stop(True, True, 1.0, 1000.0, 0.0)
    assert command_must_stop(True, False, 1.0, 1.01, 0.0)
