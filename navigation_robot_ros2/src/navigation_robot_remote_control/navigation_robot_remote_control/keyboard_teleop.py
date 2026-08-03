"""Direct terminal keyboard teleoperation over ROS 2 DDS."""

from __future__ import annotations

import os
import select
import sys
import termios
import time
import tty

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


GEAR_FACTORS = (0.20, 0.40, 0.60, 0.80, 1.00)
ARROW_KEYS = {
    '\x1b[A': 'w',
    '\x1b[B': 's',
    '\x1b[D': 'a',
    '\x1b[C': 'd',
}
MOTION_KEYS = {
    'w': (1.0, 0.0),
    's': (-1.0, 0.0),
    'a': (0.0, 1.0),
    'd': (0.0, -1.0),
}
HELP = """
navigation_robot 无线键盘遥控
----------------------------------------
W / ↑  前进       S / ↓  后退
A / ←  原地左转   D / →  原地右转
1..5    速度档位（20/40/60/80/100%）
空格    立即停车
Q       停车并退出

方向命令会持续执行，直到按下新方向、空格或 Q。
空格立即停车；电脑节点断线后由香橙派和下位机看门狗停车。
"""


def normalize_key(raw: str) -> str:
    """Normalize ANSI arrow sequences and printable keys."""
    return ARROW_KEYS.get(raw, raw.lower())


def command_for_key(
    key: str,
    gear: int,
    max_linear_speed: float,
    max_angular_speed: float,
) -> tuple[float, float] | None:
    """Map a keyboard direction and gear to a differential-drive Twist."""
    direction = MOTION_KEYS.get(normalize_key(key))
    if direction is None or not 1 <= gear <= len(GEAR_FACTORS):
        return None
    factor = GEAR_FACTORS[gear - 1]
    return (
        direction[0] * max_linear_speed * factor,
        direction[1] * max_angular_speed * factor,
    )


def diagnostic_level_value(level: object) -> int:
    """Normalize ROS Python uint8 fields represented as int or one byte."""
    if isinstance(level, (bytes, bytearray)):
        return level[0] if level else -1
    return int(level)


def command_must_stop(
    active: bool,
    ready: bool,
    last_key_time: float,
    now: float,
    key_timeout: float,
) -> bool:
    """Decide whether readiness was lost or an enabled key timeout elapsed."""
    return active and (
        not ready or (key_timeout > 0.0 and now - last_key_time > key_timeout)
    )


def base_status_ready(array: DiagnosticArray) -> bool:
    """Return true only for the base driver's explicit healthy state."""
    return any(
        status.name == 'navigation_robot/base_driver'
        and diagnostic_level_value(status.level) == 0
        and status.message == 'ready'
        for status in array.status
    )


class KeyboardTeleop(Node):
    """Publish bounded teleop commands directly to the Orange Pi base driver."""

    def __init__(self) -> None:
        super().__init__('navigation_robot_keyboard_teleop')
        self.declare_parameter('command_topic', '/cmd_vel')
        self.declare_parameter('diagnostics_topic', '/diagnostics')
        self.declare_parameter('max_linear_speed', 0.6545)
        self.declare_parameter('max_angular_speed', 5.2360)
        self.declare_parameter('initial_gear', 1)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('key_timeout_sec', 0.0)
        self.declare_parameter('diagnostic_timeout_sec', 2.5)
        self.declare_parameter('require_base_ready', False)

        command_topic = self.get_parameter('command_topic').value
        diagnostics_topic = self.get_parameter('diagnostics_topic').value
        self.max_linear_speed = float(
            self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(
            self.get_parameter('max_angular_speed').value)
        self.gear = int(self.get_parameter('initial_gear').value)
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self.key_timeout = float(self.get_parameter('key_timeout_sec').value)
        self.diagnostic_timeout = float(
            self.get_parameter('diagnostic_timeout_sec').value)
        self.require_base_ready = bool(
            self.get_parameter('require_base_ready').value)

        if self.max_linear_speed <= 0.0 or self.max_angular_speed <= 0.0:
            raise ValueError('maximum speeds must be positive')
        if not 1 <= self.gear <= len(GEAR_FACTORS):
            raise ValueError('initial_gear must be in the range 1..5')
        if publish_rate <= 0.0 or self.key_timeout < 0.0:
            raise ValueError('publish rate must be positive and key timeout non-negative')

        self.publish_period = 1.0 / publish_rate
        self.publisher = self.create_publisher(Twist, command_topic, 10)
        if self.require_base_ready:
            self.create_subscription(
                DiagnosticArray, diagnostics_topic, self._diagnostics_callback, 10)
        self.desired_linear = 0.0
        self.desired_angular = 0.0
        self.command_active = False
        self.last_key_time = 0.0
        self.next_publish_time = 0.0
        self.base_ready = False
        self.last_diagnostic_time = 0.0
        self.last_lock_warning = 0.0

        self.get_logger().info(
            f'publishing teleop to {command_topic}; initial gear={self.gear}')

    def _diagnostics_callback(self, array: DiagnosticArray) -> None:
        for status in array.status:
            if status.name != 'navigation_robot/base_driver':
                continue
            self.base_ready = (
                diagnostic_level_value(status.level) == 0
                and status.message == 'ready'
            )
            self.last_diagnostic_time = time.monotonic()
            if self.require_base_ready and not self.base_ready and self.command_active:
                self.stop()
            return

    def ready(self, now: float | None = None) -> bool:
        if not self.require_base_ready:
            return True
        current = time.monotonic() if now is None else now
        return (
            self.base_ready
            and self.last_diagnostic_time > 0.0
            and current - self.last_diagnostic_time <= self.diagnostic_timeout
        )

    def _publish(self, linear: float, angular: float) -> None:
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self.publisher.publish(message)

    def stop(self) -> None:
        was_active = self.command_active
        self.desired_linear = 0.0
        self.desired_angular = 0.0
        self.command_active = False
        if was_active:
            self._publish(0.0, 0.0)

    def handle_key(self, raw: str, now: float | None = None) -> bool:
        """Handle one terminal key; return false when the user requests exit."""
        current = time.monotonic() if now is None else now
        key = normalize_key(raw)
        if key == 'q':
            self.stop()
            return False
        if key == ' ':
            self.stop()
            self.get_logger().info('stop')
            return True
        if key in '12345':
            self.gear = int(key)
            self.get_logger().info(
                f'gear {self.gear}: {GEAR_FACTORS[self.gear - 1]:.0%}')
            return True

        command = command_for_key(
            key, self.gear, self.max_linear_speed, self.max_angular_speed)
        if command is None:
            return True
        if not self.ready(current):
            if current - self.last_lock_warning >= 2.0:
                self.get_logger().warning(
                    'motion blocked: waiting for fresh base_driver ready diagnostics')
                self.last_lock_warning = current
            self.stop()
            return True
        self.desired_linear, self.desired_angular = command
        self.last_key_time = current
        self.next_publish_time = current
        self.command_active = True
        return True

    def tick(self, now: float | None = None) -> None:
        current = time.monotonic() if now is None else now
        if not self.command_active:
            return
        if command_must_stop(
            self.command_active,
            self.ready(current),
            self.last_key_time,
            current,
            self.key_timeout,
        ):
            self.stop()
            return
        if current >= self.next_publish_time:
            self._publish(self.desired_linear, self.desired_angular)
            self.next_publish_time = current + self.publish_period

    def shutdown_stop(self) -> None:
        self.command_active = True
        for _ in range(3):
            self.stop()
            self.command_active = True
            time.sleep(0.03)
        self.command_active = False


def read_key(file_descriptor: int, timeout: float = 0.03) -> str:
    """Read one key or ANSI arrow sequence from a raw terminal."""
    readable, _, _ = select.select([file_descriptor], [], [], timeout)
    if not readable:
        return ''
    first = os.read(file_descriptor, 1).decode(errors='ignore')
    if first != '\x1b':
        return first
    sequence = first
    for _ in range(2):
        readable, _, _ = select.select([file_descriptor], [], [], 0.01)
        if not readable:
            break
        sequence += os.read(file_descriptor, 1).decode(errors='ignore')
    return sequence


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = KeyboardTeleop()
    if not sys.stdin.isatty():
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError('keyboard_teleop requires an interactive terminal')

    file_descriptor = sys.stdin.fileno()
    original_settings = termios.tcgetattr(file_descriptor)
    print(HELP, flush=True)
    try:
        tty.setraw(file_descriptor)
        keep_running = True
        while rclpy.ok() and keep_running:
            rclpy.spin_once(node, timeout_sec=0.0)
            raw = read_key(file_descriptor)
            if raw:
                keep_running = node.handle_key(raw)
            node.tick()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_stop()
        termios.tcsetattr(file_descriptor, termios.TCSADRAIN, original_settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
