#!/usr/bin/env python3
"""Run a guarded, raised-wheel motion test against an active base driver."""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time

try:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger
except ModuleNotFoundError as exc:
    print(
        "ROS2 Python 环境尚未加载。请改用同目录下的 "
        "./test_real_base_motion.sh --wheels-raised",
        file=sys.stderr,
    )
    raise SystemExit(2) from exc


WHEEL_RADIUS_M = 0.05
WHEEL_SEPARATION_M = 0.25
TARGET_RPM = 25.0
TARGET_RAD_S = TARGET_RPM * 2.0 * math.pi / 60.0
LINEAR_MPS = TARGET_RAD_S * WHEEL_RADIUS_M
ANGULAR_RAD_S = 2.0 * TARGET_RAD_S * WHEEL_RADIUS_M / WHEEL_SEPARATION_M


def diagnostic_level(value: object) -> int:
    if isinstance(value, bytes):
        return int.from_bytes(value, byteorder="little")
    return int(value)


class MotionTest(Node):
    def __init__(self) -> None:
        super().__init__("navigation_robot_motion_test")
        self.ready = False
        self.last_joint_time = 0.0
        self.last_velocity = (0.0, 0.0)
        self.ack_errors: int | None = None
        self.crc_errors: int | None = None
        self.samples: list[tuple[float, float, float]] = []
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.joint_subscription = self.create_subscription(
            JointState, "/joint_states", self.on_joint_state, 10)
        self.diagnostics_subscription = self.create_subscription(
            DiagnosticArray, "/diagnostics", self.on_diagnostics, 10)
        self.stop_client = self.create_client(Trigger, "/base_driver/stop")

    def on_joint_state(self, msg: JointState) -> None:
        if len(msg.velocity) >= 2:
            now = time.monotonic()
            self.last_joint_time = now
            self.last_velocity = (float(msg.velocity[0]), float(msg.velocity[1]))
            self.samples.append((now, *self.last_velocity))

    def on_diagnostics(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            if status.name == "navigation_robot/base_driver":
                self.ready = (
                    diagnostic_level(status.level) == 0
                    and status.message == "ready"
                )
                values = {item.key: item.value for item in status.values}
                self.ack_errors = int(values.get("ack_errors", "-1"))
                self.crc_errors = int(values.get("host_crc_errors", "-1"))

    def spin_until_ready(self, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.ready and time.monotonic() - self.last_joint_time < 0.5:
                return
        joint_age = (
            "never" if self.last_joint_time == 0.0
            else f"{time.monotonic() - self.last_joint_time:.3f}s"
        )
        raise RuntimeError(
            f"底盘诊断或轮速门控未满足（ready={self.ready}, joint_age={joint_age}）")

    def wait_until_stopped(self) -> None:
        deadline = time.monotonic() + 0.8
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if max(abs(value) for value in self.last_velocity) < 0.35:
                return
        raise RuntimeError(
            f"停车后轮速未归零：left={self.last_velocity[0]:+.3f}, "
            f"right={self.last_velocity[1]:+.3f} rad/s"
        )

    def zero_stop(self) -> None:
        zero = Twist()
        end = time.monotonic() + 0.4
        next_publish = time.monotonic()
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_publish:
                self.publisher.publish(zero)
                next_publish += 0.05
            rclpy.spin_once(self, timeout_sec=0.01)
        self.wait_until_stopped()

    def brake_stop(self) -> None:
        if not self.stop_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("找不到 /base_driver/stop 服务")
        future = self.stop_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError("底盘刹车服务调用失败")
        self.wait_until_stopped()

    def pulse(self, name: str, linear: float, angular: float) -> tuple[float, float]:
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        start = time.monotonic()
        end = start + 1.5
        next_publish = start
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_publish:
                self.publisher.publish(command)
                next_publish += 0.05
            rclpy.spin_once(self, timeout_sec=0.01)
        self.zero_stop()

        usable = [sample for sample in self.samples if start + 0.45 <= sample[0] <= end]
        if len(usable) < 3:
            raise RuntimeError(f"{name}: 运动期间轮速样本不足（{len(usable)}）")
        left = statistics.median(sample[1] for sample in usable)
        right = statistics.median(sample[2] for sample in usable)
        print(
            f"{name}: 左={left:+.3f} rad/s ({left * 60.0 / (2.0 * math.pi):+.1f} RPM), "
            f"右={right:+.3f} rad/s ({right * 60.0 / (2.0 * math.pi):+.1f} RPM)",
            flush=True,
        )
        return left, right


def signs_and_speed_ok(actual: tuple[float, float], expected: tuple[int, int]) -> bool:
    minimum = TARGET_RAD_S * 0.55
    maximum = TARGET_RAD_S * 1.45
    return all(
        value * sign > 0.0 and minimum <= abs(value) <= maximum
        for value, sign in zip(actual, expected)
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--wheels-raised",
        action="store_true",
        help="required acknowledgement that both drive wheels are off the ground",
    )
    args = parser.parse_args()
    if not args.wheels_raised:
        parser.error("必须先架空两轮，再提供 --wheels-raised")

    rclpy.init()
    node = MotionTest()
    failed: list[str] = []
    try:
        node.spin_until_ready(8.0)
        if max(abs(value) for value in node.last_velocity) >= 0.35:
            node.brake_stop()
            raise RuntimeError("测试开始前轮子不处于零速")
        initial_ack_errors = node.ack_errors
        initial_crc_errors = node.crc_errors
        if initial_ack_errors is None or initial_crc_errors is None:
            raise RuntimeError("未收到 ACK/CRC 错误计数")
        print("底盘 ready；初始轮速为零", flush=True)
        phases = (
            ("前进", +LINEAR_MPS, 0.0, (+1, +1)),
            ("后退", -LINEAR_MPS, 0.0, (-1, -1)),
            ("原地左转", 0.0, +ANGULAR_RAD_S, (-1, +1)),
            ("原地右转", 0.0, -ANGULAR_RAD_S, (+1, -1)),
        )
        for name, linear, angular, expected in phases:
            actual = node.pulse(name, linear, angular)
            if not signs_and_speed_ok(actual, expected):
                failed.append(name)
            idle_deadline = time.monotonic() + 1.0
            while time.monotonic() < idle_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)

        node.brake_stop()
        print("最终刹车服务验证成功", flush=True)

        diagnostic_deadline = time.monotonic() + 1.3
        while time.monotonic() < diagnostic_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.ack_errors != initial_ack_errors:
            raise RuntimeError(
                f"运动期间 ACK 错误从 {initial_ack_errors} 增加到 {node.ack_errors}"
            )
        if node.crc_errors != initial_crc_errors:
            raise RuntimeError(
                f"运动期间 CRC 错误从 {initial_crc_errors} 增加到 {node.crc_errors}"
            )
    except (KeyboardInterrupt, RuntimeError) as exc:
        try:
            node.brake_stop()
        except Exception:
            pass
        print(f"测试中止并停车：{exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if failed:
        print(f"反馈方向或速度不符合预期：{', '.join(failed)}", file=sys.stderr)
        return 1
    print("四项运动反馈均通过；最终刹车已发送")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
