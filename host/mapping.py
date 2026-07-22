"""单位换算、摇杆/键盘映射与控制使能标志。"""
from __future__ import annotations

from host.config import (MAX_CURRENT_MA, MAX_RPM, M0601_MODE_SPEED,
                         SPEED_GEARS)


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
