"""虚拟摇杆控件。"""
from __future__ import annotations

import tkinter as tk
from typing import Callable


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
