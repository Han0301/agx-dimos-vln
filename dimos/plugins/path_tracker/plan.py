# Extracted from dimos_simple/dimos/navigation/xnavdp/tracking.py (Apache-2.0).
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class MotionPlan:
    commands: np.ndarray
    world_path: np.ndarray
    started: float
    dt: float

    def command(
        self, now: float, valid_s: float, decel_s: float
    ) -> tuple[float, float]:
        age = max(0.0, now - self.started)
        valid_s = min(valid_s, len(self.commands) * self.dt)
        if age >= valid_s + decel_s:
            return 0.0, 0.0
        index = min(
            int(min(age, max(0.0, valid_s - 1e-9)) / self.dt), len(self.commands) - 1
        )
        scale = (
            1.0
            if age < valid_s
            else max(0.0, 1.0 - (age - valid_s) / max(decel_s, 1e-9))
        )
        v, w = self.commands[index] * scale
        return float(v), float(w)
