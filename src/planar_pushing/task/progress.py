"""Time-based normalized progress; supervisor calls only during active pushing."""
from collections import deque
from dataclasses import dataclass
import numpy as np
from .selection import angle_error


@dataclass(frozen=True)
class ProgressConfig:
    window_seconds: float = 1.
    position_scale: float = .04
    angle_scale: float = .15
    min_absolute_decrease: float = .01
    min_relative_decrease: float = .02
    min_samples: int = 10

    def __post_init__(self):
        for key in ('window_seconds', 'position_scale', 'angle_scale'):
            if not np.isfinite(getattr(self, key)) or getattr(self, key) <= 0:
                raise ValueError(key)
        for key in ('min_absolute_decrease', 'min_relative_decrease'):
            if not np.isfinite(getattr(self, key)) or getattr(self, key) < 0:
                raise ValueError(key)
        if not isinstance(self.min_samples, int) or self.min_samples < 2:
            raise ValueError('min_samples')


@dataclass(frozen=True)
class ProgressReport:
    stalled: bool
    ready: bool
    metric: float
    decrease: float | None = None
    threshold: float | None = None


class StagnationDetector:
    def __init__(self, config=None):
        self.config = config or ProgressConfig()
        self.reset()

    def reset(self):
        self.history = deque()

    def update(self, observation, goal_pose):
        c = self.config
        e = np.asarray(goal_pose[:2])-observation.slider_pose[:2]
        yaw = angle_error(goal_pose[2], observation.slider_pose[2])
        value = float(np.hypot(np.linalg.norm(e)/c.position_scale, yaw/c.angle_scale))
        now = observation.time
        if self.history and now <= self.history[-1][0]:
            raise ValueError('progress timestamps must increase; reset on new task')
        self.history.append((now, value))
        # Keep one predecessor so a window need not divide dt exactly.
        while len(self.history) > 2 and self.history[1][0] <= now-c.window_seconds:
            self.history.popleft()
        ready = len(self.history) >= c.min_samples and now-self.history[0][0] >= c.window_seconds-1e-10
        if not ready:
            return ProgressReport(False, False, value)
        values = [v for _, v in self.history]
        n = min(3, len(values)//2)
        first, last = np.median(values[:n]), np.median(values[-n:])
        decrease = float(first-last)
        threshold = max(c.min_absolute_decrease, c.min_relative_decrease*float(first))
        return ProgressReport(decrease <= threshold, True, value, decrease, threshold)
