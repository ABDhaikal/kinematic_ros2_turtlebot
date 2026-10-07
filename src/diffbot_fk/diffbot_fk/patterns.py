"""Motion pattern planners driven by forward kinematics odometry (pure Python, no ROS).

A planner receives the current FK pose (x, y, theta) and returns the velocity
command (v, omega). Segment ends are decided from the measured pose, not from
time, so the pattern closes the loop on the FK result.
"""

import math
from dataclasses import dataclass, field

from diffbot_fk.kinematics import normalize_angle


@dataclass
class Segment:
    """One step of a pattern: drive straight ('line') or rotate in place ('turn')."""

    kind: str          # 'line' | 'turn'
    target: float      # line: distance [m], turn: signed angle [rad]


def square_segments(side, loops=1, clockwise=False):
    """Square = 4 x (straight side + 90 deg turn in place), repeated `loops` times."""
    turn = -math.pi / 2 if clockwise else math.pi / 2
    segments = []
    for _ in range(loops * 4):
        segments.append(Segment('line', side))
        segments.append(Segment('turn', turn))
    return segments


@dataclass
class SegmentPlanner:
    """Executes a list of segments using FK pose feedback.

    line: drive forward until the straight-line distance from the segment start
          reaches the target; heading is held with a proportional correction.
    turn: rotate in place until the heading change reaches the target.
    Speeds ramp down near the end of each segment to limit overshoot.
    """

    segments: list
    linear_speed: float = 0.15       # [m/s]
    angular_speed: float = 0.3       # [rad/s]
    distance_tolerance: float = 0.005  # [m]
    angle_tolerance: float = 0.01    # [rad]
    heading_gain: float = 2.0        # [1/s] heading hold while driving straight
    slowdown_distance: float = 0.10  # [m] start slowing down this far from the end
    slowdown_angle: float = 0.35     # [rad]
    min_linear_speed: float = 0.02   # [m/s]
    min_angular_speed: float = 0.1   # [rad/s]

    index: int = 0
    _start: tuple = None             # pose at the start of the current segment
    _heading_ref: float = None       # absolute heading to hold / reach
    log: list = field(default_factory=list)  # (index, kind, start pose, end pose)

    @property
    def done(self):
        return self.index >= len(self.segments)

    @property
    def current(self):
        return None if self.done else self.segments[self.index]

    def _begin(self, pose):
        self._start = pose
        seg = self.segments[self.index]
        if self._heading_ref is None:
            self._heading_ref = pose[2]
        if seg.kind == 'turn':
            # Turn relative to the previous reference heading (not the measured one),
            # so small errors from earlier segments do not accumulate.
            self._heading_ref = normalize_angle(self._heading_ref + seg.target)

    def _finish(self, pose):
        seg = self.segments[self.index]
        self.log.append((self.index, seg.kind, self._start, pose))
        self.index += 1
        self._start = None

    @staticmethod
    def _ramp(remaining, slowdown, full, minimum):
        return max(minimum, min(full, full * remaining / slowdown))

    def update(self, x, y, theta):
        """Return (v, omega) for the current pose; (0, 0) when the pattern is finished."""
        pose = (x, y, theta)
        while not self.done:
            if self._start is None:
                self._begin(pose)
            seg = self.segments[self.index]

            if seg.kind == 'line':
                travelled = math.hypot(x - self._start[0], y - self._start[1])
                remaining = seg.target - travelled
                if remaining <= self.distance_tolerance:
                    self._finish(pose)
                    continue
                v = self._ramp(remaining, self.slowdown_distance,
                               self.linear_speed, self.min_linear_speed)
                heading_error = normalize_angle(self._heading_ref - theta)
                omega = max(-self.angular_speed,
                            min(self.angular_speed, self.heading_gain * heading_error))
                return v, omega

            if seg.kind == 'turn':
                error = normalize_angle(self._heading_ref - theta)
                if abs(error) <= self.angle_tolerance:
                    self._finish(pose)
                    continue
                w = self._ramp(abs(error), self.slowdown_angle,
                               self.angular_speed, self.min_angular_speed)
                return 0.0, math.copysign(w, error)

            raise ValueError(f'unknown segment kind: {seg.kind}')
        return 0.0, 0.0
