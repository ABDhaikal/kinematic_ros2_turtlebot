"""Forward kinematics of a differential drive robot (pure Python, no ROS).

Notation:
    r       : wheel radius [m]
    L       : distance between left and right wheels (wheel separation) [m]
    w_l,w_r : left/right wheel angular velocity [rad/s]
    phi     : wheel angular position (encoder) [rad]
    pose    : (x, y, theta) in the odom frame
"""

import math
from dataclasses import dataclass


def normalize_angle(angle):
    """Normalize an angle to the range (-pi, pi]."""
    a = math.atan2(math.sin(angle), math.cos(angle))
    return math.pi if a == -math.pi else a


def wheel_to_body_velocity(w_l, w_r, r, L):
    """Velocity FK: (w_l, w_r) -> (v, omega).

    v     = r (w_r + w_l) / 2
    omega = r (w_r - w_l) / L
    """
    v_l = r * w_l
    v_r = r * w_r
    return (v_r + v_l) / 2.0, (v_r - v_l) / L


def body_to_wheel_velocity(v, omega, r, L):
    """Inverse kinematics: (v, omega) -> (w_l, w_r)."""
    w_l = (v - omega * L / 2.0) / r
    w_r = (v + omega * L / 2.0) / r
    return w_l, w_r


def wheel_increment_to_body(d_phi_l, d_phi_r, r, L):
    """Wheel angle increments -> robot (ds, dtheta).

    ds_l   = r * d_phi_l,  ds_r = r * d_phi_r
    ds     = (ds_r + ds_l) / 2
    dtheta = (ds_r - ds_l) / L
    """
    ds_l = r * d_phi_l
    ds_r = r * d_phi_r
    return (ds_r + ds_l) / 2.0, (ds_r - ds_l) / L


def integrate_pose(x, y, theta, ds, dtheta, method='exact'):
    """Integrate one odometry step.

    method:
        'euler'    : x += ds cos(theta)
        'midpoint' : x += ds cos(theta + dtheta/2)   (2nd-order Runge-Kutta)
        'exact'    : circular arc, exact for constant v and omega
    """
    if method == 'euler':
        x += ds * math.cos(theta)
        y += ds * math.sin(theta)
    elif method == 'midpoint' or abs(dtheta) < 1e-9:
        x += ds * math.cos(theta + dtheta / 2.0)
        y += ds * math.sin(theta + dtheta / 2.0)
    elif method == 'exact':
        radius = ds / dtheta
        x += radius * (math.sin(theta + dtheta) - math.sin(theta))
        y -= radius * (math.cos(theta + dtheta) - math.cos(theta))
    else:
        raise ValueError('method must be euler, midpoint, or exact')
    return x, y, normalize_angle(theta + dtheta)


@dataclass
class DiffDriveOdometry:
    """Wheel-encoder odometry + distance tracking."""

    wheel_radius: float
    wheel_separation: float
    method: str = 'exact'

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    v: float = 0.0
    omega: float = 0.0
    distance: float = 0.0          # total path length [m]
    rotation: float = 0.0          # total rotation |dtheta| [rad]
    distance_left: float = 0.0     # distance travelled by left wheel [m]
    distance_right: float = 0.0    # distance travelled by right wheel [m]

    _last_l: float = None
    _last_r: float = None

    def reset(self, x=0.0, y=0.0, theta=0.0):
        self.x, self.y, self.theta = x, y, normalize_angle(theta)
        self.v = self.omega = 0.0
        self.distance = self.rotation = 0.0
        self.distance_left = self.distance_right = 0.0
        self._last_l = self._last_r = None

    @property
    def displacement(self):
        """Straight-line distance from the start point (0, 0) [m]."""
        return math.hypot(self.x, self.y)

    def update(self, phi_l, phi_r, dt):
        """Update the pose from absolute wheel angular positions [rad].

        The first call only stores the initial encoder values.
        Returns (ds, dtheta) for this step.
        """
        if self._last_l is None:
            self._last_l, self._last_r = phi_l, phi_r
            return 0.0, 0.0

        # Wrap so it stays correct if the encoder is reported in (-pi, pi].
        d_l = normalize_angle(phi_l - self._last_l)
        d_r = normalize_angle(phi_r - self._last_r)
        self._last_l, self._last_r = phi_l, phi_r

        r, L = self.wheel_radius, self.wheel_separation
        ds, dtheta = wheel_increment_to_body(d_l, d_r, r, L)
        self.x, self.y, self.theta = integrate_pose(
            self.x, self.y, self.theta, ds, dtheta, self.method)

        self.distance += abs(ds)
        self.rotation += abs(dtheta)
        self.distance_left += abs(r * d_l)
        self.distance_right += abs(r * d_r)
        if dt > 0.0:
            self.v = ds / dt
            self.omega = dtheta / dt
        return ds, dtheta
