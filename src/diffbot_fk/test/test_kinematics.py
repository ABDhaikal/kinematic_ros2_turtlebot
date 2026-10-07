import math

import pytest

from diffbot_fk.kinematics import (
    DiffDriveOdometry,
    body_to_wheel_velocity,
    integrate_pose,
    normalize_angle,
    wheel_to_body_velocity,
)

R = 0.033
L = 0.160


def run(odom, w_l, w_r, duration, dt=0.02):
    """Simulate wheel encoders at constant velocity."""
    steps = max(1, int(round(duration / dt)))
    dt = duration / steps  # exact step so total time == duration
    phi_l = phi_r = 0.0
    odom.update(phi_l, phi_r, dt)
    for _ in range(steps):
        phi_l += w_l * dt
        phi_r += w_r * dt
        odom.update(phi_l, phi_r, dt)
    return odom


def test_velocity_roundtrip():
    w_l, w_r = body_to_wheel_velocity(0.2, 1.0, R, L)
    v, omega = wheel_to_body_velocity(w_l, w_r, R, L)
    assert v == pytest.approx(0.2)
    assert omega == pytest.approx(1.0)


def test_straight_line():
    w = 0.1 / R  # 0.1 m/s
    odom = run(DiffDriveOdometry(R, L), w, w, 10.0)
    assert odom.x == pytest.approx(1.0, abs=1e-6)
    assert odom.y == pytest.approx(0.0, abs=1e-9)
    assert odom.theta == pytest.approx(0.0, abs=1e-9)
    assert odom.distance == pytest.approx(1.0, abs=1e-6)
    assert odom.v == pytest.approx(0.1)


def test_rotate_in_place_full_turn():
    omega = 0.5
    w_l, w_r = body_to_wheel_velocity(0.0, omega, R, L)
    odom = run(DiffDriveOdometry(R, L), w_l, w_r, 2 * math.pi / omega, dt=0.01)
    assert odom.x == pytest.approx(0.0, abs=1e-9)
    assert odom.y == pytest.approx(0.0, abs=1e-9)
    assert abs(normalize_angle(odom.theta)) < 1e-3
    assert odom.rotation == pytest.approx(2 * math.pi, abs=1e-3)
    assert odom.distance == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize('method,tol', [('exact', 1e-9), ('midpoint', 1e-4), ('euler', 3e-2)])
def test_circle_returns_to_start(method, tol):
    radius, v = 0.5, 0.1
    w_l, w_r = body_to_wheel_velocity(v, v / radius, R, L)
    period = 2 * math.pi * radius / v
    odom = run(DiffDriveOdometry(R, L, method=method), w_l, w_r, period, dt=0.05)
    assert math.hypot(odom.x, odom.y) < tol
    assert odom.distance == pytest.approx(2 * math.pi * radius, rel=1e-3)


def test_quarter_circle_exact_pose():
    # Quarter-circle arc to the left, R = 0.5 m -> pose (0.5, 0.5, pi/2)
    x, y, th = integrate_pose(0.0, 0.0, 0.0, 0.5 * math.pi / 2, math.pi / 2, 'exact')
    assert (x, y, th) == pytest.approx((0.5, 0.5, math.pi / 2))


def test_normalize_angle():
    assert normalize_angle(3 * math.pi) == pytest.approx(math.pi)
    assert normalize_angle(-math.pi) == pytest.approx(math.pi)
    assert normalize_angle(-3 * math.pi / 2) == pytest.approx(math.pi / 2)


def test_reset():
    odom = run(DiffDriveOdometry(R, L), 3.0, 3.0, 1.0)
    odom.reset()
    assert (odom.x, odom.y, odom.theta, odom.distance) == (0.0, 0.0, 0.0, 0.0)
