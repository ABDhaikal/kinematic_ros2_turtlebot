import math

import pytest

from diffbot_fk.kinematics import DiffDriveOdometry, body_to_wheel_velocity
from diffbot_fk.patterns import SegmentPlanner, square_segments

R = 0.033
L = 0.160


def drive(planner, dt=0.05, max_time=300.0, wheel_scale=1.0):
    """Close the loop: planner -> wheel speeds -> encoders -> FK odometry -> planner.

    wheel_scale != 1 makes the real wheels turn a bit more/less than commanded
    (an actuator error that the closed loop has to absorb).
    """
    odom = DiffDriveOdometry(R, L)
    phi_l = phi_r = 0.0
    odom.update(phi_l, phi_r, dt)
    corners = []
    t = 0.0
    while not planner.done and t < max_time:
        n_done = len(planner.log)
        v, w = planner.update(odom.x, odom.y, odom.theta)
        if len(planner.log) > n_done and planner.log[-1][1] == 'line':
            corners.append((odom.x, odom.y))
        w_l, w_r = body_to_wheel_velocity(v, w, R, L)
        phi_l += w_l * wheel_scale * dt
        phi_r += w_r * wheel_scale * dt
        odom.update(phi_l, phi_r, dt)
        t += dt
    return odom, corners, t


def test_square_segments():
    segs = square_segments(1.0, loops=2, clockwise=True)
    assert len(segs) == 16
    assert [s.kind for s in segs[:2]] == ['line', 'turn']
    assert segs[1].target == pytest.approx(-math.pi / 2)


@pytest.mark.parametrize('clockwise', [False, True])
def test_square_closes(clockwise):
    planner = SegmentPlanner(square_segments(1.0, clockwise=clockwise))
    odom, corners, t = drive(planner)
    assert planner.done and t < 300.0
    assert math.hypot(odom.x, odom.y) < 0.02          # back at the start
    assert abs(odom.theta) < 0.02                      # facing the start heading
    assert odom.distance == pytest.approx(4.0, abs=0.04)
    assert odom.rotation == pytest.approx(2 * math.pi, abs=0.1)
    sign = -1 if clockwise else 1
    expected = [(1, 0), (1, sign), (0, sign), (0, 0)]
    for (x, y), (ex, ey) in zip(corners, expected):
        assert math.hypot(x - ex, y - ey) < 0.03


def test_square_with_actuator_error():
    # Wheels turn 10% more than commanded; feedback from FK still closes the square.
    planner = SegmentPlanner(square_segments(1.0))
    odom, _, _ = drive(planner, wheel_scale=1.1)
    assert math.hypot(odom.x, odom.y) < 0.03
    assert odom.distance == pytest.approx(4.0, abs=0.05)


def test_finished_planner_stops():
    planner = SegmentPlanner([])
    assert planner.done
    assert planner.update(0.0, 0.0, 0.0) == (0.0, 0.0)
