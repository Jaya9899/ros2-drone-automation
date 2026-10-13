# Corridor fit against ray-cast ground truth (5 ft corridor).
import math

import numpy as np
import pytest

from obstacle_avoidance import corridor_geometry as cg
from obstacle_avoidance.corridor_render import Camera, Scene, render

CAM = Camera()
P = cg.CorridorParams()
W2 = P.width_m / 2


def fit(scene, pos, yaw=0.0, pitch=0.0, noise=0.0, seed=0, roll=0.0):
    z = render(scene, CAM, pos, yaw, pitch, roll)
    if noise:
        rng = np.random.default_rng(seed)
        z = z * (1.0 + rng.normal(0.0, noise, z.shape)).astype(np.float32)
    x, y, h = cg.level_points(z, CAM.fx, CAM.fy, CAM.cx, CAM.cy, pitch, roll=roll)
    left, right, upf = cg.view_limits(CAM.fx, CAM.fy, CAM.cx, CAM.cy, CAM.width)
    return cg.fit_corridor(x, y, h, P, fov=(left, right, upf + pitch))


@pytest.mark.parametrize("dy", [0.0, 0.25, -0.30])
@pytest.mark.parametrize("yaw_deg", [0.0, 8.0, -12.0])
def test_offset_and_heading(dy, yaw_deg):
    r = fit(Scene(x_start=-5.0), (0.0, dy, 1.5), math.radians(yaw_deg))
    assert r["walls"] == 2
    assert r["heading_err"] == pytest.approx(-math.radians(yaw_deg), abs=0.02)
    assert r["wall_left"] == pytest.approx(W2 - dy, abs=0.05)
    assert r["wall_right"] == pytest.approx(-W2 - dy, abs=0.05)
    assert math.isnan(r["obs_dist"])
    assert math.isnan(r["mouth_dist"])          # already inside


def test_pitch_compensated():
    # Forward flight pitches nose-down; the fit must not change.
    r = fit(Scene(x_start=-5.0), (0.0, 0.2, 1.5), 0.0, pitch=math.radians(-8))
    assert r["walls"] == 2
    assert r["wall_left"] == pytest.approx(W2 - 0.2, abs=0.05)


@pytest.mark.parametrize("roll_deg", [-10.0, 10.0])
def test_roll_compensated(roll_deg):
    # A sideways move rolls the vehicle; offsets, heights and the obstacle
    # top must not change.
    box = (2.0, 2.4, -0.275, 0.275, 2.0)
    r = fit(Scene(x_start=-5.0, boxes=[box]), (0.0, 0.2, 1.5), 0.0,
            pitch=math.radians(-5), roll=math.radians(roll_deg))
    assert r["walls"] == 2
    assert r["wall_left"] == pytest.approx(W2 - 0.2, abs=0.05)
    assert r["obs_top"] == pytest.approx(0.5, abs=0.1)


def test_quaternion_angles():
    # FLU->ENU quaternion for roll r then pitch p (nose-up positive here).
    from obstacle_avoidance import geometry as g
    r, p = math.radians(7), math.radians(-4)
    # q = qy(-p) * qx(r)   (aerospace nose-up p is rotation -p about +y FLU)
    cy_, sy_ = math.cos(-p / 2), math.sin(-p / 2)
    cx_, sx_ = math.cos(r / 2), math.sin(r / 2)
    w, x, y, z = cy_ * cx_, cy_ * sx_, sy_ * cx_, -sy_ * sx_
    assert g.roll_from_quaternion(x, y, z, w) == pytest.approx(r, abs=1e-6)
    assert g.pitch_from_quaternion(x, y, z, w) == pytest.approx(p, abs=1e-6)


def test_depth_noise():
    # ~1% range noise, worse than OAK-D Lite subpixel at these ranges.
    r = fit(Scene(x_start=-5.0), (0.0, -0.15, 1.5), 0.05, noise=0.01)
    assert r["walls"] == 2
    assert r["wall_left"] == pytest.approx(W2 + 0.15, abs=0.07)


def test_in_line_with_a_wall_before_the_mouth():
    # GPS error put the vehicle right on the right wall's line: that wall is
    # seen end-on (invisible), only the left wall is usable.
    r = fit(Scene(x_start=2.5), (0.0, -W2, 1.5))
    assert r["walls"] == 1
    assert r["wall_left"] == pytest.approx(2 * W2, abs=0.06)
    assert 0.5 * (r["wall_left"] + r["wall_right"]) == pytest.approx(W2, abs=0.06)


def test_entrance_ahead():
    r = fit(Scene(x_start=3.0), (0.0, 0.3, 1.5))
    assert r["walls"] == 2
    assert r["mouth_dist"] == pytest.approx(3.0, abs=0.2)
    assert r["wall_left"] == pytest.approx(W2 - 0.3, abs=0.06)


def test_exit_ahead():
    r = fit(Scene(x_start=-5.0, x_end=2.5), (0.0, 0.0, 1.5))
    assert r["walls"] == 2
    assert r["walls_end"] == pytest.approx(2.5, abs=0.2)


def test_obstacle_against_wall_is_not_an_exit():
    # A 40 % obstacle against the left wall hides the wall behind it: the left
    # wall "stops" at the obstacle. That must not read as the exit.
    box = (2.5, 2.9, W2 - 0.61, W2, 2.2)
    r = fit(Scene(x_start=-5.0, boxes=[box]), (0.0, -0.3, 1.5))
    assert r["walls"] >= 1
    assert math.isnan(r["walls_end"])


def test_no_corridor_in_open_field():
    r = fit(Scene(x_start=50.0, x_end=60.0), (0.0, 0.0, 1.5))
    assert r["walls"] == 0


def test_obstacle_on_left_wall():
    # 40% of the corridor (0.61 m) against the left wall, 2 m ahead.
    box = (2.0, 2.4, W2 - 0.61, W2, 2.0)
    r = fit(Scene(x_start=-5.0, boxes=[box]), (0.0, 0.0, 1.5))
    assert r["walls"] == 2
    assert r["wall_left"] == pytest.approx(W2, abs=0.05)
    assert r["obs_dist"] == pytest.approx(2.0, abs=0.1)
    assert r["obs_right"] == pytest.approx(W2 - 0.61, abs=0.08)


def test_obstacle_below_slice_ignored():
    # Top 1.0 m with the camera at 1.5 m: 0.5 m under the camera, below the
    # slice -> the vehicle passes over it, not an obstacle.
    box = (2.0, 2.4, -0.3, 0.3, 1.0)
    r = fit(Scene(x_start=-5.0, boxes=[box]), (0.0, 0.0, 1.5))
    assert r["walls"] == 2
    assert math.isnan(r["obs_dist"])


def test_obstacle_centred_with_top():
    # 0.55 m wide, centred, 2.0 m tall; camera at 1.5 m sees its top.
    box = (2.0, 2.4, -0.275, 0.275, 2.0)
    r = fit(Scene(x_start=-5.0, boxes=[box]), (0.0, 0.0, 1.5))
    assert r["walls"] == 2
    assert r["obs_dist"] == pytest.approx(2.0, abs=0.1)
    assert r["obs_left"] == pytest.approx(0.275, abs=0.08)
    assert r["obs_right"] == pytest.approx(-0.275, abs=0.08)
    assert r["obs_top"] == pytest.approx(0.5, abs=0.1)


def test_obstacle_taller_than_view():
    box = (1.2, 1.6, -0.275, 0.275, 6.0)
    r = fit(Scene(x_start=-5.0, boxes=[box]), (0.0, 0.0, 1.5))
    assert r["obs_dist"] == pytest.approx(1.2, abs=0.1)
    assert math.isnan(r["obs_top"])


def test_one_wall_visible():
    # Yawed hard right: only the left wall is in view at useful range.
    r = fit(Scene(x_start=-5.0), (0.0, -0.3, 1.5), math.radians(-38))
    assert r["walls"] in (1, 2)
    assert r["wall_left"] == pytest.approx(W2 + 0.3, abs=0.08)


def test_fit_is_fast_enough():
    import time
    z = render(Scene(x_start=-5.0), CAM, (0.0, 0.0, 1.5), 0.0)
    x, y, h = cg.level_points(z, CAM.fx, CAM.fy, CAM.cx, CAM.cy, 0.0)
    t0 = time.perf_counter()
    for _ in range(5):
        cg.fit_corridor(x, y, h, P)
    # Laptop budget; Pi 5 is ~3x slower and the node runs at 10 Hz.
    assert (time.perf_counter() - t0) / 5 < 0.03


def test_ground_cut_with_known_height():
    # Flying low (0.4 m): the ground is inside the -0.35 m slice at range and
    # would read as a floor-wide "obstacle"; the known height removes it.
    z = render(Scene(x_start=-5.0), CAM, (0.0, 0.0, 0.4), 0.0)
    x, y, h = cg.level_points(z, CAM.fx, CAM.fy, CAM.cx, CAM.cy, 0.0)
    r = cg.fit_corridor(x, y, h, P, height_above_ground=0.4)
    assert r["walls"] == 2
    assert math.isnan(r["obs_dist"])
