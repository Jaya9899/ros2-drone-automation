# corridor_render.py — ray-cast depth images of a walled corridor.
#
# Test/SITL helper (pure NumPy): renders what a forward depth camera sees in a
# straight corridor with box obstacles, so corridor_geometry can be checked
# against known ground truth and the planner can be flown in SITL without
# Gazebo. Not used on the vehicle.
#
# World frame: x along the corridor, y LEFT, z up, ground at z = 0. The corridor
# spans x in [x_start, x_end] with walls at y = +-width/2.

from dataclasses import dataclass, field

import numpy as np


@dataclass
class Scene:
    width: float = 1.524
    x_start: float = 2.0
    x_end: float = 12.0
    wall_height: float = 3.0
    # boxes: (x_min, x_max, y_min, y_max, z_top)
    boxes: list = field(default_factory=list)


@dataclass
class Camera:
    fx: float = 457.4
    fy: float = 457.4
    cx: float = 327.3
    cy: float = 240.3
    width: int = 640
    height: int = 480
    max_range: float = 15.0


def render(scene: Scene, cam: Camera, pos, yaw, pitch=0.0, roll=0.0):
    """Depth image (metres along the optical axis, NaN = no return).

    pos (x, y, z) camera position in the world, yaw CCW from +x, pitch
    nose-up, roll right-wing-down.
    """
    u, v = np.meshgrid(np.arange(cam.width), np.arange(cam.height))
    xc = (u - cam.cx) / cam.fx
    yc = (v - cam.cy) / cam.fy
    cp, sp, cy_, sy_ = np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    fwd = np.array([cp * cy_, cp * sy_, sp])
    left = np.array([-sy_, cy_, 0.0])
    up = np.array([-sp * cy_, -sp * sy_, cp])
    cr, sr = np.cos(roll), np.sin(roll)
    left, up = cr * left + sr * up, -sr * left + cr * up
    d = (fwd[None, None, :] - xc[..., None] * left - yc[..., None] * up)
    px, py, pz = pos
    t = np.full(u.shape, np.inf)

    def hit_plane(axis, value, lo, hi):
        """Intersect with the plane coord[axis] = value, bounded on the other
        two axes by lo/hi dicts {axis: (min, max)}."""
        nonlocal t
        with np.errstate(divide="ignore", invalid="ignore"):
            tt = (value - pos[axis]) / d[..., axis]
        ok = np.isfinite(tt) & (tt > 1e-6)
        for ax, (a, b) in lo.items():
            c = pos[ax] + tt * d[..., ax]
            ok &= (c >= a) & (c <= b)
        t = np.where(ok & (tt < t), tt, t)

    W2 = scene.width / 2
    for wy in (W2, -W2):
        hit_plane(1, wy, {0: (scene.x_start, scene.x_end),
                          2: (0.0, scene.wall_height)}, None)
    hit_plane(2, 0.0, {}, None)                                  # ground
    for (x0, x1, y0, y1, ztop) in scene.boxes:
        for xv in (x0, x1):
            hit_plane(0, xv, {1: (y0, y1), 2: (0.0, ztop)}, None)
        for yv in (y0, y1):
            hit_plane(1, yv, {0: (x0, x1), 2: (0.0, ztop)}, None)
        hit_plane(2, ztop, {0: (x0, x1), 1: (y0, y1)}, None)

    z = t                                    # d has optical-z component 1
    z[(z > cam.max_range)] = np.nan
    z[~np.isfinite(z)] = np.nan
    return z.astype(np.float32)
