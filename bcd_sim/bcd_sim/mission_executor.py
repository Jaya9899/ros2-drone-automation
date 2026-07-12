#!/usr/bin/env python3
"""
bcd_visual_sim_safe.py — Segfault-safe BCD Simulation
"""

import tkinter as tk
import math
import time

try:
    from shapely.geometry import Polygon as ShapelyPolygon
    HAS_SHAPELY = True
except ImportError:
    HAS_SHAPELY = False


ARENA_W = 40.0
ARENA_H = 30.0
ALTITUDE = 5.0
HFOV_DEG = 62.2
OVERLAP = 0.80
DRONE_SPEED = 1.5
BUFFER_M = 0.5

RED_ZONES = [
    [(24, 12), (36, 12), (36, 22), (24, 22)],
    [(10, 5), (15, 5), (12.5, 10)],
    [(5, 20), (10, 20), (10, 22), (7, 22), (7, 25), (5, 25)],
]

CORRIDOR_ENTRY = (0.0, 15.0)


def compute_strip_width():
    return 2 * ALTITUDE * math.tan(math.radians(HFOV_DEG) / 2) * OVERLAP


# ✅ SAFE GEOMETRY CLEANER
def safe_polygon(poly):
    try:
        if not poly.is_valid:
            poly = poly.buffer(0)
        return poly if poly.is_valid else None
    except Exception:
        return None


def generate_lawnmower_simple():
    """SAFE Shapely-based BCD"""
    try:
        arena = ShapelyPolygon([(0, 0), (ARENA_W, 0), (ARENA_W, ARENA_H), (0, ARENA_H)])
        flyable = arena

        for rz_verts in RED_ZONES:
            if len(rz_verts) >= 3:
                rz = ShapelyPolygon(rz_verts).buffer(BUFFER_M)
                rz = safe_polygon(rz)
                if rz:
                    flyable = flyable.difference(rz)

        flyable = safe_polygon(flyable)
        if not flyable:
            raise Exception("Invalid flyable region")

        strip_width = compute_strip_width()
        bounds = flyable.bounds
        min_y, max_y = bounds[1], bounds[3]

        waypoints = []
        y = min_y + strip_width / 2
        left_to_right = True

        while y <= max_y:
            slab = ShapelyPolygon([
                (-1, y - strip_width / 2),
                (ARENA_W + 1, y - strip_width / 2),
                (ARENA_W + 1, y + strip_width / 2),
                (-1, y + strip_width / 2),
            ])

            try:
                intersection = flyable.intersection(slab)
            except Exception:
                y += strip_width
                continue

            if intersection.is_empty:
                y += strip_width
                continue

            if intersection.geom_type == "MultiPolygon":
                geoms = intersection.geoms
            elif intersection.geom_type == "Polygon":
                geoms = [intersection]
            else:
                y += strip_width
                continue

            for geom in geoms:
                geom = safe_polygon(geom)
                if not geom:
                    continue

                try:
                    if not hasattr(geom, "exterior"):
                        continue
                    xs = list(geom.exterior.xy[0])
                except Exception:
                    continue

                if not xs:
                    continue

                min_x, max_x = min(xs), max(xs)

                if left_to_right:
                    waypoints.append((min_x, y))
                    waypoints.append((max_x, y))
                else:
                    waypoints.append((max_x, y))
                    waypoints.append((min_x, y))

            left_to_right = not left_to_right
            y += strip_width

        return waypoints

    except Exception as e:
        print("⚠️ Shapely failed, using fallback:", e)
        return generate_lawnmower_fallback()


def generate_lawnmower_fallback():
    """100% safe fallback"""
    strip_width = compute_strip_width()
    waypoints = []
    y = strip_width / 2
    left_to_right = True

    while y <= ARENA_H:
        if left_to_right:
            waypoints.append((0, y))
            waypoints.append((ARENA_W, y))
        else:
            waypoints.append((ARENA_W, y))
            waypoints.append((0, y))
        left_to_right = not left_to_right
        y += strip_width

    return waypoints


class BCDSimulation:
    def __init__(self):
        self.strip_width = compute_strip_width()

        if HAS_SHAPELY:
            self.waypoints = generate_lawnmower_simple()
        else:
            self.waypoints = generate_lawnmower_fallback()

        self.waypoints = [CORRIDOR_ENTRY] + self.waypoints

        self.drone_x, self.drone_y = self.waypoints[0]
        self.current_wp = 1

        self.root = tk.Tk()
        self.root.title("BCD Safe Simulation")

        self.canvas = tk.Canvas(self.root, width=800, height=600, bg="black")
        self.canvas.pack()

        self.scale = 15

        self._tick()
        self.root.mainloop()

    def _tx(self, x):
        return x * self.scale + 50

    def _ty(self, y):
        return 550 - y * self.scale

    def _tick(self):
        self.canvas.delete("all")

        # draw path
        for i in range(len(self.waypoints) - 1):
            x1, y1 = self.waypoints[i]
            x2, y2 = self.waypoints[i + 1]
            self.canvas.create_line(
                self._tx(x1), self._ty(y1),
                self._tx(x2), self._ty(y2),
                fill="blue"
            )

        # draw drone
        self.canvas.create_oval(
            self._tx(self.drone_x) - 5,
            self._ty(self.drone_y) - 5,
            self._tx(self.drone_x) + 5,
            self._ty(self.drone_y) + 5,
            fill="lime"
        )

        self.root.after(100, self._tick)


if __name__ == "__main__":
    BCDSimulation()