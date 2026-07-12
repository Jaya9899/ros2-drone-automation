#!/usr/bin/env python3
"""
bcd_visual_sim.py — Standalone BCD Lawnmower Visual Simulation
═══════════════════════════════════════════════════════════════
Visualizes the Boustrophedon Cell Decomposition (BCD) search pattern
over the AeroTHON arena using Tkinter. No ROS, no MAVROS, no Gazebo.

Usage:
    python3 bcd_visual_sim.py

Controls:
    SPACE  — Pause / Resume
    R      — Restart simulation
    +/-    — Speed up / Slow down
    Q/Esc  — Quit
"""

import tkinter as tk
import math
import time

# ── Try shapely for polygon operations, fallback to simple rect subtraction ──
try:
    from shapely.geometry import Polygon as ShapelyPolygon
    from shapely.ops import unary_union
    HAS_SHAPELY = True
except ImportError:
    HAS_SHAPELY = False

ARENA_W = 40.0   # metres
ARENA_H = 30.0   # metres
ALTITUDE = 5.0   # metres AGL
HFOV_DEG = 62.2  # OAK-D horizontal FOV
OVERLAP = 0.80   # 80 % overlap between strips
DRONE_SPEED = 1.5  # m/s (arena cruise speed)
BUFFER_M = 0.5   # red zone safety buffer

RED_ZONES = [
    [(24, 12), (36, 12), (36, 22), (24, 22)],                        # Rectangle
    [(10, 5), (15, 5), (12.5, 10)],                                   # Triangle
    [(5, 20), (10, 20), (10, 22), (7, 22), (7, 25), (5, 25)],        # L-shape
]

CORRIDOR_ENTRY = (0.0, 15.0)

# ═════════════════════════════════════════════════════════════════════
# BCD Waypoint Generation  (adapted from waypoint_gen.py)
# ═════════════════════════════════════════════════════════════════════

def compute_strip_width(altitude=ALTITUDE, hfov_deg=HFOV_DEG, overlap=OVERLAP):
    hfov_rad = math.radians(hfov_deg)
    return 2 * altitude * math.tan(hfov_rad / 2) * overlap


def generate_lawnmower_simple(arena_w, arena_h, strip_width, red_zones, buffer_m):
    """Generate waypoints using Shapely polygon difference."""
    arena = ShapelyPolygon([(0, 0), (arena_w, 0), (arena_w, arena_h), (0, arena_h)])
    flyable = arena
    for rz_verts in red_zones:
        if len(rz_verts) >= 3:
            rz = ShapelyPolygon(rz_verts).buffer(buffer_m)
            flyable = flyable.difference(rz)

    bounds = flyable.bounds
    min_y, max_y = bounds[1], bounds[3]
    waypoints = []
    y = min_y + strip_width / 2
    left_to_right = True

    while y <= max_y:
        slab = ShapelyPolygon([
            (-1, y - strip_width / 2),
            (arena_w + 1, y - strip_width / 2),
            (arena_w + 1, y + strip_width / 2),
            (-1, y + strip_width / 2),
        ])
        intersection = flyable.intersection(slab)
        if not intersection.is_empty:
            geoms = intersection.geoms if intersection.geom_type == "MultiPolygon" else [intersection]
            for geom in geoms:
                try:
                    gx, gy = geom.exterior.xy
                except AttributeError:
                    continue
                xs = list(gx)
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


def generate_lawnmower_fallback(arena_w, arena_h, strip_width):
    """Fallback without Shapely — simple full-arena sweep."""
    waypoints = []
    y = strip_width / 2
    left_to_right = True
    while y <= arena_h:
        if left_to_right:
            waypoints.append((0, y))
            waypoints.append((arena_w, y))
        else:
            waypoints.append((arena_w, y))
            waypoints.append((0, y))
        left_to_right = not left_to_right
        y += strip_width
    return waypoints


# ═════════════════════════════════════════════════════════════════════
# Tkinter Visual Simulation
# ═════════════════════════════════════════════════════════════════════

class BCDSimulation:
    # Canvas sizing
    MARGIN = 60
    CANVAS_W = 900
    CANVAS_H = 700
    INFO_PANEL_W = 280

    # Colors
    BG          = "#0a0e17"
    ARENA_FILL  = "#131a26"
    ARENA_EDGE  = "#3fb950"
    RED_FILL    = "#c62828"
    RED_EDGE    = "#ff5252"
    RED_BUF     = "#ff525230"
    PATH_LINE   = "#2196F3"
    PATH_DONE   = "#1565C0"
    DRONE_FILL  = "#00e676"
    DRONE_EDGE  = "#ffffff"
    WP_COLOR    = "#ffffff30"
    WP_NEXT     = "#ffea00"
    STRIP_COLOR = "#2196F320"
    TEXT_COLOR  = "#e0e6ed"
    MUTED       = "#6e7681"
    ACCENT      = "#58a6ff"

    def __init__(self):
        # ── Generate waypoints ──────────────────────────────────
        self.strip_width = compute_strip_width()
        if HAS_SHAPELY:
            self.waypoints = generate_lawnmower_simple(
                ARENA_W, ARENA_H, self.strip_width, RED_ZONES, BUFFER_M)
        else:
            self.waypoints = generate_lawnmower_fallback(
                ARENA_W, ARENA_H, self.strip_width)

        # Prepend corridor entry
        self.waypoints = [CORRIDOR_ENTRY] + self.waypoints

        total_dist = sum(
            math.hypot(self.waypoints[i+1][0] - self.waypoints[i][0],
                       self.waypoints[i+1][1] - self.waypoints[i][1])
            for i in range(len(self.waypoints) - 1)
        )
        self.total_dist = total_dist
        self.est_time = total_dist / DRONE_SPEED

        # ── Simulation state ────────────────────────────────────
        self.drone_x, self.drone_y = self.waypoints[0]
        self.current_wp = 1
        self.trail = [self.waypoints[0]]
        self.paused = False
        self.sim_speed = 1.0
        self.distance_flown = 0.0
        self.elapsed = 0.0
        self.done = False
        self.last_tick = time.time()

        # ── Tkinter setup ───────────────────────────────────────
        self.root = tk.Tk()
        self.root.title("AeroTHON — BCD Lawnmower Simulation")
        self.root.configure(bg=self.BG)
        self.root.resizable(False, False)

        total_w = self.CANVAS_W + self.INFO_PANEL_W
        self.root.geometry(f"{total_w}x{self.CANVAS_H}")

        # Main frame
        main_frame = tk.Frame(self.root, bg=self.BG)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Canvas
        self.canvas = tk.Canvas(
            main_frame, width=self.CANVAS_W, height=self.CANVAS_H,
            bg=self.BG, highlightthickness=0)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Info panel
        self.info_frame = tk.Frame(main_frame, width=self.INFO_PANEL_W,
                                   bg="#161b22", padx=16, pady=16)
        self.info_frame.pack(side=tk.RIGHT, fill=tk.Y)
        self.info_frame.pack_propagate(False)

        self._build_info_panel()

        # ── Key bindings ────────────────────────────────────────
        self.root.bind("<space>", lambda e: self._toggle_pause())
        self.root.bind("r", lambda e: self._restart())
        self.root.bind("R", lambda e: self._restart())
        self.root.bind("+", lambda e: self._change_speed(0.5))
        self.root.bind("=", lambda e: self._change_speed(0.5))
        self.root.bind("-", lambda e: self._change_speed(-0.5))
        self.root.bind("q", lambda e: self.root.destroy())
        self.root.bind("<Escape>", lambda e: self.root.destroy())

        # ── Compute scale ───────────────────────────────────────
        draw_w = self.CANVAS_W - 2 * self.MARGIN
        draw_h = self.CANVAS_H - 2 * self.MARGIN
        self.scale = min(draw_w / ARENA_W, draw_h / ARENA_H)
        self.ox = self.MARGIN + (draw_w - ARENA_W * self.scale) / 2
        self.oy = self.MARGIN + (draw_h - ARENA_H * self.scale) / 2

        # ── Start loop ──────────────────────────────────────────
        self._tick()
        self.root.mainloop()

    # ─── Coordinate transform (arena metres → canvas pixels) ────
    def _tx(self, x):
        return self.ox + x * self.scale

    def _ty(self, y):
        return self.oy + (ARENA_H - y) * self.scale  # flip Y

    def _tpts(self, pts):
        """Convert list of (x,y) to flat [px1,py1,px2,py2,...] for canvas."""
        flat = []
        for x, y in pts:
            flat.append(self._tx(x))
            flat.append(self._ty(y))
        return flat

    # ─── Info panel ─────────────────────────────────────────────
    def _build_info_panel(self):
        f = self.info_frame

        # Title
        tk.Label(f, text="🛩  BCD Simulation", font=("Segoe UI", 14, "bold"),
                 fg=self.ACCENT, bg="#161b22").pack(anchor="w", pady=(0, 12))

        self._info_labels = {}
        metrics = [
            ("STATE", "---"),
            ("WAYPOINT", "0 / 0"),
            ("POSITION", "(0.0, 0.0)"),
            ("SPEED", "0.0 m/s"),
            ("DISTANCE", "0.0 / 0.0 m"),
            ("TIME", "0s / 0s"),
            ("PROGRESS", "0%"),
            ("SIM SPEED", "1.0x"),
            ("STRIP WIDTH", f"{self.strip_width:.2f} m"),
            ("ALTITUDE", f"{ALTITUDE} m"),
            ("TOTAL WPS", str(len(self.waypoints))),
        ]
        for label, default in metrics:
            lbl = tk.Label(f, text=label, font=("Segoe UI", 9),
                           fg=self.MUTED, bg="#161b22")
            lbl.pack(anchor="w", pady=(8, 0))
            val = tk.Label(f, text=default, font=("Consolas", 13, "bold"),
                           fg=self.TEXT_COLOR, bg="#161b22")
            val.pack(anchor="w")
            self._info_labels[label] = val

        # Separator
        tk.Frame(f, height=1, bg="#21262d").pack(fill=tk.X, pady=12)

        # Controls help
        tk.Label(f, text="CONTROLS", font=("Segoe UI", 9, "bold"),
                 fg=self.MUTED, bg="#161b22").pack(anchor="w")
        controls = "SPACE  Pause/Resume\nR      Restart\n+/-    Speed\nQ/Esc  Quit"
        tk.Label(f, text=controls, font=("Consolas", 10),
                 fg=self.MUTED, bg="#161b22", justify=tk.LEFT).pack(anchor="w", pady=(4, 0))

    def _update_info(self):
        progress = (self.distance_flown / self.total_dist * 100) if self.total_dist > 0 else 0
        state = "COMPLETE ✓" if self.done else ("PAUSED" if self.paused else "SEARCHING")
        state_color = "#3fb950" if self.done else ("#d29922" if self.paused else self.ACCENT)

        updates = {
            "STATE": (state, state_color),
            "WAYPOINT": (f"{min(self.current_wp, len(self.waypoints))} / {len(self.waypoints)}", None),
            "POSITION": (f"({self.drone_x:.1f}, {self.drone_y:.1f})", None),
            "SPEED": (f"{DRONE_SPEED * self.sim_speed:.1f} m/s", None),
            "DISTANCE": (f"{self.distance_flown:.0f} / {self.total_dist:.0f} m", None),
            "TIME": (f"{self.elapsed:.0f}s / {self.est_time:.0f}s", None),
            "PROGRESS": (f"{progress:.1f}%", None),
            "SIM SPEED": (f"{self.sim_speed:.1f}x", None),
        }
        for key, (text, color) in updates.items():
            lbl = self._info_labels.get(key)
            if lbl:
                lbl.config(text=text)
                if color:
                    lbl.config(fg=color)

    # ─── Simulation step ────────────────────────────────────────
    def _tick(self):
        now = time.time()
        dt = now - self.last_tick
        self.last_tick = now

        if not self.paused and not self.done and self.current_wp < len(self.waypoints):
            step = DRONE_SPEED * self.sim_speed * dt
            self._move_drone(step, dt)

        self._draw()
        self._update_info()
        self.root.after(16, self._tick)  # ~60 FPS

    def _move_drone(self, step, dt):
        self.elapsed += dt * self.sim_speed
        while step > 0 and self.current_wp < len(self.waypoints):
            tx, ty = self.waypoints[self.current_wp]
            dx = tx - self.drone_x
            dy = ty - self.drone_y
            dist = math.hypot(dx, dy)

            if dist <= step:
                # Reach this waypoint
                self.drone_x, self.drone_y = tx, ty
                self.distance_flown += dist
                step -= dist
                self.trail.append((tx, ty))
                self.current_wp += 1
            else:
                # Move towards waypoint
                ratio = step / dist
                self.drone_x += dx * ratio
                self.drone_y += dy * ratio
                self.distance_flown += step
                self.trail.append((self.drone_x, self.drone_y))
                step = 0

        if self.current_wp >= len(self.waypoints):
            self.done = True

    # ─── Drawing ────────────────────────────────────────────────
    def _draw(self):
        c = self.canvas
        c.delete("all")

        # Grid lines
        for gx in range(0, int(ARENA_W) + 1, 5):
            px = self._tx(gx)
            c.create_line(px, self._ty(0), px, self._ty(ARENA_H),
                          fill="#1c2333", width=1)
            c.create_text(px, self._ty(-1.5), text=str(gx), fill=self.MUTED,
                          font=("Consolas", 8))
        for gy in range(0, int(ARENA_H) + 1, 5):
            py = self._ty(gy)
            c.create_line(self._tx(0), py, self._tx(ARENA_W), py,
                          fill="#1c2333", width=1)
            c.create_text(self._tx(-2), py, text=str(gy), fill=self.MUTED,
                          font=("Consolas", 8))

        # Arena boundary
        arena_pts = self._tpts([(0, 0), (ARENA_W, 0), (ARENA_W, ARENA_H), (0, ARENA_H)])
        c.create_polygon(arena_pts, fill=self.ARENA_FILL, outline=self.ARENA_EDGE, width=2)

        # Red zones + buffer
        if HAS_SHAPELY:
            for rz_verts in RED_ZONES:
                # Buffer
                rz_buf = ShapelyPolygon(rz_verts).buffer(BUFFER_M)
                buf_coords = list(rz_buf.exterior.coords)
                c.create_polygon(self._tpts(buf_coords), fill="", outline=self.RED_EDGE,
                                 width=1, dash=(4, 4))
                # Zone
                c.create_polygon(self._tpts(rz_verts), fill=self.RED_FILL,
                                 outline=self.RED_EDGE, width=2)
        else:
            for rz_verts in RED_ZONES:
                c.create_polygon(self._tpts(rz_verts), fill=self.RED_FILL,
                                 outline=self.RED_EDGE, width=2)

        # Red zone labels
        for i, rz in enumerate(RED_ZONES):
            cx_rz = sum(p[0] for p in rz) / len(rz)
            cy_rz = sum(p[1] for p in rz) / len(rz)
            c.create_text(self._tx(cx_rz), self._ty(cy_rz),
                          text=f"RED\nZONE {i+1}", fill="white",
                          font=("Segoe UI", 8, "bold"), justify=tk.CENTER)

        # Planned path (faint future waypoints)
        if self.current_wp < len(self.waypoints):
            future = self.waypoints[self.current_wp:]
            if len(future) >= 2:
                c.create_line(*self._tpts(future), fill="#ffffff15", width=1)

        # Waypoint dots (future)
        for i in range(self.current_wp, len(self.waypoints)):
            wx, wy = self.waypoints[i]
            px, py = self._tx(wx), self._ty(wy)
            c.create_oval(px-2, py-2, px+2, py+2, fill=self.WP_COLOR,
                          outline="", width=0)

        # Next waypoint highlight
        if self.current_wp < len(self.waypoints):
            nx, ny = self.waypoints[self.current_wp]
            px, py = self._tx(nx), self._ty(ny)
            c.create_oval(px-5, py-5, px+5, py+5, outline=self.WP_NEXT, width=2)
            c.create_line(self._tx(self.drone_x), self._ty(self.drone_y),
                          px, py, fill=self.WP_NEXT, width=1, dash=(3, 3))

        # Trail (completed path)
        if len(self.trail) >= 2:
            c.create_line(*self._tpts(self.trail), fill=self.PATH_LINE,
                          width=2, smooth=False)

        # Drone
        dr = 8
        dpx, dpy = self._tx(self.drone_x), self._ty(self.drone_y)
        # Glow
        c.create_oval(dpx-dr*2, dpy-dr*2, dpx+dr*2, dpy+dr*2,
                       fill="", outline="#00e67640", width=2)
        # Body
        c.create_oval(dpx-dr, dpy-dr, dpx+dr, dpy+dr,
                       fill=self.DRONE_FILL, outline=self.DRONE_EDGE, width=2)

        # Camera FOV cone (strip width visualization)
        half_strip_px = self.strip_width / 2 * self.scale
        c.create_rectangle(dpx - half_strip_px, dpy - half_strip_px,
                           dpx + half_strip_px, dpy + half_strip_px,
                           fill="", outline="#00e67620", width=1)

        # Corridor entry marker
        ex, ey = self._tx(CORRIDOR_ENTRY[0]), self._ty(CORRIDOR_ENTRY[1])
        c.create_polygon(ex, ey-8, ex+10, ey, ex, ey+8,
                         fill="#d29922", outline="white", width=1)
        c.create_text(ex + 16, ey, text="Entry", fill="#d29922",
                      font=("Segoe UI", 8), anchor="w")

        # Title
        c.create_text(self.CANVAS_W / 2, 18,
                      text="AeroTHON Mission 2 — BCD Lawnmower Search",
                      fill=self.ACCENT, font=("Segoe UI", 12, "bold"))

        # Axis labels
        c.create_text(self.CANVAS_W / 2, self.CANVAS_H - 10,
                      text="East (m)", fill=self.MUTED, font=("Segoe UI", 9))
        c.create_text(14, self.CANVAS_H / 2,
                      text="North (m)", fill=self.MUTED,
                      font=("Segoe UI", 9), angle=90)

        # Progress bar at bottom
        bar_y = self.CANVAS_H - 28
        bar_w = self.CANVAS_W - 2 * self.MARGIN
        progress = self.distance_flown / self.total_dist if self.total_dist > 0 else 0
        c.create_rectangle(self.MARGIN, bar_y, self.MARGIN + bar_w, bar_y + 6,
                           fill="#21262d", outline="")
        c.create_rectangle(self.MARGIN, bar_y,
                           self.MARGIN + bar_w * min(progress, 1.0), bar_y + 6,
                           fill=self.ACCENT, outline="")

    # ─── Controls ───────────────────────────────────────────────
    def _toggle_pause(self):
        self.paused = not self.paused
        self.last_tick = time.time()

    def _restart(self):
        self.drone_x, self.drone_y = self.waypoints[0]
        self.current_wp = 1
        self.trail = [self.waypoints[0]]
        self.distance_flown = 0.0
        self.elapsed = 0.0
        self.done = False
        self.paused = False
        self.last_tick = time.time()

    def _change_speed(self, delta):
        self.sim_speed = max(0.5, min(10.0, self.sim_speed + delta))


# ═════════════════════════════════════════════════════════════════════
# Entry point
# ═════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    BCDSimulation()
