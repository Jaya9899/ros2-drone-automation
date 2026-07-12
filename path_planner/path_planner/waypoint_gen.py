# waypoint_gen.py — BCD Lawnmower waypoint generator
#
# Generates Boustrophedon Cell Decomposition lawnmower search patterns
# for the arena, avoiding red zones.
#
# Can be used as:
#   - Importable library: from path_planner.waypoint_gen import generate_mission_waypoints
#   - Standalone script: python3 waypoint_gen.py (generates plot + prints waypoints)

import numpy as np
from shapely.geometry import Polygon

# ---------------------------------------------------------------------------
# Default arena parameters (overridable via function arguments)
# ---------------------------------------------------------------------------
ARENA_WIDTH_M   = 40.0
ARENA_HEIGHT_M  = 30.0
ALTITUDE_M      = 5.0
HFOV_DEG        = 62.2       # OAK-D horizontal FOV
OVERLAP         = 0.80       # 80% overlap between strips
ARENA_SPEED_MS  = 1.5

CORRIDOR_ENTRY_X = 0.0       # left edge of arena
CORRIDOR_ENTRY_Y = 15.0      # approximately centre height

# Default red zones (will be updated on competition day)
DEFAULT_RED_ZONES = [
    [(24, 12), (36, 12), (36, 22), (24, 22)],          # Rectangle
    [(10, 5), (15, 5), (12.5, 10)],                    # Triangle
    [(5, 20), (10, 20), (10, 22), (7, 22), (7, 25), (5, 25)] # L-Shape
]
RED_ZONE_BUFFER_M = 0.5


def compute_strip_width(altitude_m: float = ALTITUDE_M,
                        hfov_deg: float = HFOV_DEG,
                        overlap: float = OVERLAP) -> float:
    """Compute the lawnmower strip width from camera geometry."""
    hfov_rad = np.radians(hfov_deg)
    return 2 * altitude_m * np.tan(hfov_rad / 2) * overlap


def generate_lawnmower(
    flyable_polygon: Polygon,
    strip_width: float,
    arena_width: float = ARENA_WIDTH_M,
) -> list:
    """
    Generate lawnmower (boustrophedon) waypoints over the flyable polygon.

    Parameters
    ----------
    flyable_polygon : Shapely Polygon representing the flyable area
    strip_width : width of each scan strip in metres
    arena_width : total arena width (for slab generation)

    Returns
    -------
    List of (x, y) tuples in local ENU metres
    """
    bounds = flyable_polygon.bounds
    min_y, max_y = bounds[1], bounds[3]

    raw_strips = []
    y = min_y + strip_width / 2
    left_to_right = True

    while y <= max_y:
        slab = Polygon([
            (-1,              y - strip_width / 2),
            (arena_width + 1, y - strip_width / 2),
            (arena_width + 1, y + strip_width / 2),
            (-1,              y + strip_width / 2),
        ])
        intersection = flyable_polygon.intersection(slab)

        if not intersection.is_empty:
            geoms = (intersection.geoms
                     if intersection.geom_type == "MultiPolygon"
                     else [intersection])

            strip_pts = []
            for geom in geoms:
                gx, gy = geom.exterior.xy
                xs = list(gx)
                min_x_g = min(xs)
                max_x_g = max(xs)
                if left_to_right:
                    strip_pts.append((min_x_g, y))
                    strip_pts.append((max_x_g, y))
                else:
                    strip_pts.append((max_x_g, y))
                    strip_pts.append((min_x_g, y))

            raw_strips.append(strip_pts)

        left_to_right = not left_to_right
        y += strip_width

    waypoints = []
    for strip in raw_strips:
        waypoints.extend(strip)

    return waypoints


def local_to_gps(x_m: float, y_m: float,
                 origin_lat: float, origin_lon: float) -> tuple:
    """Convert local ENU (x=East, y=North) offset to GPS coordinates."""
    METRES_PER_DEG_LAT = 111_320.0
    metres_per_deg_lon = 111_320.0 * np.cos(np.radians(origin_lat))
    lat = origin_lat + (y_m / METRES_PER_DEG_LAT)
    lon = origin_lon + (x_m / metres_per_deg_lon)
    return lat, lon


def generate_mission_waypoints(
    origin_lat: float,
    origin_lon: float,
    arena_width: float = ARENA_WIDTH_M,
    arena_height: float = ARENA_HEIGHT_M,
    altitude: float = ALTITUDE_M,
    hfov_deg: float = HFOV_DEG,
    overlap: float = OVERLAP,
    red_zones_vertices: list = None,
    red_zone_buffer: float = RED_ZONE_BUFFER_M,
    entry_x: float = CORRIDOR_ENTRY_X,
    entry_y: float = CORRIDOR_ENTRY_Y,
) -> dict:
    """
    Generate a complete set of mission waypoints.

    Parameters
    ----------
    origin_lat, origin_lon : GPS origin of the arena (0,0 in local frame)
    arena_width, arena_height : arena dimensions in metres
    altitude : flight altitude AGL in metres
    hfov_deg : camera horizontal FOV in degrees
    overlap : strip overlap fraction (0-1)
    red_zones_vertices : list of lists of (x,y) tuples defining the red zone polygons
    red_zone_buffer : buffer distance around red zone in metres
    entry_x, entry_y : corridor entry point in local frame

    Returns
    -------
    dict with keys:
        'local_waypoints' : [(x, y), ...] in local ENU metres
        'gps_waypoints'   : [(lat, lon), ...] in decimal degrees
        'strip_width'     : computed strip width in metres
        'estimated_dist'  : total path distance in metres
        'estimated_time'  : estimated flight time in seconds at ARENA_SPEED_MS
    """
    if red_zones_vertices is None:
        red_zones_vertices = DEFAULT_RED_ZONES

    # Build arena polygon
    arena = Polygon([
        (0, 0),
        (arena_width, 0),
        (arena_width, arena_height),
        (0, arena_height),
    ])

    flyable = arena
    for rz_verts in red_zones_vertices:
        if len(rz_verts) >= 3:
            red_zone = Polygon(rz_verts)
            red_zone_buffered = red_zone.buffer(red_zone_buffer)
            flyable = flyable.difference(red_zone_buffered)

    # Compute strip width
    strip_width = compute_strip_width(altitude, hfov_deg, overlap)

    # Generate lawnmower waypoints
    local_wps = generate_lawnmower(flyable, strip_width, arena_width)

    # Prepend corridor entry point
    entry_wp = (entry_x, entry_y)
    local_wps = [entry_wp] + local_wps

    # Convert to GPS
    gps_wps = [local_to_gps(x, y, origin_lat, origin_lon) for x, y in local_wps]

    # Compute path distance
    estimated_dist = sum(
        np.hypot(local_wps[i+1][0] - local_wps[i][0],
                 local_wps[i+1][1] - local_wps[i][1])
        for i in range(len(local_wps) - 1)
    )

    return {
        'local_waypoints': local_wps,
        'gps_waypoints': gps_wps,
        'strip_width': strip_width,
        'estimated_dist': estimated_dist,
        'estimated_time': estimated_dist / ARENA_SPEED_MS,
    }


# ===================================================================
# Standalone script mode — generate plot + print waypoints
# ===================================================================
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    import os

    ORIGIN_LAT = 12.9716   # placeholder, update on competition day
    ORIGIN_LON = 77.5946

    result = generate_mission_waypoints(ORIGIN_LAT, ORIGIN_LON)
    waypoints = result['local_waypoints']
    gps_waypoints = result['gps_waypoints']
    strip_w = result['strip_width']
    estimated_dist = result['estimated_dist']

    print(" " * 55)
    print("  Mission 2 : Waypoint Generator")
    print(" " * 55)
    print(f"  Arena          : {ARENA_WIDTH_M} m × {ARENA_HEIGHT_M} m")
    print(f"  Altitude       : {ALTITUDE_M} m AGL")
    print(f"  HFOV           : {HFOV_DEG}°")
    print(f"  Strip width    : {strip_w:.2f} m  (with {int(OVERLAP*100)}% overlap)")
    print(f"  Corridor entry : ({CORRIDOR_ENTRY_X}, {CORRIDOR_ENTRY_Y}) m")
    print(f"  Total waypoints: {len(waypoints)}")
    print(f"  Est. path dist : {estimated_dist:.1f} m")
    print(f"  Est. flight time: {estimated_dist / ARENA_SPEED_MS / 60:.1f} min "
          f"@ {ARENA_SPEED_MS} m/s")
    print()

    print("  Waypoints (local ENU metres):")
    for i, (x, y) in enumerate(waypoints):
        print(f"    WP{i:02d}  x={x:6.2f} m   y={y:6.2f} m")

    print()
    print("  GPS waypoints (update ORIGIN_LAT/LON on event day):")
    for i, (lat, lon) in enumerate(gps_waypoints):
        print(f"    WP{i:02d}  lat={lat:.7f}  lon={lon:.7f}")

    # ----- Matplotlib plot -----
    arena = Polygon([
        (0, 0), (ARENA_WIDTH_M, 0),
        (ARENA_WIDTH_M, ARENA_HEIGHT_M), (0, ARENA_HEIGHT_M),
    ])
    
    fig, ax = plt.subplots(figsize=(13, 8))

    xa, ya = arena.exterior.xy
    ax.fill(xa, ya, alpha=0.08, color="green")
    ax.plot(xa, ya, "g-", linewidth=2.5, label="Arena boundary (40×30 m)")

    for i, rz_verts in enumerate(DEFAULT_RED_ZONES):
        rz = Polygon(rz_verts)
        xr, yr = rz.exterior.xy
        ax.fill(xr, yr, alpha=0.6, color="red")
        ax.plot(xr, yr, "r-", linewidth=2)
        
        c = rz.centroid
        ax.text(c.x, c.y, f"Red Zone {i+1}\n(no-fly)", color="white", fontsize=8,
                ha="center", va="center", fontweight="bold")

        rz_buf = rz.buffer(RED_ZONE_BUFFER_M)
        xb, yb = rz_buf.exterior.xy
        label = f"Red zone buffer ({RED_ZONE_BUFFER_M} m)" if i == 0 else ""
        ax.plot(xb, yb, "r--", linewidth=1, alpha=0.5, label=label)

    wx = [p[0] for p in waypoints]
    wy = [p[1] for p in waypoints]
    ax.plot(wx, wy, "b-", linewidth=1.2, alpha=0.7, label="Lawnmower path")
    ax.plot(wx, wy, "b.", markersize=6)

    for i, (x, y) in enumerate(waypoints):
        ax.annotate(f"WP{i}", (x, y), fontsize=6.5,
                    xytext=(3, 3), textcoords="offset points", color="navy")

    ax.plot(CORRIDOR_ENTRY_X, CORRIDOR_ENTRY_Y, "k<", markersize=14,
            label="Corridor entry")
    ax.annotate("Corridor\nentry", (CORRIDOR_ENTRY_X, CORRIDOR_ENTRY_Y),
                xytext=(1.5, 1.5), fontsize=8, color="black")

    ax.annotate("", xy=(40, -2), xytext=(0, -2),
                arrowprops=dict(arrowstyle="<->", color="black"))
    ax.text(20, -3.2, "40 m", ha="center", fontsize=9)
    ax.annotate("", xy=(-2, 30), xytext=(-2, 0),
                arrowprops=dict(arrowstyle="<->", color="black"))
    ax.text(-4.5, 15, "30 m", ha="center", fontsize=9, rotation=90)

    ax.annotate("", xy=(5, strip_w), xytext=(5, 0),
                arrowprops=dict(arrowstyle="<->", color="purple", lw=1.2))
    ax.text(6.5, strip_w / 2, f"Strip\n{strip_w:.1f} m", fontsize=7.5,
            color="purple", va="center")

    ax.set_title(
        f"Mission 2: SkyScan — BCD Lawnmower\n"
        f"Alt={ALTITUDE_M} m | Strip={strip_w:.1f} m | "
        f"{len(waypoints)} waypoints | ~{estimated_dist/ARENA_SPEED_MS/60:.1f} min",
        fontsize=11
    )
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_xlim(-6, 43)
    ax.set_ylim(-5, 33)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)

    plt.tight_layout()
    output_dir = os.path.expanduser("~/ros2_ws/src/outputs")
    os.makedirs(output_dir, exist_ok=True)
    output_file = os.path.join(output_dir, "arena_search_mission2.png")
    plt.savefig(output_file, dpi=150)
    print()
    print(f"  Plot saved → {output_file}")