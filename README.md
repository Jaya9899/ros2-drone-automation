# Aerothon 2026 Mission 2 drone stack

ROS 2 Jazzy + MAVROS + ArduPilot Copter 4.7, on a Raspberry Pi 5 with a Pixhawk 6C Mini,
an OAK-D Lite (forward depth) and an RPi Camera Module 3 (downward, QR codes).

Mission: read the reference QR on the ground → take off → corridor 1 → lawnmower search
for the matching QR → drop the payload on it → corridor 2 (obstacles) → fly home → land.

The corridors are 5 ft (1.524 m) wide; obstacles are 30–40 % of the width. They are
flown by `path_planner` from the OAK-D corridor fit (`corridor_perception_node`):
stay centred, move to the free side of an obstacle, or climb over one that leaves no
side gap. Entrance (green) and exit (blue) are read from the ground markings by the
downward camera. ArduPilot's BendyRuler is off — at this width it cannot pass the
obstacles (`obstacle_avoidance/params/README.md`).

## Packages

| Package | Role | Executables |
|---|---|---|
| `mission_bringup` | **Start here.** Launch files, `config/mission_params.yaml`, SITL script, Gazebo world | `mission.launch.py`, `sitl_mission.launch.py`, `run_sim.sh` |
| `mission_common` | Shared library: parameters (`config_loader`), MAVROS helpers (`mavros_utils`), service names (`srv_helpers`) | — |
| `mission_manager` | Mission state machine and the node that runs it; SITL QR stand-in | `mission_node`, `sitl_qr_simulator`, `flight_primitives` |
| `path_planner` | Flies each leg as GUIDED position targets: lawnmower routed around red zones; corridors followed from perception (`corridor_follow.py`) | `path_planner_node` |
| `payload` | Payload release servo | `payload_node` |
| `qr_detection` | Downward camera: QR detection, confirmation gate, ground projection; corridor entrance/exit colour markers | `qr_scanner_node`, `ground_marker_node` |
| `obstacle_avoidance` | OAK-D depth → 72-bin scan → ArduPilot proximity; sensor-health signals; corridor fit (walls, centring error, obstacles, entrance/exit). SITL depth stand-in. Bench tools in `scripts/`, FC parameter sets in `params/` | `sector_builder_node`, `obstacle_publisher_node`, `coverage_guard_node`, `corridor_perception_node`, `sitl_corridor_sim`, `sector_viz`, `synthetic_sector_node` |
| `depthai-ros` | Luxonis OAK driver, built from source (separate repo) | — |

`mission_manager`, `path_planner`, `payload` and `obstacle_avoidance` depend on
`mission_common` (`corridor_msg.py` defines the `/corridor/perception` layout);
`mission_bringup` depends on everything; `qr_detection` stands alone.

## How the nodes talk

```
mission_node ── /perception/scan_qr, /perception/start_qr_scan ──▶ qr_scanner_node
     │       ── /path_planner/fly_corridor_1|2, run_lawnmower, stop ──▶ path_planner_node ──▶ MAVROS (GUIDED)
     │       ── /payload/release ──▶ payload_node ──▶ MAVROS (DO_SET_SERVO)
     │◀── /qr/detection ("content|lat|lon", after the confirmation gate)
     │◀── /avoidance/sensor_live, /avoidance/coverage_ok ◀── coverage_guard_node
     ▼                                                     sector_builder_node ── /avoidance/scan ──▶ obstacle_publisher_node ──▶ ArduPilot
                     corridor_perception_node ── /corridor/perception ──▶ path_planner_node
                     ground_marker_node ── /corridor/ground_marker ("green"|"blue"|"none") ──▶ path_planner_node
 /mission/status, /mission/telemetry
```

All services are `std_srvs/Trigger`; leg services block until the leg finishes, is
stopped or fails. `mission_node` runs the phases in a worker thread and callbacks only
record data. The behaviour rules (RC CH7 start/abort, QR retries, drop, homing,
sensor-health holds) are documented at the top of `mission_node.py` and
`path_planner_node.py`.

## Build

```bash
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install && source install/setup.bash
```

Python packages not covered by rosdep: `python-statemachine`, `shapely`, `pymavlink`.

## Run

SITL (ArduPilot SITL + MAVROS + mission, each in its own terminal):

```bash
ros2 run mission_bringup run_sim.sh             # or: --mavros-only, then
ros2 launch mission_bringup sitl_mission.launch.py
```

`sitl_mission.launch.py` is the only place the hardware gates are relaxed (no RC
switch, fallback QR ID, 3S battery thresholds). By default (`corridor_sim:=true`)
`sitl_corridor_sim` ray-casts the OAK-D depth and the ground markers into 5 ft
corridors with obstacles (with a deliberate 0.5 m GPS offset), and the real
avoidance and corridor-perception nodes run on it; `corridor_sim:=false` flies the
corridors on the GPS centreline with no depth at all.

Hardware (Pi 5 ↔ Pixhawk on TELEM2):

```bash
ros2 launch mission_bringup mission.launch.py launch_mavros:=true \
    fcu_url:=/dev/ttyAMA0:921600 launch_camera:=true
```

The mission waits for the pilot's RC CH7 switch; switching it low at any time returns
control to the pilot (LOITER).

## Configure

| File | Contents |
|---|---|
| `mission_bringup/config/mission_params.yaml` | Mission settings; the box at the top lists what changes on competition day |
| `obstacle_avoidance/config/avoidance.yaml` | Depth → scan pipeline; corridor perception (width, slice, ranges) |
| `obstacle_avoidance/config/oak_d_lite_depth.yaml` | **Required** OAK-D driver settings (the defaults corrupt obstacle bearings) |
| `obstacle_avoidance/params/*.param` | ArduPilot parameter sets per test stage (see its README) |

## Test

```bash
cd ~/ros2_ws/src
python3 -m pytest mission_manager/test path_planner/test qr_detection/test obstacle_avoidance/test
python3 path_planner/path_planner/waypoint_gen.py   # preview the lawnmower (PNG in cwd)
ros2 run mission_manager flight_primitives          # arm/takeoff/goto/land self-test on SITL
```
