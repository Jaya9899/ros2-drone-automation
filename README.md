# SkyScan — Aerothon 2026 Mission 2 drone stack

ROS 2 Jazzy + MAVROS + ArduPilot (Copter 4.7). Companion computer: Raspberry Pi 5;
flight controller: Pixhawk 6C Mini; sensors: OAK-D Lite (forward depth) and
RPi Camera Module 3 (downward, QR codes).

Mission: scan the reference QR on the ground → take off → corridor 1 →
lawnmower search of the arena for the matching QR → drop the payload on it →
corridor 2 (static obstacles, BendyRuler) → fly home → land.

## Packages

| Package | What it is | Entry points |
|---|---|---|
| `skyscan_bringup` | **Start here.** Launch files, `config/mission_params.yaml`, SITL script, Gazebo world | `mission.launch.py`, `sitl_mission.launch.py`, `run_sim.sh` |
| `skyscan_common` | Shared library: mission parameters (`config_loader`), MAVROS helpers (`mavros_utils`), service names + `call_trigger` (`srv_helpers`) | — |
| `mission_manager` | Mission state machine and the node that runs it; safety/telemetry monitors; SITL QR stand-in | `mission_node`, `safety_monitor`, `telemetry_node`, `sitl_qr_simulator`, `flight_primitives` (Gate 1 self-test) |
| `path_planner` | Flies each leg as GUIDED waypoints; lawnmower generation routed around red zones | `path_planner_node` |
| `payload` | Release servo via `MAV_CMD_DO_SET_SERVO` | `payload_node` |
| `perception` | Downward-camera QR detection, confirmation gate, ground projection | `qr_scanner_node` |
| `skyscan_avoidance` | OAK-D depth → 72-bin scan → ArduPilot `OBSTACLE_DISTANCE`; sensor-health Bools; bench/gate tools in `scripts/`, FC param sets in `params/` | `sector_builder_node`, `obstacle_publisher_node`, `coverage_guard_node`, `sector_viz`, `synthetic_sector_node` |
| `depthai-ros` | Vendored Luxonis driver, built from source (not part of this repo) | — |

Dependency direction: `mission_manager`, `path_planner`, `payload` → `skyscan_common`;
`skyscan_bringup` → everything. `perception` and `skyscan_avoidance` are standalone.

## How the nodes talk

```
                       Trigger services                         MAVROS
mission_node ──/perception/scan_qr, start_qr_scan──▶ qr_scanner_node
     │       ──/path_planner/fly_corridor_1|2, run_lawnmower, stop──▶ path_planner_node ──▶ GUIDED setpoints
     │       ──/payload/release──▶ payload_node ──▶ DO_SET_SERVO
     │◀── /qr/detection  ("content|lat|lon", only after the confirmation gate)
     │◀── /avoidance/sensor_live, /avoidance/coverage_ok ◀── coverage_guard_node
     ▼                                                          ▲
 /mission/status, /mission/telemetry          sector_builder_node ── /avoidance/scan ──▶ obstacle_publisher_node ──▶ ArduPilot proximity
```

`mission_node` runs the phases in a worker thread; ROS callbacks only record data.
Leg services block until the leg finishes, is cancelled (`/path_planner/stop`) or fails.
Behaviour decisions (RC CH7 start/abort switch, QR retry, homing, drop, sensor-health
holds) are documented at the top of `mission_manager/mission_node.py` and
`path_planner/path_planner_node.py`.

## Build

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Python deps outside rosdep: `python-statemachine`, `shapely`, `pymavlink`.

## Run

**SITL** (ArduPilot SITL + MAVROS + mission stack, each in its own terminal):

```bash
ros2 run skyscan_bringup run_sim.sh            # or --mavros-only, then:
ros2 launch skyscan_bringup sitl_mission.launch.py
```

SITL relaxes the hardware gates (no RC switch, fallback QR ID, no OAK-D, 3S battery
thresholds) — only in `sitl_mission.launch.py`.

**Hardware** (Pi 5 ↔ Pixhawk on TELEM2):

```bash
ros2 launch skyscan_bringup mission.launch.py launch_mavros:=true \
    fcu_url:=/dev/ttyAMA0:921600 launch_camera:=true
```

The mission waits for the pilot's RC CH7 switch; flipping it low at any time hands
control back to the pilot (LOITER).

## Configure

- `skyscan_bringup/config/mission_params.yaml` — everything mission-level. The box at
  the top lists what must change on competition day (origin, geofence, red zones,
  corridor waypoints).
- `skyscan_avoidance/config/avoidance.yaml` — depth → scan pipeline.
- `skyscan_avoidance/config/oak_d_lite_depth.yaml` — **required** OAK-D driver params
  (the driver defaults silently corrupt obstacle bearings).
- `skyscan_avoidance/params/*.param` — ArduPilot parameter sets per test stage
  (see `skyscan_avoidance/params/README.md`).

## Test

```bash
cd ~/ros2_ws/src
python3 -m pytest mission_manager/test/test_state_machine.py \
    path_planner/test/test_waypoint_gen.py perception/test/test_qr_pipeline.py \
    skyscan_avoidance/test
python3 path_planner/path_planner/waypoint_gen.py   # preview the lawnmower (PNG in cwd)
ros2 run mission_manager flight_primitives          # Gate 1: arm/takeoff/goto/land x5 on SITL
```
