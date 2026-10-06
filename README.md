# Aerothon 2026 Mission 2 drone stack

ROS 2 Jazzy + MAVROS + ArduPilot Copter 4.7, on a Raspberry Pi 5 with a Pixhawk 6C Mini,
an OAK-D Lite (forward depth) and an RPi Camera Module 3 (downward, QR codes).

Mission: read the reference QR on the ground → take off → corridor 1 → lawnmower search
for the matching QR → drop the payload on it → corridor 2 (obstacles, BendyRuler) →
fly home → land.

## Packages

| Package | Role | Executables |
|---|---|---|
| `mission_bringup` | **Start here.** Launch files, `config/mission_params.yaml`, SITL script, Gazebo world | `mission.launch.py`, `sitl_mission.launch.py`, `run_sim.sh` |
| `mission_common` | Shared library: parameters (`config_loader`), MAVROS helpers (`mavros_utils`), service names (`srv_helpers`) | — |
| `mission_manager` | Mission state machine and the node that runs it; safety and telemetry monitors; SITL QR stand-in | `mission_node`, `safety_monitor`, `telemetry_node`, `sitl_qr_simulator`, `flight_primitives` |
| `path_planner` | Flies each leg as GUIDED waypoints; lawnmower routed around red zones | `path_planner_node` |
| `payload` | Payload release servo | `payload_node` |
| `qr_detection` | Downward-camera QR detection, confirmation gate, ground projection | `qr_scanner_node` |
| `obstacle_avoidance` | OAK-D depth → 72-bin scan → ArduPilot proximity; sensor-health signals. Bench tools in `scripts/`, FC parameter sets in `params/` | `sector_builder_node`, `obstacle_publisher_node`, `coverage_guard_node`, `sector_viz`, `synthetic_sector_node` |
| `depthai-ros` | Luxonis OAK driver, built from source (separate repo) | — |

`mission_manager`, `path_planner` and `payload` depend on `mission_common`;
`mission_bringup` depends on everything; `qr_detection` and `obstacle_avoidance` stand alone.

## How the nodes talk

```
mission_node ── /perception/scan_qr, /perception/start_qr_scan ──▶ qr_scanner_node
     │       ── /path_planner/fly_corridor_1|2, run_lawnmower, stop ──▶ path_planner_node ──▶ MAVROS (GUIDED)
     │       ── /payload/release ──▶ payload_node ──▶ MAVROS (DO_SET_SERVO)
     │◀── /qr/detection ("content|lat|lon", after the confirmation gate)
     │◀── /avoidance/sensor_live, /avoidance/coverage_ok ◀── coverage_guard_node
     ▼                                                     sector_builder_node ── /avoidance/scan ──▶ obstacle_publisher_node ──▶ ArduPilot
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
switch, fallback QR ID, no OAK-D, 3S battery thresholds).

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
| `obstacle_avoidance/config/avoidance.yaml` | Depth → scan pipeline |
| `obstacle_avoidance/config/oak_d_lite_depth.yaml` | **Required** OAK-D driver settings (the defaults corrupt obstacle bearings) |
| `obstacle_avoidance/params/*.param` | ArduPilot parameter sets per test stage (see its README) |

## Test

```bash
cd ~/ros2_ws/src
python3 -m pytest mission_manager/test path_planner/test qr_detection/test obstacle_avoidance/test
python3 path_planner/path_planner/waypoint_gen.py   # preview the lawnmower (PNG in cwd)
ros2 run mission_manager flight_primitives          # arm/takeoff/goto/land self-test on SITL
```
