#!/bin/bash
# ─────────────────────────────────────────────────────────────
# run_px4_sim.sh — Run BCD simulation on PX4 SITL + Gazebo
#
# Opens 3 terminals:
#   1. PX4 SITL (Gazebo 3D)
#   2. MAVROS
#   3. PX4 BCD Mission Node + RViz2
#
# Usage:
#   chmod +x run_px4_sim.sh
#   ./run_px4_sim.sh
# ─────────────────────────────────────────────────────────────
set -e

PX4_DIR="$HOME/PX4-Autopilot"
ROS2_WS="$HOME/ros2_ws"
BCD_NODE="$ROS2_WS/src/bcd_sim/bcd_sim/px4_bcd_node.py"
RVIZ_CFG="$ROS2_WS/src/bcd_sim/bcd_sim/bcd_rviz.rviz"

export QT_QPA_PLATFORM=wayland
export GZ_GUI_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gz-gui-7/plugins

echo "═══════════════════════════════════════════════════════"
echo "  BCD Lawnmower Simulation (PX4 + Gazebo)"
echo "═══════════════════════════════════════════════════════"

# Terminal 1: PX4 SITL + Gazebo
echo "► Starting PX4 SITL with Gazebo..."
gnome-terminal --title="PX4 Gazebo" -- bash -c "
export QT_QPA_PLATFORM=wayland
export GZ_GUI_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gz-gui-7/plugins
cd ${PX4_DIR}
make px4_sitl gz_x500
exec bash" 2>/dev/null

sleep 10
echo "  PX4 started"

# Terminal 2: MAVROS
echo "► Starting MAVROS..."
gnome-terminal --title="MAVROS" -- bash -c "
source /opt/ros/jazzy/setup.bash
source ${ROS2_WS}/install/setup.bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14580 -p gcs_url:=udp://@127.0.0.1:14550
exec bash" 2>/dev/null

sleep 6
echo "  MAVROS started"

# Terminal 3: BCD Node + RViz2
echo "► Starting PX4 BCD Mission Node + RViz2..."
gnome-terminal --title="BCD Mission" -- bash -c "
source /opt/ros/jazzy/setup.bash
source ${ROS2_WS}/install/setup.bash
echo 'Starting BCD mission node...'
python3 ${BCD_NODE} &
sleep 2
echo 'Starting RViz2...'
rviz2 -d ${RVIZ_CFG} 2>/dev/null &
wait
exec bash" 2>/dev/null

echo ""
echo "═══════════════════════════════════════════════════════"
echo "  All 3 terminals launched!"
echo ""
echo "  - Gazebo will show the 3D drone."
echo "  - RViz2 will show the top-down BCD path and telemetry."
echo "  - The drone will AUTO-ARM, TAKEOFF, and FLY."
echo "═══════════════════════════════════════════════════════"
