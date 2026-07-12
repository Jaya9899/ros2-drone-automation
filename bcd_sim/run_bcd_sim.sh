#!/bin/bash
# ─────────────────────────────────────────────────────────────
# run_bcd_sim.sh — Run BCD simulation on ArduPilot SITL
#
# Opens 3 terminals:
#   1. ArduPilot SITL
#   2. MAVROS
#   3. BCD Mission Node + RViz2
#
# Usage:
#   chmod +x run_bcd_sim.sh
#   ./run_bcd_sim.sh
# ─────────────────────────────────────────────────────────────
set -e

ARDUPILOT_DIR="$HOME/ardupilot"
ROS2_WS="$HOME/ros2_ws"
BCD_NODE="$ROS2_WS/src/bcd_sim/bcd_sim/bcd_mission_node.py"
RVIZ_CFG="$ROS2_WS/src/bcd_sim/bcd_sim/bcd_rviz.rviz"

echo "═══════════════════════════════════════════════════════"
echo "  BCD Lawnmower Simulation"
echo "═══════════════════════════════════════════════════════"

# Terminal 1: ArduPilot SITL
echo "► Starting ArduPilot SITL..."
gnome-terminal --title="SITL" -- bash -c "
cd ${ARDUPILOT_DIR}/Tools/autotest
python3 sim_vehicle.py -v ArduCopter -f quad --no-mavproxy \
    -l 12.9716,77.5946,0,0
exec bash" 2>/dev/null

sleep 10
echo "  SITL started"

# Terminal 2: MAVROS
echo "► Starting MAVROS..."
gnome-terminal --title="MAVROS" -- bash -c "
source /opt/ros/humble/setup.bash
source ${ROS2_WS}/install/setup.bash
ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5760
exec bash" 2>/dev/null

sleep 6
echo "  MAVROS started"

# Terminal 3: BCD Node + RViz2
echo "► Starting BCD Mission Node + RViz2..."
gnome-terminal --title="BCD Mission" -- bash -c "
source /opt/ros/humble/setup.bash
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
echo "  In RViz2:"
echo "    - Blue path  = planned BCD lawnmower"
echo "    - Green trail = where drone has flown"
echo "    - Green sphere = live drone position"
echo "    - Yellow cylinder = next waypoint target"
echo "    - Red lines = no-fly zones"
echo ""
echo "  The drone will auto-arm, takeoff, and fly the pattern."
echo "═══════════════════════════════════════════════════════"
