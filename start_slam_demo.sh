#!/usr/bin/env bash
# Map the warehouse with robot_1 only.
# Gazebo + RViz + slam_toolbox, then the keyboard.
# In another terminal, once the area is covered:
#   source /opt/ros/lyrical/setup.bash
#   python3 save_map.py
set -euo pipefail
cd "$(dirname "$0")"

SLAM_SETUP="$(cd .. && pwd)/ros2-slam-nav2/install/setup.bash"
if [[ ! -f "$SLAM_SETUP" ]]; then
  echo "slam_toolbox is not in the base ROS install."
  echo "It is built in ros2-slam-nav2. The install directory is missing:"
  echo "  $SLAM_SETUP"
  exit 1
fi

set +u
source /opt/ros/lyrical/setup.bash
set -u
if [[ ! -f install/setup.bash ]]; then
  colcon build --packages-select fleet_nav --symlink-install
fi
set +u
source "$SLAM_SETUP"
source install/local_setup.bash
set -u

cleanup() {
  echo ""
  echo "Stopping mapping."
  kill 0 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Gazebo, RViz, and robot_1."
ros2 launch fleet_nav mapping.launch.py &

echo "Waiting for slam_toolbox..."
until ros2 service list 2>/dev/null | grep -q "/slam_toolbox/change_state"; do
  sleep 0.5
done
ros2 lifecycle set /slam_toolbox configure || true
sleep 1
ros2 lifecycle set /slam_toolbox activate || true

echo "=========================================================="
echo "One robot (robot_1). Drive it to fill the map."
echo "  i forward    k stop    j left    l right    , back"
echo "Type those keys in THIS terminal. Gazebo and RViz do not read them."
echo "If the Gazebo window is closed, the robot is gone."
echo "When the useful area is covered, in another terminal:"
echo "  source /opt/ros/lyrical/setup.bash"
echo "  python3 save_map.py"
echo "Then ./start_fleet_demo.sh --gui loads that map."
echo "=========================================================="

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot_1/cmd_vel
