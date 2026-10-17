#!/usr/bin/env bash
# Three AMRs on the saved map.
#   ./start_fleet_demo.sh          # Gazebo, no window
#   ./start_fleet_demo.sh --gui    # Gazebo + RViz. Click a robot, then its goal.
set -euo pipefail
cd "$(dirname "$0")"

MAP_YAML="${PWD}/src/fleet_nav/maps/warehouse_map.yaml"
MAP_PGM="${PWD}/src/fleet_nav/maps/warehouse_map.pgm"
if [[ ! -f "$MAP_YAML" || ! -f "$MAP_PGM" ]]; then
  echo "No saved map in src/fleet_nav/maps/."
  echo "  ./start_slam_demo.sh"
  echo "  python3 save_map.py"
  exit 1
fi

set +u
source /opt/ros/lyrical/setup.bash
set -u

if [[ ! -f install/setup.bash ]]; then
  colcon build --packages-select fleet_nav --symlink-install
fi
set +u
source install/setup.bash
set -u

headless=true
rviz=false
if [[ "${1:-}" == "--gui" ]]; then
  headless=false
  rviz=true
fi

echo "Map loaded: $MAP_YAML"
if [[ "$rviz" == true ]]; then
  echo "In RViz, use Publish Point: click a robot, then its goal."
  echo "A click within 0.7 m of a robot selects it."
fi

exec ros2 launch fleet_nav fleet.launch.py \
  headless:="${headless}" \
  rviz:="${rviz}" \
  map_yaml:="${MAP_YAML}"
