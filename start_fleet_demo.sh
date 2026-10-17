#!/usr/bin/env bash
# Trois AMR sur la carte enregistrée.
#   ./start_fleet_demo.sh          # Gazebo sans fenêtre
#   ./start_fleet_demo.sh --gui    # Gazebo + RViz. Clic robot, puis clic cible.
set -euo pipefail
cd "$(dirname "$0")"

MAP_YAML="${PWD}/src/fleet_nav/maps/warehouse_map.yaml"
MAP_PGM="${PWD}/src/fleet_nav/maps/warehouse_map.pgm"
if [[ ! -f "$MAP_YAML" || ! -f "$MAP_PGM" ]]; then
  echo "Pas de carte enregistrée dans src/fleet_nav/maps/."
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

echo "Carte chargée : $MAP_YAML"
if [[ "$rviz" == true ]]; then
  echo "Dans RViz, outil Publish Point : clique un robot, puis sa cible."
  echo "Un clic à moins de 0,7 m d'un robot le sélectionne."
fi

exec ros2 launch fleet_nav fleet.launch.py \
  headless:="${headless}" \
  rviz:="${rviz}" \
  map_yaml:="${MAP_YAML}"
