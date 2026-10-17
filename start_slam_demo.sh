#!/usr/bin/env bash
# Cartographie de l'entrepôt avec robot_1 seulement.
# Gazebo + RViz + slam_toolbox, puis le clavier.
# Dans un autre terminal, une fois la zone explorée :
#   source /opt/ros/lyrical/setup.bash
#   python3 save_map.py
set -euo pipefail
cd "$(dirname "$0")"

SLAM_SETUP="$(cd .. && pwd)/ros2-slam-nav2/install/setup.bash"
if [[ ! -f "$SLAM_SETUP" ]]; then
  echo "slam_toolbox n'est pas dans l'installation ROS de base."
  echo "Il est construit dans ros2-slam-nav2. Le dossier install est absent :"
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
  echo "Arrêt de la cartographie."
  kill 0 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Gazebo, RViz et robot_1."
ros2 launch fleet_nav mapping.launch.py &

echo "Attente de slam_toolbox..."
until ros2 service list 2>/dev/null | grep -q "/slam_toolbox/change_state"; do
  sleep 0.5
done
ros2 lifecycle set /slam_toolbox configure || true
sleep 1
ros2 lifecycle set /slam_toolbox activate || true

echo "=========================================================="
echo "Un seul robot (robot_1). Conduis-le pour remplir la carte."
echo "  i avant    k stop    j gauche    l droite    , arrière"
echo "Tape ces touches dans CE terminal. Gazebo et RViz ne les lisent pas."
echo "Si la fenêtre Gazebo est fermée, le robot n'existe plus."
echo "Quand la zone utile est couverte, dans un autre terminal :"
echo "  source /opt/ros/lyrical/setup.bash"
echo "  python3 save_map.py"
echo "Puis ./start_fleet_demo.sh --gui charge cette carte."
echo "=========================================================="

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot_1/cmd_vel
