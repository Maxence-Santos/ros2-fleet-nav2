#!/usr/bin/env bash
# Pure tests. They do not start Gazebo. /usr/bin/python3 is the ROS interpreter
# and already has numpy, PyYAML, and Pillow.
set -euo pipefail
cd "$(dirname "$0")"
export PYTHONPATH="${PWD}/src/fleet_nav${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 -m unittest discover -s tests -v
