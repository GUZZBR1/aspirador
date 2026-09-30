#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
cd "$ROOT/robot_ws"
rosdep install --from-paths src --ignore-src -r -y --skip-keys=ament_python
colcon build --cmake-clean-cache --symlink-install --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3
echo "Workspace compilado. Use: source $ROOT/robot_ws/install/setup.bash"
