#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
[[ -f "$ROOT/robot_ws/install/setup.bash" ]] || { echo 'Execute colcon build em robot_ws antes.' >&2; exit 2; }
source "$ROOT/robot_ws/install/setup.bash"
export GZ_SIM_RESOURCE_PATH="$ROOT/robot_ws/src/robot_description/models:$ROOT/robot_ws/src/robot_description/worlds:${GZ_SIM_RESOURCE_PATH:-}"
export ASPIRADOR_RESULTS_DIR="$ROOT/results"
mkdir -p "$ROOT/results"
rm -f "$ROOT/results/mobility_report.json"
if ros2 launch robot_tests mobility_tests.launch.py output_dir:="$ROOT/results" headless:=true "$@"; then
  launch_status=0
else
  launch_status=$?
fi
if [[ -f "$ROOT/results/mobility_report.json" ]]; then
  python3 -c 'import json,sys; r=json.load(open(sys.argv[1])); print("Mobility tests: %d passed, %d failed" % (r["passed"], r["failed"])); sys.exit(1 if r["failed"] else 0)' "$ROOT/results/mobility_report.json"
else
  echo 'Não foi gerado results/mobility_report.json.' >&2
  exit "${launch_status:-1}"
fi
