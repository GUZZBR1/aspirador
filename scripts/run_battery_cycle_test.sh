#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
[[ -f "$ROOT/robot_ws/install/setup.bash" ]] || { echo 'Execute ./scripts/build_ws.sh antes.' >&2; exit 2; }
source "$ROOT/robot_ws/install/setup.bash"
export GZ_SIM_RESOURCE_PATH="$ROOT/robot_ws/src/robot_description/models:$ROOT/robot_ws/src/robot_description/worlds:${GZ_SIM_RESOURCE_PATH:-}"
mkdir -p "$ROOT/results"
ros2 launch robot_tests battery_cycle.launch.py output_dir:="$ROOT/results"
launch_status=$?
if [[ -f "$ROOT/results/battery_report.json" ]]; then
  python3 -c 'import json,sys; r=json.load(open(sys.argv[1])); print("Battery cycle: %s; states=%s" % ("PASS" if r["passed"] else "FAIL", " -> ".join(r["states"]))); sys.exit(0 if r["passed"] else 1)' "$ROOT/results/battery_report.json"
else
  echo 'Não foi gerado results/battery_report.json.' >&2
  exit "${launch_status:-1}"
fi
