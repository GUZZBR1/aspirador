#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
[[ -f "$ROOT/robot_ws/install/setup.bash" ]] || { echo 'Execute ./scripts/build_ws.sh antes.' >&2; exit 2; }
source "$ROOT/robot_ws/install/setup.bash"
export GZ_SIM_RESOURCE_PATH="$ROOT/robot_ws/src/robot_description/models:$ROOT/robot_ws/src/robot_description/worlds:${GZ_SIM_RESOURCE_PATH:-}"
gazebo_pids() {
  ps -eo pid=,comm=,args= | awk '$2 == "ruby" && /gz sim/ { print $1 }'
}
existing_gz_pids=" $(gazebo_pids | tr '\n' ' ') "
cleanup_new_gazebo() {
  local pid
  while read -r pid; do
    [[ -n "$pid" ]] || continue
    [[ "$existing_gz_pids" == *" $pid "* ]] && continue
    kill -TERM "$pid" 2>/dev/null || true
  done < <(gazebo_pids)
}
trap cleanup_new_gazebo EXIT
mkdir -p "$ROOT/results"
headless_arg='headless:=false'
if [[ -z "${WAYLAND_DISPLAY:-}" && -z "${DISPLAY:-}" ]]; then
  headless_arg='headless:=true'
fi
ros2 launch robot_tests battery_cycle.launch.py output_dir:="$ROOT/results" "$headless_arg"
launch_status=$?
cleanup_new_gazebo
if [[ -f "$ROOT/results/battery_report.json" ]]; then
  python3 -c 'import json,sys; r=json.load(open(sys.argv[1])); print("Battery cycle: %s; states=%s" % ("PASS" if r["passed"] else "FAIL", " -> ".join(r["states"]))); sys.exit(0 if r["passed"] else 1)' "$ROOT/results/battery_report.json"
else
  echo 'Não foi gerado results/battery_report.json.' >&2
  exit "${launch_status:-1}"
fi
