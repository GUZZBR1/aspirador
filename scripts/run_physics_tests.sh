#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
[[ -f "$ROOT/robot_ws/install/setup.bash" ]] || { echo 'Execute ./scripts/build_ws.sh antes.' >&2; exit 2; }
source "$ROOT/robot_ws/install/setup.bash"
set -u
export GZ_SIM_RESOURCE_PATH="$ROOT/robot_ws/src/robot_description/models:$ROOT/robot_ws/src/robot_description/worlds:${GZ_SIM_RESOURCE_PATH:-}"
mkdir -p "$ROOT/results"
rm -f "$ROOT/results/threshold_report.json" "$ROOT/results/bed_clearance_report.json"

gazebo_pids() {
  ps -eo pid=,comm=,args= | awk '$2 == "ruby" && /gz sim/ { print $1 }'
}

existing_gz_pids=" $(gazebo_pids | tr '\n' ' ') "
headless_arg='headless:=false'
if [[ -z "${WAYLAND_DISPLAY:-}" && -z "${DISPLAY:-}" ]]; then
  headless_arg='headless:=true'
fi
cleanup_fixture_gz() {
  local pid
  while read -r pid; do
    [[ -n "$pid" ]] || continue
    [[ "$existing_gz_pids" == *" $pid "* ]] && continue
    kill -TERM "$pid" 2>/dev/null || true
  done < <(gazebo_pids)
}
trap cleanup_fixture_gz EXIT

if ros2 launch robot_tests physics_tests.launch.py scenario:=threshold output_dir:="$ROOT/results" "$headless_arg"; then
  threshold_status=0
else
  threshold_status=$?
fi
cleanup_fixture_gz
if ros2 launch robot_tests physics_tests.launch.py scenario:=bed output_dir:="$ROOT/results" "$headless_arg"; then
  bed_status=0
else
  bed_status=$?
fi
cleanup_fixture_gz
python3 - "$ROOT/results" <<'PY'
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
reports = [
    ('threshold_report.json', 'soleira de 12 mm'),
    ('bed_clearance_report.json', 'passagem sob a cama'),
]
failed = False
for name, label in reports:
    path = root / name
    if not path.is_file():
        print(f'{label}: relatório ausente ({path})')
        failed = True
        continue
    report = json.loads(path.read_text(encoding='utf-8'))
    passed = report.get('passed') is True
    reason = report.get('reason', report.get('failure', 'sem falha reportada'))
    print(f'{label}: {"PASS" if passed else "FAIL"} ({reason})')
    failed |= not passed
raise SystemExit(1 if failed else 0)
PY
test "$threshold_status" -eq 0 && test "$bed_status" -eq 0
