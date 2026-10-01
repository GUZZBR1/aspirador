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
scenario_name=''
scenario_limit=8
run_control_tests=true
warmup_seconds=120.0
warmup_explicit=false
headless_override=''
extra_args=()
for arg in "$@"; do
  case "$arg" in
    scenario_name:=*) scenario_name="${arg#scenario_name:=}" ;;
    scenario_limit:=*) scenario_limit="${arg#scenario_limit:=}" ;;
    run_control_tests:=*) run_control_tests="${arg#run_control_tests:=}" ;;
    warmup_seconds:=*) warmup_seconds="${arg#warmup_seconds:=}"; warmup_explicit=true ;;
    headless:=*) headless_override="${arg#headless:=}" ;;
    *) extra_args+=("$arg") ;;
  esac
done
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
headless_arg='headless:=false'
if [[ -n "$headless_override" ]]; then
  headless_arg="headless:=$headless_override"
elif [[ -z "$scenario_name" || ( -z "${WAYLAND_DISPLAY:-}" && -z "${DISPLAY:-}" ) ]]; then
  headless_arg='headless:=true'
fi
run_launch() {
  local status=0
  ros2 launch robot_tests mobility_tests.launch.py output_dir:="$ROOT/results" "$headless_arg" "$@" || status=$?
  cleanup_new_gazebo
  return "$status"
}

if [[ -n "$scenario_name" ]]; then
  run_launch "scenario_name:=$scenario_name" "run_control_tests:=$run_control_tests" \
    "warmup_seconds:=$warmup_seconds" "${extra_args[@]}" || true
  if [[ ! -f "$ROOT/results/mobility_report.json" ]]; then
    echo 'Não foi gerado results/mobility_report.json.' >&2
    exit 1
  fi
else
  case_dir="$ROOT/results/mobility_cases"
  mkdir -p "$case_dir"
  rm -f "$case_dir"/*.json
  mapfile -t scenario_names < <(python3 - "$ROOT/robot_ws/src/robot_tests/scenarios/mobility.json" "$scenario_limit" <<'PY'
import json, sys
with open(sys.argv[1], encoding='utf-8') as stream:
    scenarios = json.load(stream)
limit = max(0, int(sys.argv[2]))
for item in scenarios[:limit]:
    print(item['name'])
PY
)

  if [[ "$run_control_tests" == true ]]; then
    # Run probes at the known spawn pose, before autonomy can move the robot.
    control_warmup=$warmup_seconds
    if [[ "$warmup_explicit" == false ]]; then control_warmup=20.0; fi
    # scenario_limit=0 runs the three control probes without a mobility case.
    run_launch scenario_limit:=0 run_control_tests:=true "warmup_seconds:=$control_warmup" \
      "${extra_args[@]}" || true
    if [[ -f "$ROOT/results/mobility_report.json" ]]; then
      cp "$ROOT/results/mobility_report.json" "$case_dir/control_tests.json"
    fi
  fi

  for name in "${scenario_names[@]}"; do
    run_launch "scenario_name:=$name" run_control_tests:=false \
      "warmup_seconds:=$warmup_seconds" "${extra_args[@]}" || true
    if [[ -f "$ROOT/results/mobility_report.json" ]]; then
      cp "$ROOT/results/mobility_report.json" "$case_dir/$name.json"
    else
      echo "Não foi gerado relatório para o cenário $name." >&2
    fi
  done

  python3 - "$case_dir" "$ROOT/results/mobility_report.json" "${scenario_names[@]}" <<'PY'
import json, os, sys, time
case_dir, report_path, *names = sys.argv[1:]
rows = []
for filename in ('control_tests.json', *(name + '.json' for name in names)):
    path = os.path.join(case_dir, filename)
    if os.path.isfile(path):
        with open(path, encoding='utf-8') as stream:
            rows.extend(json.load(stream).get('scenarios', []))
    elif filename != 'control_tests.json':
        rows.append({'scenario': filename[:-5], 'passed': False,
                     'outcome': 'runner_missing_report', 'observed': 'runner_missing_report'})
report = {
    'suite': 'mvp1_mobility',
    'timestamp_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
    'passed': sum(bool(row.get('passed')) for row in rows),
    'failed': sum(not bool(row.get('passed')) for row in rows),
    'scenarios': rows,
    'notes': ('Control probes run once; every configured mobility scenario runs in a fresh Gazebo, '
              'SLAM and Nav2 instance. This avoids carrying localization, costmap and robot pose '
              'between independent scenarios. Per-run reports are in results/mobility_cases/.'),
}
with open(report_path, 'w', encoding='utf-8') as stream:
    json.dump(report, stream, ensure_ascii=False, indent=2)
print('Mobility tests: %d passed, %d failed' % (report['passed'], report['failed']))
sys.exit(1 if report['failed'] else 0)
PY
  exit $?
fi

python3 -c 'import json,sys; r=json.load(open(sys.argv[1])); print("Mobility tests: %d passed, %d failed" % (r["passed"], r["failed"])); sys.exit(1 if r["failed"] else 0)' "$ROOT/results/mobility_report.json"
