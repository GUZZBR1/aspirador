#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
[[ -f "$ROOT/robot_ws/install/setup.bash" ]] || { echo 'Execute colcon build em robot_ws antes.' >&2; exit 2; }
source "$ROOT/robot_ws/install/setup.bash"
export GZ_SIM_RESOURCE_PATH="$ROOT/robot_ws/src/robot_description/models:$ROOT/robot_ws/src/robot_description/worlds:${GZ_SIM_RESOURCE_PATH:-}"
export ASPIRADOR_RESULTS_DIR="$ROOT/results"
if ! printf '%s\n' "$@" | grep -Eq '^headless:=true$' && [[ -z "${WAYLAND_DISPLAY:-}" && -z "${DISPLAY:-}" ]]; then
  cat >&2 <<'EOF'
Não foi detectado display gráfico para Gazebo/RViz.
No WSL2, use WSLg (WAYLAND_DISPLAY/DISPLAY configurados) ou inicie um X server
e configure DISPLAY. Para executar sem interface gráfica, use headless:=true.
EOF
  exit 2
fi
if printf '%s\n' "$@" | grep -q '^slam:=false$' && ! printf '%s\n' "$@" | grep -Eq '^map:=.+$'; then
  echo 'Modo de mapa salvo requer map:=/caminho/para/mapa.yaml.' >&2
  exit 2
fi
exec ros2 launch robot_simulation mvp.launch.py "$@"
