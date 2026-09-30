#!/usr/bin/env bash
set -euo pipefail

if [[ ! -f /etc/os-release ]] || ! grep -q 'VERSION_CODENAME=noble' /etc/os-release; then
  echo 'Este instalador requer Ubuntu 24.04 (Noble).' >&2
  exit 2
fi
if ! command -v sudo >/dev/null 2>&1; then
  echo 'sudo é necessário para instalar os pacotes do sistema.' >&2
  exit 2
fi

# Configure the signed ROS repository before `apt-get update`. Updating first
# fails on a fresh install because APT refuses the ROS repository without its key.
if ! command -v curl >/dev/null 2>&1; then
  echo 'curl é necessário para baixar a chave oficial do repositório ROS.' >&2
  exit 2
fi
sudo install -d -m 0755 /usr/share/keyrings
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
  | sudo tee /usr/share/keyrings/ros-archive-keyring.asc >/dev/null
sudo chmod 0644 /usr/share/keyrings/ros-archive-keyring.asc
. /etc/os-release
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.asc] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
sudo apt-get update
sudo apt-get install -y curl gnupg lsb-release software-properties-common
sudo add-apt-repository -y universe
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  python3-colcon-common-extensions \
  python3-rosdep
sudo rosdep init 2>/dev/null || true
rosdep update
echo 'Dependências instaladas. Abra um shell novo ou source /opt/ros/jazzy/setup.bash.'
