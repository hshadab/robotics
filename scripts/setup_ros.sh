#!/usr/bin/env bash
set -euo pipefail

# Installs ROS 2 (Jazzy on Ubuntu 24.04) and required packages for the demo.
# Safe to re-run. Requires sudo.

DISTRO=${ROS_DISTRO:-jazzy}

have_ros() {
  command -v ros2 >/dev/null 2>&1
}

echo "[setup_ros] Using ROS_DISTRO=${DISTRO}"

if ! have_ros; then
  echo "[setup_ros] Installing ROS 2 ${DISTRO} base..."
  sudo apt-get update
  sudo apt-get install -y curl gnupg lsb-release software-properties-common
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  sudo apt-get update
  sudo apt-get install -y ros-${DISTRO}-ros-base
  echo "source /opt/ros/${DISTRO}/setup.bash" | sudo tee -a /etc/skel/.bashrc >/dev/null || true
fi

echo "[setup_ros] Installing demo packages..."
sudo apt-get install -y \
  ros-${DISTRO}-teleop-twist-keyboard \
  ros-${DISTRO}-twist-mux \
  ros-${DISTRO}-image-tools \
  ros-${DISTRO}-rosbag2-storage-mcap \
  python3-tk

echo "[setup_ros] Done. If needed, source /opt/ros/${DISTRO}/setup.bash"

