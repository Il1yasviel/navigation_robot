#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"

sudo apt-get update
sudo apt-get install -y \
  "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp" \
  "ros-${ROS_DISTRO}-twist-mux" \
  "ros-${ROS_DISTRO}-teleop-twist-keyboard" \
  "ros-${ROS_DISTRO}-image-transport-plugins" \
  "ros-${ROS_DISTRO}-robot-localization" \
  "ros-${ROS_DISTRO}-xacro" \
  python3-rosdep python3-vcstool

# ROS 2's generated setup scripts probe optional variables that may be unset.
# Temporarily disable nounset while sourcing them, then restore strict mode.
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

if [[ ! -d /etc/ros/rosdep/sources.list.d ]]; then
  sudo rosdep init
fi
rosdep update
rosdep install \
  --from-paths "${WORKSPACE}/src" \
  --ignore-src \
  --skip-keys gazebo_ros_pkgs \
  -r -y \
  --rosdistro "${ROS_DISTRO}"
