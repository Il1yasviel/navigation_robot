#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
PC_WORKSPACE="${NAVIGATION_ROBOT_PC_WS:-${HOME}/navigation_robot_pc_ws}"
SOURCE_DIR="${PC_WORKSPACE}/src/navigation_robot_source"
REPOSITORY="https://github.com/Il1yasviel/navigation_robot.git"
PACKAGE_PATH="navigation_robot_ros2/src/navigation_robot_remote_control"

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "找不到 ROS 2 ${ROS_DISTRO}；请先在 Ubuntu 22.04 安装 ros-${ROS_DISTRO}-desktop。" >&2
  exit 2
fi
if ! command -v git >/dev/null || ! command -v colcon >/dev/null; then
  echo "需要先安装 git 和 python3-colcon-common-extensions。" >&2
  exit 2
fi

mkdir -p "${PC_WORKSPACE}/src"
if [[ -d "${SOURCE_DIR}/.git" ]]; then
  git -C "${SOURCE_DIR}" pull --ff-only
elif [[ -e "${SOURCE_DIR}" ]]; then
  echo "${SOURCE_DIR} 已存在但不是 Git 仓库；为避免覆盖，安装已停止。" >&2
  exit 2
else
  git clone --depth 1 --filter=blob:none --sparse "${REPOSITORY}" "${SOURCE_DIR}"
fi
git -C "${SOURCE_DIR}" sparse-checkout set "${PACKAGE_PATH}"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
cd "${PC_WORKSPACE}"
rosdep install --from-paths src --ignore-src -r -y --rosdistro "${ROS_DISTRO}"
colcon build --symlink-install --packages-select navigation_robot_remote_control

echo
echo "安装完成。每个新终端先执行："
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  source ${PC_WORKSPACE}/install/setup.bash"
echo "  export ROS_DOMAIN_ID=30"
echo "  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
echo "然后启动："
echo "  ros2 run navigation_robot_remote_control keyboard_teleop --ros-args --params-file \"\$(ros2 pkg prefix --share navigation_robot_remote_control)/config/remote_control.yaml\""
