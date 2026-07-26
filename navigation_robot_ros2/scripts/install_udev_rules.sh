#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_RULE="${WORKSPACE}/src/navigation_robot_base_driver/scripts/99-navigation-base.rules"
LIDAR_RULE="${WORKSPACE}/src/navigation_robot_lidar_driver/scripts/99-navigation-lidar.rules"

if grep -q 'idVendor}=="0000"' "${BASE_RULE}"; then
  echo "底盘 udev 规则仍是 0000:0000 占位值；请先用 lsusb/udevadm 填写 VID/PID/serial。" >&2
  exit 2
fi

sudo install -m 0644 "${BASE_RULE}" /etc/udev/rules.d/99-navigation-base.rules
sudo install -m 0644 "${LIDAR_RULE}" /etc/udev/rules.d/99-navigation-lidar.rules
sudo usermod -aG dialout,video "${USER}"
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "规则已安装。用户组变更需重新登录后生效。相机规则请使用最终选定驱动自带的安装脚本。"
