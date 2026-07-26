#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET_DIR="${HOME}/.config/systemd/user"
TARGET="${TARGET_DIR}/navigation_robot.service"

mkdir -p "${TARGET_DIR}"
sed "s|@WORKSPACE@|${WORKSPACE}|g" \
  "${WORKSPACE}/deploy/navigation_robot.service.in" > "${TARGET}"
systemctl --user daemon-reload

echo "已安装但未启用：${TARGET}"
echo "完成实车标定后可执行：systemctl --user enable --now navigation_robot.service"
