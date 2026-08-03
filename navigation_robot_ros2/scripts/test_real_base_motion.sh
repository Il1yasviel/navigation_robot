#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
BASE_DEVICE="${BASE_DEVICE:-/dev/navigation_base}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
LOG_DIR="$(mktemp -d /tmp/navigation-motion-test.XXXXXX)"
DRIVER_LOG="${LOG_DIR}/base_driver.log"
DRIVER_PID=""
STARTED_DRIVER=false

if [[ " ${*} " != *" --wheels-raised "* ]]; then
  echo "拒绝运行：确认两轮架空后必须提供 --wheels-raised。" >&2
  exit 2
fi
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "找不到 ROS2 Humble：${ROS_SETUP}" >&2
  exit 2
fi
if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "找不到工作空间安装环境；请先在 ${WORKSPACE} 编译。" >&2
  exit 2
fi
if [[ ! -r "${BASE_DEVICE}" || ! -w "${BASE_DEVICE}" ]]; then
  echo "底盘设备不存在或当前用户不能读写：${BASE_DEVICE}" >&2
  exit 2
fi

set +u
source "${ROS_SETUP}"
source "${WORKSPACE}/install/setup.bash"
set -u
export ROS_DOMAIN_ID RMW_IMPLEMENTATION
export CYCLONEDDS_URI="file://${WORKSPACE}/install/navigation_robot_bringup/share/navigation_robot_bringup/config/cyclonedds.xml"

cleanup() {
  if [[ "${STARTED_DRIVER}" == true ]]; then
    timeout 4s ros2 service call /base_driver/stop std_srvs/srv/Trigger '{}' \
      >/dev/null 2>&1 || true
  fi
  if [[ -n "${DRIVER_PID}" ]] && kill -0 -- "-${DRIVER_PID}" 2>/dev/null; then
    kill -INT -- "-${DRIVER_PID}" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 -- "-${DRIVER_PID}" 2>/dev/null || break
      sleep 0.25
    done
    if kill -0 -- "-${DRIVER_PID}" 2>/dev/null; then
      kill -TERM -- "-${DRIVER_PID}" 2>/dev/null || true
    fi
  fi
  if [[ -n "${DRIVER_PID}" ]]; then
    wait "${DRIVER_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if ros2 node list 2>/dev/null | grep -qx '/base_driver'; then
  echo "检测到已有 /base_driver，将使用现有节点。"
else
  echo "临时启动底盘驱动（motion_enabled=true，测试结束自动关闭）"
  setsid ros2 launch navigation_robot_base_driver base_driver.launch.py \
    serial_port:="${BASE_DEVICE}" \
    wheel_radius_m:=0.05 \
    wheel_separation_m:=0.25 \
    motion_enabled:=true \
    >"${DRIVER_LOG}" 2>&1 &
  DRIVER_PID=$!
  STARTED_DRIVER=true

  ready=false
  for _ in $(seq 1 30); do
    if grep -q 'lower controller handshake complete' "${DRIVER_LOG}"; then
      ready=true
      break
    fi
    if ! kill -0 "${DRIVER_PID}" 2>/dev/null; then
      break
    fi
    sleep 0.25
  done
  if [[ "${ready}" != true ]]; then
    echo "底盘驱动握手失败。日志：${DRIVER_LOG}" >&2
    tail -n 60 "${DRIVER_LOG}" >&2
    exit 1
  fi
fi

"${WORKSPACE}/scripts/test_real_base_motion.py" "$@"
echo "测试结束；驱动日志：${DRIVER_LOG}"
