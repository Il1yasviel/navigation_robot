#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_DEVICE="${BASE_DEVICE:-/dev/navigation_base}"
CHECK_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
CHECK_RMW="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
LOG_DIR="$(mktemp -d /tmp/navigation-base-check.XXXXXX)"
LAUNCH_LOG="${LOG_DIR}/base_driver.log"
LAUNCH_PID=""

cleanup() {
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 -- "-${LAUNCH_PID}" 2>/dev/null; then
    kill -INT -- "-${LAUNCH_PID}" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 -- "-${LAUNCH_PID}" 2>/dev/null || break
      sleep 0.25
    done
    if kill -0 -- "-${LAUNCH_PID}" 2>/dev/null; then
      kill -TERM -- "-${LAUNCH_PID}" 2>/dev/null || true
    fi
  fi
  wait "${LAUNCH_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [[ ! -e "${BASE_DEVICE}" ]]; then
  echo "找不到 ${BASE_DEVICE}。请先运行 ./scripts/install_udev_rules.sh 并重新插拔底盘 USB。" >&2
  exit 2
fi
if [[ ! -r "${BASE_DEVICE}" || ! -w "${BASE_DEVICE}" ]]; then
  echo "当前用户不能读写 ${BASE_DEVICE}；确认已加入 dialout 组并重新登录。" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${CHECK_DOMAIN_ID}"
export RMW_IMPLEMENTATION="${CHECK_RMW}"
export CYCLONEDDS_URI="file://${WORKSPACE}/install/navigation_robot_bringup/share/navigation_robot_bringup/config/cyclonedds.xml"

ros2 daemon stop >/dev/null 2>&1 || true
if ros2 node list 2>/dev/null | grep -qx '/base_driver'; then
  echo "/base_driver 已在运行；为避免两个进程争抢串口，检测已停止。" >&2
  exit 2
fi

echo "底盘设备: ${BASE_DEVICE}"
echo "安全模式: motion_enabled=false，轮径/轮距=0，不发送运动RPM"
echo "日志目录: ${LOG_DIR}"

setsid ros2 launch navigation_robot_base_driver base_driver.launch.py \
  serial_port:="${BASE_DEVICE}" motion_enabled:=false \
  >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!

ready=false
for _ in $(seq 1 20); do
  if grep -q 'lower controller handshake complete' "${LAUNCH_LOG}"; then
    ready=true
    break
  fi
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    break
  fi
  sleep 0.5
done

if [[ "${ready}" != true ]]; then
  echo "底盘握手未完成。驱动日志：" >&2
  tail -n 40 "${LAUNCH_LOG}" >&2
  exit 1
fi

echo
echo "[1/4] 握手成功"
grep 'connected through\|handshake complete\|safety lock' "${LAUNCH_LOG}" | tail -n 10

echo
echo "[2/4] 生命周期"
ros2 lifecycle get /base_driver

failed=0
echo
echo "[3/4] 底盘诊断"
if ! timeout 8s ros2 topic echo /diagnostics --once; then
  echo "未收到 /diagnostics" >&2
  failed=1
fi

echo
echo "[4/4] 左右轮与 IMU"
if ! timeout 8s ros2 topic echo /joint_states --once; then
  echo "未收到 /joint_states" >&2
  failed=1
fi
if ! timeout 8s ros2 topic echo /imu/data_raw --once; then
  echo "未收到 /imu/data_raw" >&2
  failed=1
fi

echo
ros2 service call /base_driver/stop std_srvs/srv/Trigger '{}'
echo "检测完成；已发送停车命令。完整日志：${LAUNCH_LOG}"
exit "${failed}"
