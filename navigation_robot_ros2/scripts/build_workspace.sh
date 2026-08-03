#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# ROS 2's generated setup scripts probe optional variables that may be unset.
# Temporarily disable nounset while sourcing them, then restore strict mode.
set +u
source /opt/ros/humble/setup.bash
set -u
cd "${WORKSPACE}"

# ARM64 上可通过 COLCON_WORKERS=2 限制峰值内存；开发机默认使用 4。
WORKERS="${COLCON_WORKERS:-4}"
colcon build \
  --symlink-install \
  --parallel-workers "${WORKERS}" \
  --packages-skip nav2_system_tests \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
