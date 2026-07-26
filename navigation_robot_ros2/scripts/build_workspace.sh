#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
cd "${WORKSPACE}"

# ARM64 上可通过 COLCON_WORKERS=2 限制峰值内存；开发机默认使用 4。
WORKERS="${COLCON_WORKERS:-4}"
colcon build \
  --symlink-install \
  --parallel-workers "${WORKERS}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
