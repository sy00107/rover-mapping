#!/usr/bin/env bash
# ================================================================
# imu_fusion.sh
# 설명: 노트북에 저장된 stereo·외장 IMU를 별도 관성 SLAM 결과로 재생한다.
# 사용법: bash tools/imu_fusion.sh RUN [--segment 1] [--correlation-window-s START END]
# ================================================================
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
cd "$root"
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
exec "$root/.venv-mapping/bin/python" -m experiments.handheld_mapping.inertial "$@"
