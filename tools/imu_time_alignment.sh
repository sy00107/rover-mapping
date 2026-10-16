#!/usr/bin/env bash
# ================================================================
# imu_time_alignment.sh
# 설명: 종료된 D435·IMU 기록의 시계 드리프트와 회전 시차를 노트북에서 진단한다.
# 사용법: bash tools/imu_time_alignment.sh /absolute/path/to/RUN [--output NEW_DIRECTORY]
# ================================================================
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
cd "$root"
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
exec "$root/.venv-mapping/bin/python" -m experiments.handheld_mapping.time_alignment "$@"
