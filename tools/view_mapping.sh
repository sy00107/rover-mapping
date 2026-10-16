#!/usr/bin/env bash
# ================================================================
# view_mapping.sh
# 설명: 노트북 전용 환경에서 최신 또는 지정 촬영의 저장된 3D 지도를 연다.
# 사용법: bash tools/view_mapping.sh [RUN_OR_PLY] [--depth-max 1|2|3] [--surface-cleanup] [--build-only] [--cloud] [--no-trajectory] [--no-terrain-colors] [--no-obstacle-smoothing] [--no-ground-gap-fill]
# ================================================================
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
cd "$root"
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
export MPLCONFIGDIR="${MPLCONFIGDIR:-${TMPDIR:-/tmp}/rover-mapping-mpl-$UID}"
if [[ -n "${DISPLAY:-}" ]]; then
    unset WAYLAND_DISPLAY
    export XDG_SESSION_TYPE=x11
fi
exec "$root/.venv-mapping/bin/python" -m experiments.handheld_mapping.viewer "$@"
