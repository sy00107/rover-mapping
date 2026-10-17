#!/usr/bin/env bash
# Native preview of simplified rectangular terrain; O toggles source-point overlay.
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
cd "$root"
folder="$root/outputs/handheld_mapping/exports/orb_rtab_block_terrain_v3"
if [[ $# -gt 0 && "$1" != --* ]]; then
    folder=$(realpath "$1")
    shift
fi
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/rover-mapping-mpl}"
if [[ -n "${DISPLAY:-}" ]]; then
    unset WAYLAND_DISPLAY
    export XDG_SESSION_TYPE=x11
fi
exec "$root/.venv-mapping/bin/python" -m experiments.handheld_mapping.block_terrain view "$folder" "$@"
