#!/usr/bin/env bash
# Detail-preserving terrain with short local gaps joined; O toggles measured source.
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
cd "$root"
folder="$root/outputs/handheld_mapping/exports/orb_rtab_joined_terrain_v2"
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
exec "$root/.venv-mapping/bin/python" -m experiments.handheld_mapping.joined_terrain view "$folder" "$@"
