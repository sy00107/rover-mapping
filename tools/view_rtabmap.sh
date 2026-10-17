#!/usr/bin/env bash
# RTAB-Map 결과를 기존 Open3D 지도 창에서 연다.
# Usage: bash tools/view_rtabmap.sh [COMPARISON_FOLDER] [--raw] [--cloud]
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
folder="$root/outputs/handheld_mapping/comparisons/rtab_rgbd_20261016_174639_full"
if [[ $# -gt 0 && "$1" != --* ]]; then
    folder=$(realpath "$1")
    shift
fi
# Refined results retain their stored classification colours in a native Z-up viewer.
refined="$folder/surface_refined_v1"
if [[ -f "$folder/surface_refined_v2/summary.json" ]]; then
    refined="$folder/surface_refined_v2"
fi
if [[ -f "$folder/surface_refined_v4/summary.json" ]]; then
    refined="$folder/surface_refined_v4"
fi
if [[ -f "$folder/rtab_surface_mesh.ply" ]]; then
    refined="$folder"
fi
if [[ "${1:-}" == --raw ]]; then
    shift
elif [[ -f "$refined/summary.json" ]]; then
    cd "$root"
    export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
    export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/rover-mapping-mpl}"
    if [[ -n "${DISPLAY:-}" ]]; then
        unset WAYLAND_DISPLAY
        export XDG_SESSION_TYPE=x11
    fi
    exec "$root/.venv-mapping/bin/python" -m experiments.handheld_mapping.rtabmap_surface view "$refined" "$@"
fi
mesh_name=rtab_common_depth_mesh.ply
cloud_name=rtab_common_depth.ply
if [[ -f "$folder/rtab_map_summary.json" ]]; then
    mesh_name=rtab_map_mesh.ply
    cloud_name=rtab_map_cloud.ply
fi
geometry="$folder/$mesh_name"
if [[ "${1:-}" == --cloud ]]; then
    geometry="$folder/$cloud_name"
    shift
    set -- --cloud "$@"
fi
if [[ ! -f "$geometry" ]]; then
    echo "RTAB-Map 3D 결과가 없습니다: $geometry" >&2
    exit 1
fi
exec bash "$root/tools/view_mapping.sh" "$geometry" --no-trajectory --no-terrain-colors "$@"
