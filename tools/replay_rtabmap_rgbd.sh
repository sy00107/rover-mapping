#!/usr/bin/env bash
# Recorded left IR + measured depth -> RTAB-Map RGB-D -> native 3D mesh.
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
cd "$root"
if [[ $# -lt 2 || $# -gt 3 ]]; then
    echo 'Usage: bash tools/replay_rtabmap_rgbd.sh RUN NEW_OUTPUT [MAX_FRAMES]' >&2
    exit 2
fi
run=$(realpath "$1")
output=$(realpath -m "$2")
[[ ! -e "$output" ]] || { echo 'Use a new output folder.' >&2; exit 2; }
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
export MPLCONFIGDIR=/tmp/rover-mapping-mpl
cmake -S experiments/handheld_mapping/rtabmap -B outputs/handheld_mapping/rtabmap_build -DCMAKE_PREFIX_PATH=/opt/ros/humble -DCMAKE_BUILD_TYPE=Release
cmake --build outputs/handheld_mapping/rtabmap_build -j2
"$root/.venv-mapping/bin/python" -m experiments.handheld_mapping.rtabmap_rgbd "$run" "$output" --max-frames "${3:-0}"
"$root/.venv-mapping/bin/python" -m experiments.handheld_mapping.rtabmap_rgbd_map "$run" "$output"
echo "Open 3D map: bash tools/view_rtabmap.sh '$output'"
