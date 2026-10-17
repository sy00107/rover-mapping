#!/usr/bin/env bash
# Existing final ORB keyframes + original IR/depth -> RTAB database -> refined 3D map.
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
cd "$root"
if [[ $# -ne 2 ]]; then
    echo 'Usage: bash tools/build_orb_rtabmap.sh RUN NEW_OUTPUT' >&2
    exit 2
fi
run=$(realpath "$1")
output=$(realpath -m "$2")
[[ ! -e "$output" ]] || { echo 'Use a new output folder.' >&2; exit 2; }
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/rover-mapping-mpl
cmake -S experiments/handheld_mapping/rtabmap -B outputs/handheld_mapping/rtabmap_build -DCMAKE_PREFIX_PATH=/opt/ros/humble -DCMAKE_BUILD_TYPE=Release
cmake --build outputs/handheld_mapping/rtabmap_build -j2
python="$root/.venv-mapping/bin/python"
"$python" -m experiments.handheld_mapping.orb_rtabmap "$run" "$output"
"$python" -m experiments.handheld_mapping.rtabmap_rgbd_map "$run" "$output"
"$python" -m experiments.handheld_mapping.rtabmap_surface build "$output" "$output/surface_refined_v1"
"$python" -m experiments.handheld_mapping.rtabmap_ground_polish "$output/surface_refined_v1" "$output/surface_refined_v4"
echo "View: bash tools/view_rtabmap.sh '$output'"
