#!/usr/bin/env bash
# Replay preserved stereo IR through RTAB-Map and compare with existing ORB offline output.
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
cd "$root"
if [[ $# -ne 2 ]]; then
  echo 'Usage: bash tools/compare_orb_rtabmap.sh RUN NEW_OUTPUT' >&2
  exit 2
fi
run=$(realpath "$1")
output=$(realpath -m "$2")
if [[ -e "$output" ]]; then
  echo 'Use a new output folder; existing results are preserved.' >&2
  exit 2
fi
export LD_LIBRARY_PATH="/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}"
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 PYTHONNOUSERSITE=1
export MPLCONFIGDIR=/tmp/rover-mapping-mpl
python="$root/.venv-mapping/bin/python"
cmake -S experiments/handheld_mapping/rtabmap -B outputs/handheld_mapping/rtabmap_build -DCMAKE_PREFIX_PATH=/opt/ros/humble -DCMAKE_BUILD_TYPE=Release
cmake --build outputs/handheld_mapping/rtabmap_build -j2
"$python" -m experiments.handheld_mapping.compare_rtabmap "$run" --output "$output"
mapfile -t calibration < <("$python" - "$run" <<'PY'
import json,sys
from pathlib import Path
c=json.loads((Path(sys.argv[1])/'capture_calibration.json').read_text())['stereo']
if c.get('input_geometry') != 'rectified_confirmed':
    raise ValueError('Rectified stereo required')
k=c['left']['k']
for v in [k[0],k[4],k[2],k[5],c['T_c1_c2'][0][3]]: print(v)
PY
)
[[ ${#calibration[@]} -eq 5 ]]
outputs/handheld_mapping/rtabmap_build/rover_rtab_compare "$output/stereo_pairs.tsv" "$output" "${calibration[@]}" 0 > "$output/rtab_runner.log" 2>&1
"$python" -m experiments.handheld_mapping.compare_rtabmap "$run" --output "$output" --rtab-tum "$output/rtab_optimized.tum"
"$python" -m experiments.handheld_mapping.compare_rtabmap_maps "$run" "$output"
echo "Comparison: $output/maps_comparison.html"
