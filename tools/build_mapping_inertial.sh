#!/usr/bin/env bash
# ================================================================
# build_mapping_inertial.sh
# 설명: 기존 native core와 별도 빌드 폴더에서 IMU 비교 실행기·입력 검사를 빌드한다.
# 사용법: bash tools/build_mapping_inertial.sh /absolute/path/to/ORB_SLAM3
# ================================================================
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
orb_root=${1:?기존 native ORB_SLAM3 source·library 경로가 필요합니다}
prefix="$root/outputs/handheld_mapping/native_dependencies/pangolin"
build="$root/outputs/handheld_mapping/orb3_imu_build"
runtime="$root/outputs/handheld_mapping/orb3_comparison_build/runtime"
test -r "$runtime/libORB_SLAM3.so"
if ! cmp -s "$orb_root/lib/libORB_SLAM3.so" "$runtime/libORB_SLAM3.so"; then
  echo '빌드 source library와 기존 비교 runtime이 다릅니다. 같은 native core를 먼저 준비하세요.' >&2
  exit 1
fi
cmake -S "$root/experiments/handheld_mapping/orbline/baseline" -B "$build" \
  -DORB3_ROOT="$orb_root" -DROVER_BUILD_IMU_COMPARE=ON \
  -DCMAKE_PREFIX_PATH="$prefix" -DPangolin_DIR="$prefix/lib/cmake/Pangolin"
cmake --build "$build" --target rover_orb3_imu_compare rover_imu_input_probe --parallel 1
ctest --test-dir "$build" --output-on-failure
