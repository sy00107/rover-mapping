#!/usr/bin/env bash
# Show ORB camera pose + RTAB RGB-D map with its camera trajectory and endpoints.
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
folder="$root/outputs/handheld_mapping/comparisons/orb_rtab_fusion_20261016_174639"
if [[ "${1:-}" == --raw ]]; then
    exec bash "$root/tools/view_rtabmap.sh" "$folder" "$@"
fi
exec bash "$root/tools/view_rtabmap.sh" "$folder" "$@" --trajectory
