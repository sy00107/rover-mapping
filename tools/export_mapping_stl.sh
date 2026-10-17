#!/usr/bin/env bash
# Export an independently refined STL without modifying the source mapping results.
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
cd "$root"
export PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
exec "$root/.venv-mapping/bin/python" -m experiments.handheld_mapping.stl_export "$@"
