#!/usr/bin/env bash
set -euo pipefail
field_code_root=$(cd "$(dirname "$0")/.." && pwd)
cd "$field_code_root"
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
unset QT_QPA_PLATFORM_PLUGIN_PATH
exec "$field_code_root/.venv-mapping/bin/python" -m experiments.handheld_mapping.field_collection "$@"
