#!/usr/bin/env bash
# Fuse image-validated session registrations into a new, separate 2m map.
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
cd "$root"
export PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/rover-mapping-merge-mpl-$UID}"
exec "$root/.venv-mapping/bin/python" -m experiments.handheld_mapping.session_merge "$@"
