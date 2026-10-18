#!/usr/bin/env bash
# Read-only preflight: no SPI access and no motor command.
set -euo pipefail
imu_code_root=$(cd "$(dirname "$0")/.." && pwd)
imu_target=${1:-pi@raspberrypi.local}
if [[ ! "$imu_target" =~ ^[A-Za-z0-9_.@:-]+$ || "$imu_target" == -* ]]; then
  echo '잘못된 SSH 대상' >&2
  exit 2
fi
ssh -T -o BatchMode=yes -o ConnectTimeout=8 "$imu_target" \
  /usr/bin/python3 - < "$imu_code_root/experiments/handheld_mapping/imu_health.py"
