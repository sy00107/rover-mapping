#!/usr/bin/env bash
# ================================================================
# mapping_imu_bridge.sh
# 설명: Pi의 단일 독립 supervisor로 IMU·조이스틱 브리지를 실행하고 SSH는 로그만 표시한다.
# 사용법: bash tools/mapping_imu_bridge.sh [pi@raspberrypi.local]
# ================================================================
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
target_host=${1:-pi@raspberrypi.local}
if [[ ! "$target_host" =~ ^[A-Za-z0-9_.@:-]+$ || "$target_host" == -* ]]; then
  echo '잘못된 SSH 대상' >&2
  exit 2
fi
ssh_options=(-o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3)
# Pi의 단일 supervisor가 SPI 브리지를 소유한다. SSH는 로그만 따라간다.
# Ctrl+C는 로그 연결만 닫는다. 명시적 종료: bash tools/mapping_imu_bridge.sh HOST --stop
remote_root=/dev/shm/rover_mapping_imu_bridge_service
if [[ "${2:-}" == "--stop" ]]; then
  ssh -T "${ssh_options[@]}" "$target_host" "/usr/bin/python3 '$remote_root/imu_bridge_service.py' stop"
  exit
fi
if [[ -n "${2:-}" ]]; then
  echo '지원하지 않는 옵션: --stop만 사용할 수 있습니다.' >&2
  exit 2
fi
ssh -T "${ssh_options[@]}" "$target_host" "mkdir -p '$remote_root'"
scp "${ssh_options[@]}" "$root/experiments/handheld_mapping/imu_bridge_service.py" "$target_host:$remote_root/imu_bridge_service.py"
scp "${ssh_options[@]}" "$root/ros2_ws/src/orbslam3_ros2/patches/ksrc_telemetry_tee.patch" "$target_host:$remote_root/tee.patch"
ssh -T "${ssh_options[@]}" "$target_host" "/usr/bin/python3 '$remote_root/imu_bridge_service.py' start"
echo 'IMU 브리지는 Pi에서 독립 실행 중입니다. Ctrl+C는 로그만 닫습니다.'
echo "브리지 종료: bash tools/mapping_imu_bridge.sh $target_host --stop"
ssh -T "${ssh_options[@]}" "$target_host" "/usr/bin/python3 -u '$remote_root/imu_bridge_service.py' follow"
