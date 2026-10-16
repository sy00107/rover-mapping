#!/usr/bin/env bash
# ================================================================
# mapping_imu_bridge.sh
# 설명: Pi 원본 KSRC를 보존하고 RAM 사본에 비차단 IMU 분기를 적용해 조이스틱 중계를 실행한다.
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
remote_dir=/dev/shm/rover_mapping_imu_bridge
ssh "${ssh_options[@]}" "$target_host" 'set -e
command -v fuser >/dev/null
command -v patch >/dev/null
if fuser -s /dev/spidev0.0; then
  echo "SPI가 사용 중입니다. 기존 joystick bridge/IMU 프로그램을 정상 종료한 후 다시 실행하세요." >&2
  exit 23
fi
test -r /home/pi/ksrc/pi/common/nucleo_link.py
mkdir -p /dev/shm/rover_mapping_imu_bridge/pi/common
cp /home/pi/ksrc/pi/common/udp_spi_bridge.py /home/pi/ksrc/pi/common/nucleo_link.py /dev/shm/rover_mapping_imu_bridge/pi/common/'
scp "${ssh_options[@]}" "$root/ros2_ws/src/orbslam3_ros2/patches/ksrc_telemetry_tee.patch" "$target_host:$remote_dir/tee.patch"
ssh -tt "${ssh_options[@]}" "$target_host" 'set -e
if fuser -s /dev/spidev0.0; then
  echo "SPI가 다른 프로그램에서 열렸습니다. 중계를 시작하지 않습니다." >&2
  exit 23
fi
cd /dev/shm/rover_mapping_imu_bridge
patch --batch --fuzz=0 --forward -p1 -i tee.patch
exec env PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -u pi/common/udp_spi_bridge.py \
  --hz 50 --max-lin 0.45 --max-ang 0.8 --telemetry-socket /dev/shm/rover-mapping-imu.sock'
