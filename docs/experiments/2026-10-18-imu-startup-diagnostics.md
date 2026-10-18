# IMU 시작 타임아웃 진단

2026-10-18 확인:

- `sessions/20261018_122106_df4fe0`: subscriber ready manifest는 있으나 IMU raw 0건,
  valid/unique 0, 이미지 0건. 카메라 시작 전 IMU 준비 경계에서 실패했다.
- 최초 Pi 점검에서는 udp_spi_bridge 및 SPI owner가 없었다. 이어 SSH 연결에 timeout/
  no-route가 관측됐다. 이후 연결이 복구되고 사용자의 브리지 PID 1417이 확인됐다.
- 실행 중인 브리지를 중단하지 않고 별도 진단 수신기로 1.5초 데이터를 기록:
  `diagnostics/imu_check_20261018_122645`, recorded=76, valid=76, unique=60,
  stale=false. 진단 수신기는 정상 종료했고 기존 브리지는 유지했다. 카메라는 시작하지 않았다.

변경:

- 일반 IMU 시작 전 Pi `/proc`에서 정확한 `--telemetry-socket` 송신 인자를 확인한다.
  송신 브리지가 없으면 촬영 전에 터미널 1 실행 안내로 실패한다. SPI를 열거나 모터를
  제어하는 preflight가 아니며, 구독기의 `--socket`이나 셸 문자열을 송신자로 오인하지 않는다.
- timeout을 SSH/receiver 준비 전, receiver ready지만 0건, IMU_VALID 미수신,
  새로운 MCU 표본 미수신으로 구분한다. 시간을 무조건 늘리거나 무효 데이터를 수락하지 않는다.
- SSH/원격 preflight stderr를 시작 실패 메시지에 포함한다.
- 읽기 전용 점검: `bash tools/check_mapping_imu.sh pi@raspberrypi.local`.

사용 절차: 터미널 1의 `mapping_imu_bridge.sh`를 계속 유지하고, 수정 후 노트북 매핑 앱을
완전히 종료했다가 다시 연다. 이미 bridge가 실행 중이면 중복 실행하지 않는다.
다시 네트워크 timeout/no route가 나면 Wi-Fi/Pi 연결부터 복구해야 한다.
