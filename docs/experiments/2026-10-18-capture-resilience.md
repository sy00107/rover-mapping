# Pi 원본 촬영 연결 내성·복구 보완

사용자가 승인한 리뷰 항목 1·2·3·4·6·7을 구현했다. 항목 5, Pi 재부팅 후 사라지는
RAM 배포 코드의 복구 경로는 보류했다. 기존 시작 명령은 유지한다.

## 바뀐 동작

### 1. IMU 브리지 독립 실행 및 영상 기록 분리

`tools/mapping_imu_bridge.sh`는 표준 라이브러리만 사용하는 `imu_bridge_service.py`를
Pi RAM 작업 경로에 배포한다. 독립 session·표준 입출력 분리·단일 소유 lock·PID 시작 시각
검증을 사용하는 supervisor가 기존 SPI 브리지를 실행한다. 반복 실행은 기존 서비스에 연결하며,
다른 SPI 소유자가 있으면 중복 실행하지 않는다. 기존 UDP 명령 0.3초 stale 처리와 MCU
watchdog 코드는 변경하지 않는다.

SSH는 로그만 표시한다. Ctrl+C/SSH 종료는 로그 연결만 닫으며, 브리지 종료는 명시적으로:

```bash
bash tools/mapping_imu_bridge.sh pi@raspberrypi.local --stop
```

종료 시 기존 브리지에 SIGINT를 전달해 KeyboardInterrupt/finally의 0 명령 종료를 사용한다.
이전 버전의 SSH 직결 브리지가 실행 중이면 그 터미널에서 먼저 정상 종료한 뒤 새 명령을 실행한다.
이번 작업에서 실제 Pi 서비스 시작·중지나 모터 명령을 수행하지 않았다.

IMU 시작 실패·일시 stale·subscriber 오류는 기본적으로 영상 기록을 중단하지 않는다.
`imu_health.jsonl`과 최종 quality에 상태 전환을 남기며, 후처리 결과는 부분 성공으로 표시한다.
명시적 `strict_imu` 설정은 기존 엄격 종료 정책을 선택할 수 있다.

### 2. IMU 오류 뒤에도 원본 회수 완료

Pi 로컬 기록 모드의 RemoteImu 오류는 노트북 수신 루프를 중단하지 않는다.
정상 원본 회수/검증 후 IMU 품질을 부분 성공으로 처리한다. 누락된 IMU 요약·보정 파일도
영상 회수를 먼저 막지 않으며, 남은 파일을 회수한 뒤 누락 사실을 보존한다.
유효하지 않은 IMU를 융합 성공으로 바꾸거나 빈 구간을 임의의 유효 표본으로 채우지 않는다.

### 3. 촬영 완전성과 전송 검증 분리

`CaptureIntegrity`가 저장 영상의 stream별 개수, 최대 시각 공백, 순번 공백,
좌우 exact timestamp 매칭률, left/depth 2ms 대응률을 계산한다. 최대 128개 pending 시각만
유지하므로 한 stream이 장시간 멈춰도 해당 비교 버퍼는 커지지 않는다.

15 FPS의 1.5배 주기(100ms)보다 긴 시각 공백, 미대응 좌우 또는 깊이 누락을 partial로 표시한다.
시각 공백은 불연속의 증거이며 검증된 카메라 hardware frame counter와 같다고 주장하지 않는다.
카메라 worker는 left/right 각각의 수신 중단을 감시한다. depth만 중단되면 경고를 내고
좌우 원본은 계속 기록하며 최종 품질에 누락을 남긴다.

### 4. recorder 비정상 종료 원본 회수

`recording_recovery.py`는 소유 PID 시작 시각과 recording lock을 확인한다. 죽은 recorder의
잔여 자식만 정리한 뒤, capture의 패킷 구조·영상 압축을 검사해 완전한 prefix를 찾는다.
Pi 원본을 truncate하거나 덮어쓰지 않는다. 복구 manifest는 prefix size/SHA256을 기록하고
fetch는 그 byte 수까지만 전송한다. 미완성 IMU JSONL 마지막 행도 사본에서만 제외한다.

마감 finished 패킷이 없으면 수신 측에서 '복구됨/부분 기록'을 명시한 종료 이벤트를 만든다.
충분한 원본이 있으면 후처리를 진행하지만 전체 촬영 정상 완료로 표시하지 않는다.
보정값이 없는 촬영은 복구 실패 사유를 유지한다.

### 6. 앱/전송/파일 정리 중단 후 재개

촬영 폴더의 `transfer_checkpoint.json`에 host·원격 run_id·단계·manifest 해시를 저장한다.
다른 host/ID로 같은 폴더를 재사용하지 않는다. .part는 기존 byte offset에서 이어받고,
검증 완료 파일은 재사용한다. PNG/frames.jsonl 생성 중 중단돼도 기존 인덱스를 읽고
기존 영상 내용과 보정값을 확인해 중복 없이 계속한다. 없거나 디코딩되지 않는 PNG는
검증된 원본으로 복구하며, 정상적으로 읽히지만 다른 영상으로 바뀐 파일은 거부한다.

앱: '중단된 원본 이어받기'에서 기존 촬영 폴더 선택. 전송·검증 뒤 최종 연산을 진행한다.
CLI: 아래 명령은 최종 지도 계산 없이 영상·IMU를 회수한다.

```bash
PYTHONNOUSERSITE=1 .venv-mapping/bin/python -m experiments.handheld_mapping.local_client \
  --recover --host pi@raspberrypi.local --run-id RUN_ID \
  --run outputs/handheld_mapping/sessions/RUN_ID
```

회수 경로는 기존 촬영에 attach하며 새 카메라 촬영을 시작하지 않는다. 완료 manifest와
전체 파일이 이미 노트북에 검증돼 있으면 Pi 재연결 없이 파일 정리를 재개할 수 있다.
이것이 Pi 재부팅 후 아직 전송하지 못한 파일을 자동 회수하는 기능을 추가했다는 뜻은 아니다.

### 7. 종료 요청·실제 종료·전송 구분 및 보류

종료 요청 중 → Pi 요청 수신 → Pi 촬영 종료 확인/마감 중 → 원본 전송으로 상태를 나눈다.
로컬 stop 클릭만으로 '촬영 종료·전송 중'으로 바꾸지 않는다. 원격 종료는
capture_stopped 또는 마감 manifest로 확인한다.

앱의 '전송 보류 · 나중에 이어받기'는 원본과 checkpoint를 보존한다. 전송/종료 대기 중
창 닫기도 보류 경로를 사용한다. 장기 응답/종료 대기는 기본 75초 후 보류하며, 미확인
원격 촬영을 종료 성공으로 표시하지 않는다. 종료 확인 전 보류라면 Pi가 계속 촬영 중일 수 있음을 표시한다.

## 검증

- 전체 handheld 회귀: **522 passed, 9 skipped (67.78초)**.
- 새 회귀는 실제 로컬 subprocess로 IMU 일시 정지/시작 실패, 오류 후 영상 회수,
  recorder 강제 종료와 손상 tail, IMU JSONL tail, 다운로드 및 PNG import 재개,
  잘못된 run_id 거부, remote stop 미확인 timeout, 단일 브리지/로그 종료/정상 정리,
  앱 resume의 binary 통신과 창 닫기, 남은 지도가 성공해도 partial 판정 등을 검증한다.
- 소스 및 회수 영상 파일의 불변성, 원래 command watchdog 보존을 확인했다.
- 변경 Python pyflakes, compileall, bash -n, git diff --check 및 앱 화면 확인을 수행했다.
- 실제 Pi 카메라·CPU·SD·Wi-Fi 동시 부하/전원 차단 시험은 아직 하지 않았다.
  따라서 프레임 무누락이나 실제 현장 성능을 보장하지 않는다.
