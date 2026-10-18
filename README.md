# Rover Mapping · v3

Intel RealSense D435와 Raspberry Pi로 영상·IMU를 기록하고, 촬영이 끝나면 노트북에서 ORB-SLAM3와 TSDF로 3D 지도를 만드는 프로젝트다.

**v3의 기본 흐름은 Pi 원본 저장 → PC 전송·검증 → 검증된 과거 Pi 원본 정리 → PC에서 최종 지도 생성이다.** 촬영 중 화면은 미리보기이며 최종 지도는 전체 원본으로 다시 계산한다. 조종기는 별도 창에서 실행한다.

이 README는 **2026-10-18 기준 `v3` 브랜치**의 실행 방법을 설명한다. 날짜가 붙은 실험 문서는 당시 결과를 담으므로 현재 기본 동작은 아래 절차를 따른다.

## v3에서 달라진 점

| 기능 | 현재 동작 |
|---|---|
| 영상·IMU 기록 | 실제 촬영에서는 항상 함께 수집 요청. 촬영 관련 체크박스 제거 |
| Pi 원본 저장 | 좌·우 IR과 깊이 원본을 Pi에 기록. SSH 미리보기 연결과 분리 |
| 연결·IMU 이상 | 미리보기 연결이나 IMU 수신이 끊겨도 영상 기록은 계속하고 누락 이력을 남김 |
| PC 후처리 | 종료 후 원본을 가져와 전체 영상으로 ORB·TSDF·중력 기준 지형 처리 |
| 전송 표시 | 별도 터미널 자동 실행. 퍼센트·용량·속도·예상 시간을 한 줄에서 갱신 |
| 전송 복구 | 부분 다운로드와 영상 import를 기존 촬영 폴더에서 이어받기 |
| Pi 저장 공간 | PC 원본을 검증한 과거 촬영만 정리. 최신 촬영과 미검증·이상 촬영은 보관 |
| 조종 | 방향키·Space·Q/E와 화면 버튼. 대각선은 **45° 평행 이동**, 좌우 단독은 제자리 회전 |
| 지도 확인 | 3D 보기, 거리별 재생성, 높이·경사 보기, 평면 보정 STL 내보내기 |

## 준비

- 노트북: Ubuntu 22.04 / Python 3.10 / `.venv-mapping` 기준.
- Pi: Ubuntu 24.04 / native ROS 2 Jazzy / RealSense와 기존 KSRC·STM 제어 환경 기준.
- 노트북에서 Pi에 SSH 접속할 수 있어야 한다. 조종은 UDP 5005를 사용한다.
- 실제 촬영에는 native ORB 실행기·라이브러리·vocabulary와 Pi runtime이 필요하다.
- 실제 IMU 보정 파일은 로컬 `outputs/handheld_mapping/calibration/imu_calibration.json`에 준비하거나 앱의 **세부 설정**에서 지정한다.

처음 받는 경우 접근 권한이 있는 계정으로 clone하고 [설치 설명서](docs/installation.md)를 따른다.

```bash
git clone --branch v3 https://github.com/sy00107/rover-mapping.git
cd rover-mapping
```

Python 환경·native 빌드·Pi 설치를 이미 마쳤다면 매번 다시 설치할 필요는 없다. 촬영 원본, 로컬 보정 JSON, 실행 바이너리와 vocabulary는 Git에 포함되지 않는다. [별도 자산 안내](docs/data-assets.md)를 참고한다.

## 실제 실행: 노트북 터미널 3개

각 터미널에서 **clone한 저장소 루트**로 이동한 뒤 실행한다. 기본 Pi 주소는 `pi@raspberrypi.local`이다. IP를 사용할 경우 브리지·조종기·앱의 Pi 주소를 함께 맞춘다.

### 1. 터미널 1 — IMU·조종 중계

```bash
bash tools/mapping_imu_bridge.sh pi@raspberrypi.local
```

Pi에서 단일 브리지를 독립 실행하고 이 터미널은 로그를 표시한다. 같은 브리지가 이미 실행 중이면 다시 연결한다. 별도의 `udp_spi_bridge.py`나 SPI monitor를 중복 실행하지 않는다.

IMU 상태만 확인하려면 다른 터미널에서 아래 명령을 사용할 수 있다.

```bash
bash tools/check_mapping_imu.sh pi@raspberrypi.local
```

### 2. 터미널 2 — 매핑 앱

```bash
bash tools/handheld_mapping.sh
```

창을 열기만 해서는 촬영이 시작되지 않는다. 앱의 **Pi 연결**과 **세부 설정 → IMU 보정 파일·저장 폴더**를 확인한다.

1. **매핑 시작**을 누른다.
2. 평지에서 처음 **3초 이상 정지**하고 영상·IMU 상태를 확인한다.
3. 조종기로 천천히 이동하며 같은 표면을 여러 시점에서 촬영한다.
4. 촬영을 마치면 **먼저 조종기로 정지**한다.
5. **종료 및 지도 생성**을 누른다. 매핑 종료 버튼은 모터 정지 버튼이 아니다.
6. 자동으로 열리는 전송 터미널에서 수신·검증·Pi 정리 결과를 확인한다.
7. 앱의 최종 지도 생성과 완료 상태를 확인한 뒤 Pi 전원을 종료한다.

IMU 수신 문제가 생겨도 영상은 보존한다. 앱의 경고와 `partial` 상태를 확인해야 하며, IMU 기록을 요청했다는 사실이 유효한 IMU 표본을 모두 받았다는 뜻은 아니다.

### 3. 터미널 3 — 실제 조종기

```bash
PYTHONNOUSERSITE=1 .venv-mapping/bin/python \
  teleop/ksrc_snapshot/pi/teleop/teleop_joystick.py \
  --udp raspberrypi.local --rate 50 \
  --max-lin 0.15 --max-ang 0.8 --monitor
```

조종 창에 포커스를 둔 상태에서 입력한다.

| 입력 | 동작 |
|---|---|
| ↑ 또는 Space | 전진 |
| ↓ | 후진 |
| ← 또는 Q | 제자리 좌회전 |
| → 또는 E | 제자리 우회전 |
| ↑/Space + ←/Q | 앞·왼쪽 45° 평행 이동 |
| ↑/Space + →/E | 앞·오른쪽 45° 평행 이동 |
| ↓ + ←/Q | 뒤·왼쪽 45° 평행 이동 |
| ↓ + →/E | 뒤·오른쪽 45° 평행 이동 |
| 1 / 2 / 3 | 전후·대각선 속도 1 / 2 / 3배 |
| Esc | 정지 명령 후 조종 창 종료 |

대각선 명령은 `|vx| = |vy|`, `omega = 0`이다. 전체 이동 속도를 정규화하므로 대각선에서도 선택 속도를 유지한다. 위 명령의 속도는 **0.15 / 0.30 / 0.45m/s**이고, 단독 회전은 `--max-ang`의 50%인 **0.4rad/s**다. 실제 명령은 Pi 브리지의 상한으로도 제한된다.

화면 오른쪽 8방향 버튼도 같은 동작을 한다. 기존 왼쪽 마우스 패드는 전후·좌우 이동, 오른쪽 패드는 회전이다. 키는 누르는 동안 적용되고 모두 떼면 정지한다. 포커스를 잃으면 입력을 해제하며, 돌아온 뒤 새 입력이 필요하다.

### 사용 후 브리지 종료

앱의 촬영·전송을 마치고 조종기도 정지한 뒤, 저장소 루트의 터미널에서 실행한다.

```bash
bash tools/mapping_imu_bridge.sh pi@raspberrypi.local --stop
```

새 터미널에서 실행해도 되고, 터미널 1에서 Ctrl+C로 로그를 빠져나온 뒤 실행해도 된다. **Ctrl+C는 로그 연결만 닫는다. 브리지 종료는 `--stop` 명령으로 한다.**

## 전송 창과 Pi 원본 정리

전송이 시작되면 새 터미널이 자동으로 열린다. **전송 창 열기** 버튼으로 다시 열 수 있다.

- 전송률·용량·속도·남은 예상 시간을 한 줄에서 갱신한다.
- 연결 불안정·복구 때만 알림 줄을 추가한다.
- **100%는 바이트 수신률**이다. 이후 영상 정리·전체 검증·Pi 정리 결과를 확인한다.
- 터미널을 닫아도 실제 전송은 계속된다. 전송 보류는 앱에서 한다.
- 완료·보류·실패 후에는 Enter를 누를 때까지 창이 남는다.

정상 촬영과 전송만 있을 때 Pi에는 최신 촬영 한 개가 남는다. 과거 촬영을 지우기 전에는 **PC 원본 전체의 크기·SHA-256·영상 디코딩·프레임 수·JSON/JSONL을 확인하고 디스크에 저장을 확정(`fsync`)**한다. 과거 PC 복사본도 매번 다시 확인하며 Pi 원본의 크기·체크섬도 재검사한다.

미전송·손상·비정상 종료·IMU 이상 촬영, 또는 PC 복사본을 확인할 수 없는 촬영은 보존한다. 최신 촬영이 미전송 상태면 마지막 검증 촬영도 남기므로 **예외가 있으면 한 개보다 많이 남는다.** PC의 원본은 자동 삭제하지 않는다.

정리 결과와 보류 사유는 앱 화면 및 촬영 폴더의 `pi_retention.json`에 남는다. [삭제 승인 조건·예외·검증 기록](docs/experiments/2026-10-18-recording-retention.md).

## 중단된 원본 이어받기

앱의 **전송 보류 · 나중에 이어받기**로 중단하고, 나중에 **중단된 원본 이어받기**에서 같은 촬영 폴더를 선택한다. 부분 다운로드와 이미 정리한 영상을 확인해 중복 없이 재개한다.

앱 없이 원본 회수만 하려면 아래에서 `RUN_ID`를 실제 촬영 ID로 바꾼다.

```bash
PYTHONNOUSERSITE=1 .venv-mapping/bin/python \
  -m experiments.handheld_mapping.local_client \
  --recover --host pi@raspberrypi.local --run-id RUN_ID \
  --run outputs/handheld_mapping/sessions/RUN_ID
```

이 명령은 원본 회수·검증·정리 경로이며 최종 지도 계산은 별도다. 완료된 PC 원본은 Pi 연결 없이도 import할 수 있지만, **Pi 재부팅 후 아직 전송하지 못한 원본을 회수하는 경로에는 RAM 배포 코드 유실 문제가 남아 있다.** 촬영 중단·전송 보류 뒤에는 Pi 전원을 유지한다. [연결 내성·복구 동작](docs/experiments/2026-10-18-capture-resilience.md).

## 지도 보기와 저장 위치

앱의 **3D 지도 보기**, **높이·경사 보기**, **기록에서 다시 생성**, **평면 보정 STL 내보내기**를 사용한다. 거리·표면 보정·지도 종류는 **세부 설정**에 있다. 3D 보기 기본은 카메라 지도 우선, 2m, 표면 보정이다.

```bash
# 최신 촬영 지도
bash tools/view_mapping.sh

# 특정 촬영을 2m·표면 보정으로 보기
bash tools/view_mapping.sh outputs/handheld_mapping/sessions/RUN_ID \
  --depth-max 2 --surface-cleanup
```

| 위치 | 내용 |
|---|---|
| Pi `/home/pi/handheld_mapping/recordings/RUN_ID` | 촬영 중 원본·IMU·마감 manifest |
| PC `outputs/handheld_mapping/sessions/RUN_ID` | 촬영별 원본·지도·상태·로그 |
| `RUN/pi_originals/` | 전송받은 `capture.bin`·IMU 등 원본 파일과 manifest |
| `RUN/raw/`, `frames.jsonl` | 원본 아카이브에서 정리한 영상과 인덱스 |
| `RUN/preview/` | 촬영 중 받은 미리보기 |
| `RUN/transfer_checkpoint.json` | 촬영 ID·전송·이어받기 상태 |
| `RUN/transfer_status.json`, `transfer_progress.jsonl` | 현재 전송 표시와 진행 이력 |
| `RUN/pi_retention.json`, `session.json` | Pi 정리 결과와 세션 상태 |

`pi_originals`는 PNG나 지도를 다시 만들 수 있는 원본이므로 필요한 촬영은 **RUN 전체**를 백업한다.

## 장비 없이 확인

```bash
# 매핑 앱: 합성 입력으로 촬영·저장·지도 생성 확인
bash tools/handheld_mapping.sh --demo

# 조종기: 화면과 입력만 확인, 네트워크·시리얼·SPI·ROS 출력 없음
PYTHONNOUSERSITE=1 .venv-mapping/bin/python \
  teleop/ksrc_snapshot/pi/teleop/teleop_joystick.py \
  --demo --max-lin 0.15 --max-ang 0.8 --rate 50

# 합성 지형·costmap 생성: 매번 새 출력 폴더 지정
bash tools/terrain_mapping.sh --demo --output outputs/terrain_demo_01
```

매핑 demo는 알려진 합성 pose를 사용하므로 native ORB 없이 실행할 수 있다. 실제 카메라·IMU 정확도나 주행 시험을 대신하지 않는다.

## 검증 상태와 남은 확인

2026-10-18 v3 소스 검증: **605 passed, 9 skipped**. 명령은 아래와 같다.

```bash
PYTHONNOUSERSITE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
QT_QPA_PLATFORM=offscreen OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv-mapping/bin/python -m pytest \
  tests/handheld_mapping teleop/ksrc_snapshot/pi/teleop/test_joystick_math.py -q
```

로컬 파일·합성 packet·별도 subprocess·모의 제어 출력을 사용해 연결 중단, 이어받기, 손상 파일, 저장 동기화 실패, Pi 정리, 터미널 표시, 45° 이동 명령을 검사했다. 선택적 의존성에 따른 건너뜀은 머신마다 다를 수 있다.

- v3 변경 후 실제 Pi의 카메라·IMU·SD·Wi-Fi 동시 부하, 전송·삭제, 실제 바퀴의 45° 조향은 아직 재검증하지 않았다.
- IMU는 함께 기록하지만 기본 ORB는 stereo다. 동적 IMU 융합과 카메라–IMU 시계 동기화는 별도 검증 대상이다.
- 중력 기준은 정지 IMU와 장착 모델로 추정한다. 카메라 장착 기준은 하향 20°이며 실측 지도 경사 정확도는 별도 확인이 필요하다.
- Wi-Fi 미리보기 손실과 Pi 원본 촬영 누락은 다르다. 영상 공백은 quality로 표시하며 무누락을 보장하지 않는다.
- 실시간 coverage 색과 1cm voxel은 정확도나 통행 가능성을 보증하지 않는다.
- `fsync`와 체크섬 검증 후에도 향후 PC 디스크 자체 고장에 대비하려면 별도 백업이 필요하다.

## 코드와 문서

| 경로 | 내용 |
|---|---|
| `experiments/handheld_mapping` | 앱, Pi 기록, 전송·복구·정리, ORB·TSDF·지형·costmap |
| `teleop/ksrc_snapshot` | 조종기와 확인한 KSRC 제어 소스 일부 |
| `tools` | 실행 wrapper, 상태 확인, native 빌드 도구 |
| `ros2_ws/src` | ROS adapter·patch·로버 URDF |
| `tests` | 합성·파일·subprocess·모의 제어 회귀 검사 |
| `docs`, `third_party` | 운영·설치·실험 기록과 외부 의존성 안내 |

- [설치 설명서](docs/installation.md): Python 환경과 native·Pi 준비
- [사용 설명서](docs/user-guide.md): 지도·지형·조종 상세
- [Pi 원본 기록 구조](docs/experiments/2026-10-18-local-recording-stream-design.md)
- [원본 정리·전송 터미널 정책](docs/experiments/2026-10-18-recording-retention.md)
- [지도 통합](docs/experiments/2026-10-16-session-merge.md) / [경사 색상 연속성 보정](docs/experiments/2026-10-18-slope-colour-continuity.md)
- [평면 보정 STL](docs/experiments/2026-10-18-structured-map-export.md)
- [별도 데이터·runtime 자산](docs/data-assets.md) / [외부 의존성과 라이선스](third_party/README.md)

이 저장소는 코드·문서 공유본이다. 촬영 PNG/PLY/Atlas, 실행 바이너리·vocabulary, venv, 전체 STM firmware와 대형 mesh는 별도로 준비한다. ORB-SLAM3 등 외부 프로젝트의 라이선스는 외부 의존성 안내를 따른다.
