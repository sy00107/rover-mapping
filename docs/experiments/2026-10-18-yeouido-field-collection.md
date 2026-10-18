# 여의도 센서·실측·매핑 수집

기존 `outputs/handheld_mapping`은 유지한다. 새 데이터 경로는
`/home/parkshinyoung/yeouido-data-collection/20261018`이며,
`/home/parkshinyoung/yeouido-data-collection/current.env`에 실행 경로와 Pi 주소가 있다.
Pi 주소가 바뀌면 env의 `FIELD_PI_HOST`/`FIELD_PI_ADDR`를 수정한다.

## 저장 항목

- 촬영 원본: `sessions/RUN/raw/{left,right,depth}/*.png` (mono8 IR, 16bit depth).
- 시계: 이미지 ROS header 시각 + Pi 수신 realtime/monotonic/RAW + 노트북 수신 monotonic.
  현장 모드만 RealSense metadata 원문을 추가 구독한다. publisher가 제공하는
  frame/sensor timestamp·exposure·gain·clock domain 원문을 보존하되 없으면 보고서에 누락 표시.
  ROS header stamp를 hardware clock의 원시값으로 이름만 바꿔 저장하지 않는다.
- IMU: MCU `t_ms`, Pi/노트북 수신 시계, raw/보정값을 `imu.jsonl`에 보존한다.
  MCU 제어주기 telemetry이므로 센서 내부 208Hz 샘플 전부라고 보장하지 않는다.
- 모터/encoder: 같은 raw telemetry의 selected/nominal/STM cmd, `enc_ticks`, `wheel_mps`를
  `derived/RUN/REV/telemetry.csv`로 추출한다. `steer_volt`는 조향 서보 전압이며 배터리 전압이 아니다.
- 카메라 projector는 기존대로 OFF. 노출/게인/auto-exposure/global-time 설정은 읽기만 한다.
  RGB는 켜지 않으므로 고정 노출 컬러 사진은 별도 카메라로 `photos/color/`에 직접 넣는다.
- GT: `ground_truth/measurements.csv`. 위치 m, 방향 도, 배터리 V. 미측정 값은 비워둔다.
  STL 기준 위치와 매핑 원점은 다르며 `dataset.json`의 `stl_from_mapping`을 실측 후 입력한다.
- 수동 표식: `field_events.jsonl`, run/trial ID + 노트북 시각. 체크포인트 IR은
  같은 RUN의 가장 가까운 노트북 수신 시각 프레임(3초 이내)에서 추출한다. 노출 시각 GT가 아니다.
- 공유 지도: 종료 후 저장된 기본 TSDF의 컬러 PLY와 최종 경로를 별도 생성한다.
  `derived/RUN/latest.json`과 상대경로만으로 로드해 다른 PC에서도 볼 수 있다.
  앱의 1/2/3m 거리별 보기는 원래 앱 기능으로 가능하며, 자동 공유 프리뷰는 기본 TSDF다.

## 실행 순서

아래 터미널은 모두 노트북이다. 앱만 열어서는 기록이 시작되지 않는다.
SPI owner는 터미널 1의 기존 bridge 하나만 사용한다.

### 터미널 1 — IMU·조종 중계 (계속 유지)

```bash
source /home/parkshinyoung/yeouido-data-collection/current.env
cd "$FIELD_CODE_ROOT"
bash tools/mapping_imu_bridge.sh "$FIELD_PI_HOST"
```

카메라를 별도로 ROS launch하지 않는다. 앱이 Pi camera worker를 시작한다.

### 터미널 2 — 주행 전 IMU → 매핑 → 주행 후 IMU

```bash
source /home/parkshinyoung/yeouido-data-collection/current.env
cd "$FIELD_CODE_ROOT"
bash tools/field_collection.sh imu "$FIELD_DATASET" --phase pre --seconds 60 --host "$FIELD_PI_HOST"
bash tools/field_collection.sh app "$FIELD_DATASET"
# 매핑/공유 정리를 마치고 앱 창을 닫으면 다음 명령으로 진행
bash tools/field_collection.sh imu "$FIELD_DATASET" --phase post --seconds 60 --host "$FIELD_PI_HOST"
```

IMU-only 단계에서는 로버를 1분 정지한다. 매핑과 동시에 IMU-only를 켜면
같은 UNIX socket을 서로 빼앗으므로 수집 도구가 차단한다.
매핑 앱에서 IMU·중력 기준을 켜고 시작, 초기 정지, 수동 주행,
정지 후 종료 및 지도 생성 순서로 진행한다. 종료 후 공유 CSV/PLY 자동 생성까지 기다린다.

### 터미널 3 — 조종

```bash
source /home/parkshinyoung/yeouido-data-collection/current.env
cd "$FIELD_CODE_ROOT"
PYTHONNOUSERSITE=1 .venv-mapping/bin/python \
  teleop/ksrc_snapshot/pi/teleop/teleop_joystick.py \
  --udp "$FIELD_PI_ADDR" --rate 50 --max-lin 0.05 --max-ang 0.3 --monitor
```

Space 유지=직진, 떼면 정지. 기존 1/2/3배 선택 유지. 매핑 종료는 모터 정지가 아니다.

### 터미널 4 — 구간·실측 표식

```bash
source /home/parkshinyoung/yeouido-data-collection/current.env
cd "$FIELD_CODE_ROOT"
bash tools/field_collection.sh console "$FIELD_DATASET"
```

콘솔 입력 형식 (숫자는 반드시 현장 실측값으로 입력):

```text
begin sand_straight_01 sand
cp CP0 X미터 Y미터 HEADING도
note 모래 직선 첫 번째; 날씨와 관찰 기록
battery 실측전압
cp CP1 X미터 Y미터 HEADING도
distance 명령기준거리미터 실제이동거리미터
end
begin hard_straight_01 hard
...
end
quit
```

`cp`는 마킹 지점에 2–3초 정지한 동안 입력한다. `distance`는 목표/기준 거리와 줄자 거리이며,
그 차이를 곧바로 바퀴 슬립률로 판정하지 않는다. 슬립 분석은 encoder/wheel 거리와 비교한다.
한 촬영 RUN 안의 여러 trial은 begin/end 시각으로 구분한다. 새 RUN마다 별도 trial ID를 쓴다.

### 터미널 5 — 지도 보기 / 최종 정리 / 공유

```bash
source /home/parkshinyoung/yeouido-data-collection/current.env
cd "$FIELD_CODE_ROOT"
bash tools/field_collection.sh view "$FIELD_DATASET"
```

아래 최종 정리는 앱을 닫은 후 실행한다. 이후 수정한 사진/실측 자료도 함께 압축된다.

```bash
source /home/parkshinyoung/yeouido-data-collection/current.env
cd "$FIELD_CODE_ROOT"
bash tools/field_collection.sh finalize "$FIELD_DATASET"
bash tools/field_collection.sh pack "$FIELD_DATASET" \
  --output "/home/parkshinyoung/yeouido-data-collection/share/yeouido_$(date +%Y%m%d_%H%M%S).tar.gz"
```

`pack`은 앱/IMU 기록 중 거절하며 기존 압축 파일을 덮어쓰지 않는다. 데이터 전체의
SHA256 manifest와 압축 파일 옆 `.sha256`을 생성한다. 실제 업로드·팀원 전송은 수행하지 않는다.

## 팀원

tar.gz와 sha256을 함께 전달한다. 받은 파일을 검증하고 해제한 뒤 데이터 폴더 안에서:

```bash
sha256sum -c 받은파일.tar.gz.sha256
tar -xzf 받은파일.tar.gz
cd 20261018
python3 verify_dataset.py
# numpy/open3d가 설치된 환경에서
python3 view_maps.py
```

코드 저장소나 원래 노트북 절대경로 없이 PLY/최종 경로를 볼 수 있다. 여러 독립 map은
서로 다른 좌표계일 수 있어 한 창에 임의 병합하지 않고 map별로 연다.
`code_snapshot/`은 촬영 설정 당시 Python 코드 사본이다. native 실행기와 vocabulary,
venv는 별도 설치 자산이며 원본 전체를 Git에 올리지는 않는다.

## 검증 범위

장비 없는 synthetic capture→종료→자동 CSV/컬러 지도 생성, 실측 이벤트/RUN 연결,
전체 telemetry 필드 추출, archive 이동 후 manifest 검증, 손상 탐지, 동시 수집 차단,
카메라 metadata 시계/원문 보존을 테스트한다. 현장 Pi/STM/카메라 기록과 대역폭은
현장에서 확인해야 하며 metadata/encoder 미수신을 완료로 표시하지 않는다.
