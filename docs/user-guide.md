# 사용 설명서

기준 날짜: 2026-10-15. 모든 노트북 명령은 **clone한 `rover-mapping` 루트**에서 실행한다. 먼저 [설치 설명서](installation.md)를 따른다. 이 문서는 운영 절차이고, 구현/검증 상태는 [인수인계서](handoff/d435-rover-mapping-handoff.md)에 있다.

## 1. 장비 없이 먼저 확인

```bash
bash tools/handheld_mapping.sh --demo
```

1. **합성 입력으로 시험**이 선택됐는지 확인한다.
2. **매핑 시작**을 누르고 원본 IR과 frame 수가 갱신되는지 확인한다.
3. **종료 및 지도 생성**을 누르고 저장 완료를 기다린다.
4. **저장 폴더 열기**, **3D 지도 보기**로 원본과 TSDF 결과를 확인한다.

합성 입력은 알려진 pose를 사용한다. native ORB 없이 실행 가능하지만 native 실행기가 없으면 live 안내 옵션은 비활성화된다. demo가 실제 카메라/SLAM 정확도 시험은 아니다.

## 2. 실제 촬영 준비

최신 로버의 카메라 장착각은 사용자 확인 **하향 20°**다. [최신 URDF·센서 좌표](experiments/2026-10-14-rover-extrinsics-update.md)를 따른다. 기본 stereo 지도는 카메라 기준이다. **중력 기준 지도**를 선택하면 같은 RUN의 보정 IMU와 보존된 URDF·CameraInfo rectification으로 map별 z-up 기준을 추정한다. 장착 변환은 여전히 실측 검증 전이며 [중력·지형 처리 범위](experiments/2026-10-15-gravity-terrain-pipeline.md)를 확인한다.

| 준비 | 확인 |
|---|---|
| 카메라 | D435를 Pi에 연결. 케이블·포트·실제 USB 속도 확인 |
| 노트북 native | `outputs/handheld_mapping/orb3_comparison_build/rover_orb3_compare`와 runtime/vocabulary 준비 |
| Pi | native Jazzy/RealSense 설치, 기존 runtime/profile 경로 확인 |
| 통신 | 노트북→Pi SSH 접속. 조종할 경우 UDP5005도 별도 확인 |
| 중복 실행 | 이전 camera/ORB 정상 종료. SPI bridge는 단일 owner |
| 저장 | 시작 전 노트북 여유2GB 이상. 원본과 지도를 보존할 공간 준비 |

```bash
ssh -o BatchMode=yes -o ConnectTimeout=8 pi@raspberrypi.local hostname
bash tools/handheld_mapping.sh
```

Pi IP가 달라졌으면 앱의 **Pi 연결**을 `pi@현재_IP`로 바꾼다. 과거 hotspot IP를 그대로 쓰지 않는다. 앱을 열기만 해서는 카메라·모터가 시작되지 않는다.

native가 준비되면 다음 두 옵션을 사용한다.

- **Pi는 센서 전송만 · 종료 후 노트북에서 ORB 계산**: ON.
- **실시간 촬영 범위 표시 · 노트북에서 ORB 동시 실행**: ON.
- 처리 방식은 **ORB-SLAM3 + TSDF**로 시작한다. 점선 비교는 선택적 후처리다.

## 3. 촬영 시작·이동·종료

IMU를 함께 기록하려면 [IMU 동시 기록 절차](experiments/2026-10-14-imu-co-recording.md)에 따라 `tools/mapping_imu_bridge.sh`로 Pi 중계를 실행하고, 앱의 **보정 IMU 함께 기록**과 실제 보정 JSON 경로를 확인한다. IMU 유효 샘플 확인 후 카메라가 시작되며 데이터는 같은 RUN에 저장된다. **중력 기준 지도와 높이·경사·단차 생성**을 켜면 촬영 시작 후 최소3초간 정지하고 오른쪽의 중력 기준 X-Y 표시를 기다린다. GUI의 ORB는 계속 stereo이며 IMU를 동적으로 융합하는 모드는 아니다.

IMU의 **가속도 참고 roll/pitch**는 Pi 표본 시각 기준0.2초 평활화를 거친 진단 표시이며, 가속·진동 중에는 실제 자세와 다를 수 있다. 알려진 평지에서3초 정지한 뒤 **평지 영점**을 누르면 표시 기준만 설정한다. 경사면에서 누르지 않는다. 센서 보정 원본·기록된 보정 측정값·중력 지도 기준은 바꾸지 않으며 영점 이력은 `imu_display_events.jsonl`에 남긴다. 재시작된 MCU segment에는 이전 표시 영점을 적용하지 않는다.

1. 저장 폴더를 선택한다. 기본값은 clone 경로의 `outputs/handheld_mapping/sessions`다.
2. **매핑 시작**을 누른다. 매번 새 RUN을 만들고 worker를 Pi RAM disk에 배포한다.
3. 정지 상태에서 카메라 준비와 오른쪽 추적 정상 표시를 기다린다. 초기 vocabulary 로딩으로 live pair를 일부 건너뛸 수 있다.
4. 작은 구역을 천천히 이동하며 표면을 여러 위치에서 겹쳐 본다. 시작점으로 돌아와 재방문한다. 거리/단차를 실측할 수 있으면 함께 기록한다.
5. 로버를 사용했다면 **먼저 조이스틱으로 정지**한다. 매핑 종료는 모터 정지가 아니다.
6. **종료 및 지도 생성**을 누른다. 정상 마감·전송·offline ORB·TSDF가 끝날 때까지 Pi 전원을 유지한다.
7. `complete`/`partial`/`failed`와 RUN 경로를 확인한다. 종료 상태와 지도 품질 합격은 다르다.

저장 후 **3D 보기 설정**에서 지도 종류·최대 거리·표면 보정을 고르고 **3D 지도 보기**를 누른다. 기본은 **전체 카메라 지도 우선 · 2m · 표면 보정 켜짐**이다. 1m·3m 또는 저장된 기본 지도도 선택할 수 있으며 IMU 융합 시험 지도는 별도로 선택한다. 첫 거리별 생성에는 시간이 걸리고 이후에는 재사용한다. 이미 생성된 통합·거리별 폴더는 해당 결과의 저장 설정으로 열린다.

창 닫기/Ctrl+C도 소유 세션의 정상 마감을 요청한다. 촬영 중 강제 전원 차단이나 임의 killall을 정상 종료로 대신하지 않는다. 실패해도 저장된 원본은 남는다.

## 4. 화면과 지표 읽기

| 화면 / 지표 | 의미 |
|---|---|
| 왼쪽 위 | 원본 IR |
| 왼쪽 아래 | 같은 frame의 관측색 오버레이 |
| 오른쪽 | 잠정 coverage·camera 위치/경로·시야 방향 |
| 회색/노랑/초록 | 미관측 /1~2회 /3회 이상 시점 변화 관측 참고값 |
| 저장 frame 수 | 노트북에 실제 저장한 stream별 수 |
| 전송 drop | Pi queue 거절 누적 |
| live 건너뜀 | FIFO256쌍 포화로 거절한 새 stereo pair. 원본 저장·전송 drop과 별개 |
| 수신 후 ms | 노트북 stereo 저장 후 pose 수신까지. 전체 지연 아님 |

카메라 입력 요청15FPS, 화면/overlay는 최대5Hz다. 초록은 정확도·통행 가능성·충분한 모든 시점의 보증이 아니다. 기본 coverage는 초기 optical X-Z 투영이고, 중력 기준을 얻었으면 고정된 z-up 좌표의 X-Y 투영이다. 어느 경우든 벽이나 서로 다른 높이의 표면이 투영에 들어올 수 있으며 coverage는 지면 분류가 아니다. 중력 기준 차체 기울기는 지면 경사와 구분해 표시한다. 5° 진입/3° 이탈을1초간 유지하면 구간 이벤트를 기록하지만 ORB/IMU 모드를 전환하지 않는다.

추적 손실/유효 depth 없음/오래된 결과에서는 색을 제거한다. live가 실패해도 원본 기록과 종료 후 전체 재계산을 유지한다. 최종 맵은 live 잠정 격자와 다를 수 있다.

`기존 지도 복구 중`이면 멈춰서 이미 찍은 장면을 천천히 다시 보여준다. stereo는 영상 시각 기준 최대15초 동안 기존 지도 재위치 추정을 시도하며, 그동안 이전 coverage·경로는 남기고 유효하지 않은 위치·관측은 추가하지 않는다. 특징점과 겹침이 부족하면 복구하지 못하고 별도 지도를 만들 수 있다. 다른 좌표계의 지도를 임의로 이어 붙이지 않는다. [변경·검증 기록](experiments/2026-10-15-tracking-recovery.md).

후속 수정은 첫 키프레임만 있는 지도도 복구 검색에 등록하고, 저장 중 관측 삭제의 충돌을 보완한다. [초기 지도 복구·저장 안전성](experiments/2026-10-15-initial-recovery-and-save-safety.md). 구버전 native core가 거절되면 [설치 설명서](installation.md)의 기존 설치 갱신 절차를 따른다.

## 5. 조이스틱 조종

매핑 앱과 별도다. 기존 Pi/STM 준비·배선·정지 응답을 확인한 뒤 사용자가 조작한다. 코드 변경은 fake/시뮬레이션을 먼저 검증하고 실제 기체는 바퀴를 띄워 방향·정지부터 확인하는 것을 권고한다.

Pi 별도 터미널:

```bash
cd /home/pi/ksrc
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -u pi/common/udp_spi_bridge.py \
  --hz 50 --max-lin 0.05 --max-ang 0.3
```

노트북 저장소 루트:

```bash
PYTHONNOUSERSITE=1 .venv-mapping/bin/python \
  teleop/ksrc_snapshot/pi/teleop/teleop_joystick.py \
  --udp raspberrypi.local --rate 50 --max-lin 0.05 --max-ang 0.3 --monitor
```

- 왼쪽 패드: `vx/vy` 병진. 오른쪽 패드: 회전. Q/E를 누르며 병진을 줄 수 있다.
- **Space를 누르는 동안 직진**, 떼면 정지한다. 숫자 **1/2/3**(숫자패드 포함) 또는 화면 버튼으로 `--max-lin` 기준 **1/2/3배**를 선택한다. 시작은 1배이며 선택한 속도를 화면에 표시한다. Space를 누르는 중에도 변경할 수 있고, 선택만으로 출발하지 않는다. Space를 누르는 동안에는 패드와 Q/E보다 직진이 우선하여 횡이동·회전은 0이다.
- 마우스를 놓으면 해당 패드0, 포커스 상실은 전체0, Esc는 정지 후 종료다. 포커스가 돌아와도 Space를 새로 눌러야 다시 직진한다. 실제 정지 응답도 확인한다.
- 속도 제한은 bridge와 조이스틱 **양쪽**에 있다. 병진 m/s, 회전 rad/s다. 위 값은 저속 시작 예시다.
- `--max-lin 0.15`로 조이스틱을 실행하면 Space 속도는 **0.15 / 0.30 / 0.45 m/s**다. `tools/mapping_imu_bridge.sh`의 병진 상한은 이 3배를 허용하도록 **0.45 m/s**이며 변경 후 중계도 재시작해야 한다. 위의 수동 bridge 예시처럼 상한이 0.05이면 그 이상 명령은 제한된다. 마우스 패드의 병진 속도와 회전 속도는 배율의 영향을 받지 않는다.
- 조이스틱의 오른쪽 패드와 Q/E 회전 명령은 `--max-ang`의 **50%**로 전송한다. 위 예시의 `0.3`이면 최대 `0.15 rad/s`(약 `8.6°/s`)이며 병진 속도는 그대로다. Pi bridge는 받은 회전 명령을 자체 `--max-ang`으로 제한한다.
- SPI owner는 하나만 실행한다. bridge 중에 별도 SpiLink monitor를 동시에 열지 않는다.
- `--monitor` 요약은 전체 IMU/encoder 동시 기록이나 관성융합이 아니다.
- Pi의 원래 `/home/pi/ksrc`와 snapshot의 버전/firmware가 다르면 먼저 대조한다. 이 저장소는 전체 STM firmware를 포함하지 않는다.

SPI 배선: Pi 물리23(SCLK)→PB13,19(MOSI)→PC3,21(MISO)→PC2,24(CE0)→PB12,20 또는25(GND)→GND. 상세는 [인수인계서](handoff/d435-rover-mapping-handoff.md)를 따른다.

## 6. 저장 결과 확인·3D 보기

```text
RUN/
  session.json, capture_calibration.json, frames.jsonl, events.jsonl
  raw/{left,right,depth}/*.png
  orb/camera.log, worker.log
  orb_live/                    잠정 live pose/coverage
  orb_offline/                 최종 point pose/Atlas/keyframe
  dense_offline/               sensors-only 기본 TSDF
  dense/                      Pi ORB 또는 합성 TSDF
  orbline/, dense_orbline/      선택적 점선 결과
  gravity/BACKEND/map_ID.json  map별 고정 중력 기준·입력 checksum
  terrain/BACKEND/map_ID/      높이·경사·단차·거칠기·cost·preview
  orb_inertial/, dense_inertial/ 실측 보정 profile이 필요한 선택적 관성 후처리
```

`session.json.state`, drop/counts, native `runner_complete.json`, dense `summary.json`의 융합 수·누락·map 수를 확인한다. `complete`는 절차 완료이며 실측 정확도 판정이 아니다. map ID `0/421`은 두 맵의 이름이다.

촬영은 좌·우 exact 시각과 2ms 이내 depth를 묶어 Pi 전송 큐에 전부 넣거나 전부 거절한다. depth를 먼저 저장해 live stereo 처리보다 늦게 도착하는 경쟁을 줄인다. 노트북은 별도 수신 thread와 기본32MiB 버퍼로 PNG 저장·live 처리의 일시 지연을 흡수한다. 버퍼가 계속 차면 역압이 걸리며 지속적인 대역폭 부족까지 해결하지는 못한다. `capture_metrics.bundles`, stream별 drop, `laptop_capture.receive_buffer`, 프레임별 `pi_bundle_wait_ms`·`pi_queue_wait_ms`·`laptop_receive_queue_wait_ms`·PNG 시간을 확인한다. 큰 ORB artifact 하나는 프로토콜 상한128MiB 내에서 수신 버퍼 기본 크기를 넘을 수 있다.

TSDF와 지형 재생에는 depth 고립값·깊이 경계 필터와 최종 pose 기반 주변 시점 일치 검사를 기본 적용한다. raw PNG·카메라 보정·ORB 위치는 바꾸지 않는다. 주변3초 안에 참조 키프레임이 없으면 공간 필터만 적용하며 `frames_without_reference`에 센다. 검사되지 않은 영역을 채우지 않는다.

3D 뷰어는 같은 map의 최신 지면 분리 결과가 있으면 **초록=지면 / 주황=장애물 표면 / 회색=미분류**, 파랑=추정 경로로 표시한다. 같은 XY라도 표면의 높이와 국소 지면을 비교해 색을 나눈다. 초록이 통과 가능 판정을 뜻하지는 않는다. 지형이 없거나 출처가 맞지 않으면 단색으로 열고 이유를 안내한다. `--no-terrain-colors`로 끌 수 있으며 자세한 기준은 [뷰어 설명](experiments/mapping-viewer.md)에 있다.

같은 기록을 거리별로 비교하려면 `bash tools/view_mapping.sh /absolute/path/to/RUN --depth-max 1`을 실행하고 숫자를2·3으로 바꾼다. 처음에는 해당 최대 depth로3D·지형을 함께 재처리하며 이후에는 입력과 코드가 같은 결과를 재사용한다. 원본과 기본 지도는 그대로 두고 `RUN/depth_variants/BACKEND/depth_1m/` 등에 저장한다. `--build-only`를 추가하면 창 없이 생성만 한다. 거리 기준은 카메라 optical Z이며 전체 지도 반경이 아니다. [1/2/3m 실행 순서와 결과 폴더](experiments/mapping-viewer.md)를 참고한다.

```bash
# 최신 RUN
bash tools/view_mapping.sh
# 실제 본인 RUN으로 교체
bash tools/view_mapping.sh /absolute/path/to/RUN
bash tools/view_mapping.sh /absolute/path/to/RUN --map-id 0
bash tools/view_mapping.sh /absolute/path/to/RUN --cloud
bash tools/view_mapping.sh /absolute/path/to/RUN --backend orbline
# 직접 PLY: backend/map-id를 함께 주지 않음
bash tools/view_mapping.sh /absolute/path/to/map_0_mesh.ply
```

서로 다른 map은 별도 창·독립 좌표계다. 근거 없이 합치지 않는다. 왼쪽 드래그 회전, 휠 확대·축소, Shift+왼쪽 드래그 이동이다. 최신 RUN에 아직 지도가 없으면 오류를 보여주며 이전 RUN으로 자동 돌아가지 않는다.

3D 지도에는 같은 map/backend의 **최종 ORB 키프레임 기반 카메라 추정 경로**를 기본으로 표시한다. 파란 선은 시간순 위치 연결, 초록 구는 시작, 빨간 구는 끝이다. 중력 기준 TSDF이면 경로에도 동일한 고정 변환을 적용한다. mesh와 cloud 모두 지원하며 기존 촬영도 최종 키프레임 CSV가 있으면 바로 표시한다. 로봇 중심·지면으로 보정하거나 실측 이동 경로로 간주하지 않는다. 키프레임 사이의 움직임 전체를 복원한 선은 아니며, 최종 지도와 좌표가 달라질 수 있는 live 경로는 섞지 않는다. 대응하는 최종 키프레임이 없으면 지도만 표시한다. 경로를 숨기려면 `bash tools/view_mapping.sh --no-trajectory`를 사용한다. viewer는 원본 PLY를 수정하지 않는다.

## 7. 재계산과 점선 비교

중요한 기준 맵은 RUN 전체를 보존한 사본에서 재계산한다.

```bash
# ORB만 새로 계산. orb_offline이 있으면 덮어쓰지 않음
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv-mapping/bin/python -m experiments.handheld_mapping.offline /absolute/path/to/RUN

# 선택적 point+line. 기존 orbline이 있으면 덮어쓰지 않음
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv-mapping/bin/python -m experiments.handheld_mapping.compare /absolute/path/to/RUN
```

TSDF만 다시 만들려면 GUI **기록에서 다시 생성**을 사용한다. 이는 ORB 재추정 버튼이 아니다. sensors-only RUN의 backend를 명시할 때:

```bash
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv-mapping/bin/python - /absolute/path/to/RUN <<'PY'
from pathlib import Path
import sys
from experiments.handheld_mapping.reconstruct import reconstruct
print(reconstruct(Path(sys.argv[1]), backend="orb_offline"))
PY
```

TSDF는 해당 dense 출력을 갱신하고 raw는 보존한다. `reconstruct` CLI의 default는 `orb`이며 backend flag는 없으므로 위 API 또는 GUI를 사용한다. point+line의 정확도 이득은 아직 입증하지 않았다.

## 8. 지형·costmap 생성

```bash
# 합성 예제. 새 output 필요
bash tools/terrain_mapping.sh --demo --output outputs/terrain_demo_02

# 실제 기록: 검증한 z-up 정렬과 map ID 필요
bash tools/terrain_mapping.sh /absolute/path/to/RUN \
  --backend orb_offline --map-id 0 \
  --alignment /absolute/path/to/grid_from_world.json \
  --bounds -5 5 -5 5 --resolution 0.05 --stride 4 \
  --output outputs/terrain_real_01
```

alignment는 ORB world→z-up gravity frame의 4×4 rigid transform JSON 배열이다. 위 CLI에서는 사용자가 검증한 변환을 주며 identity나 다른 RUN의 변환을 임의 사용하지 않는다. GUI의 중력 기준 옵션은 같은 RUN의 정지 IMU로 이를 추정해 지도와 지형 층을 함께 생성한다. 결과는 **높이·경사 보기**로 연다. 지면 분리·지형 정확도·로버 통과 한계의 실기 검증은 남아 있다.

결과는 `terrain.npz`, `summary.json`, `preview.png`다. NPZ는 `np.load(path, allow_pickle=False)`로 읽고 배열 축은 `(y,x)`다. 높이/단차/거칠기는 m, 경사/방향은 degree다. cost는 유효0..252, blocked254, unknown255이고 unknown을 평지로 채우지 않는다.

실제 재생은 셀별 최대3개 높이군 중 반복된 아래쪽 표면을 후보로 삼고, 카메라 주변 아래쪽 국소 면에서 연결된 표면만 지면으로 쓴다. 전역 평면을 빼지 않아 실제 경사를 보존한다. `height_m`·`slope_deg`·`roughness_m`는 지면 층이며, `obstacle_height_m`는 가까운 지면 대비 위쪽 물체 높이다. `ground`, `surface_observed`, `unclassified_surface`로 지면과 미확정 표면을 구분한다. 가려진 바닥·연결되지 않은 표면은 unknown으로 남을 수 있고 seed/연결 기준은 heuristic이다. preview는 지면 높이·경사·거칠기·장애물 높이·표면 분류·cost 6개 패널이다. 합성 demo는 기존 기하 층의 기준 시험이다.

prototype 기본 한계는 경사20°/단차0.06m/거칠기0.025m/원형여유0.15m이며 실제 로버 한계로 확정하지 않았다. `--rover-limits FILE.json`으로 조정한다. coverage와 별개 offline 도구이며 live costmap/Nav2/슬립·토질 추정은 미연결이다.

## 9. 문제 해결·자료 이전

| 증상 | 먼저 확인 |
|---|---|
| hostname 조회 실패 | 현재 Pi IP·네트워크·SSH 인증. GUI host 수정 |
| D435 미검출/USB -71 | 케이블·접촉·포트·전원·USB 속도·camera/kernel 로그 |
| 이동 중 끊김/drop | source/receipt gap → queue/compress/write → 노트북 PNG 시간 |
| 특징점은 많은데 손실 | keypoint 수와 map-point inlier는 다름. 공백/blur/시야 변화 확인 |
| 지도 기울어짐 | IMU 융합·자동 중력 정렬 없음. alignment 검증 |
| 지도 두 창 | map 분리. 각 map 좌표계를 임의 합치지 않음 |
| Open3D 창 오류 | wrapper와 로컬 DISPLAY/X11 사용 |
| FileExistsError | 기존 결과 보호. 새 사본/output 사용 |
| 조종 명령과 STM 적용 불일치 | UDP/SPI/CRC/watchdog/조향 gate 확인 |

새 계측은 `session.json.capture_metrics`, `laptop_capture`, `frames.jsonl`에 남는다. Pi/laptop monotonic 절대값을 서로 빼지 않는다. 최종 drop은 후처리 실패에도 보존된다. 자세한 판독은 [입력 공백 조사](experiments/2026-10-14-input-gap-investigation.md)를 따른다.

IMU의 현재 수신이 회복되면 **수신 지연** 표시는 해제된다. `delivery_delay_events`는 노트북 도착 간격500ms 초과 횟수이며, 뒤늦게 모두 수신된 지연만으로 세션을 부분 실패 처리하지 않는다. 실제 MCU control packet 간격120ms 초과(`source_gap_events`), reset/clock jump/tee 누락 또는 종료 시 수신 정지는 `partial`에 반영한다. 두 계측은 각각의 clock 내부 간격이며 개별 IMU 취득 시각이나208Hz 원시 표본 품질을 뜻하지 않는다.

RUN을 옮길 때 raw/보정/index/최종pose/summary/PLY/log를 함께 보존한다. native `pairs.tsv`의 절대경로는 새 위치에 맞춰 재생성한다. 이전 실기 RUN과 native 자산은 Git에 없으며 [별도 자산 안내](data-assets.md)에 정리돼 있다.

## 10. 개발 검증과 다음 순서

```bash
KSRC_REVIEW_ROOT="$PWD/teleop/ksrc_snapshot" \
QT_QPA_PLATFORM=offscreen PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv-mapping/bin/python -m pytest -q
```

fake SPI/합성 테스트이며 실제 모터를 움직이지 않는다. optional native 실행기가 없으면 해당 load test는 skip된다. source snapshot과 공유본 테스트 결과는 [공유본 검증](share-validation.md)을 본다.

다음 작업은 **입력 연속성 계측 → IMU/encoder 기록·시각/장착 검증 → 실측 지도 품질 → 중력/지형 검증 → 고정 스캔/STL을 사용한 별도 주행 보정** 순서다. COLMAP은 offline 비교 후보이며 아직 통합하지 않았다.

매핑 앱의 **평면 보정 STL 내보내기** 버튼은 선택된 backend와 거리 설정의
지면을 평지/경사별 평면으로 복원합니다. 중력 좌표계를 회전하지 않으며
경사면의 측정 굴곡은 평면 모델로 제거합니다. 작은 닫힌 구멍(8cm 이내)만
메우고 외곽과 큰 미관측 틈은 유지합니다. 장애물은 원본 3D 표면을 보존합니다.
완료되면 `exports/planar_날짜시간/` 폴더가 열립니다. 각 Map의 `planar/` 안에
`ground_solid_m.stl`, `ground_solid_mm.stl`(추정 두께 8cm의 닫힌 지면),
`scene_surface_m.stl`(장애물을 포함한 열린 관측 표면), `scene_colored.ply`
(초록 평지/핑크 경사/주황 장애물)이 있습니다. STL 자체에는 색이 없습니다.
현재 평면 접합은 볼록한 평지–오르막 모델이며, 충분한 관측과 모델 적합성이
확인되지 않으면 실패합니다. 임의 지형 전체에 일반화한 복원은 아닙니다.
생성 중 중복 내보내기와 새 촬영은 비활성화됩니다. 실패 원인은 촬영 폴더의
`joined_export.log`에 기록합니다. 원본과 이전 결과는 보존합니다.

평면 보정 STL/PLY 내보내기의 원점은 **첫 최종 카메라 위치 바로 아래의 보정 지면**입니다.
XYZ 축과 중력 방향은 유지하고 모든 모델과 경로에 같은 평행이동을 적용합니다.
출발점 바로 아래 관측이 없으면 보정 평면을 연장해 높이를 추정하며,
`summary.json`에 추정 여부와 인근 지면 관측까지의 XY 거리를 기록합니다.
`export_from_source`/`source_from_export`는 metre 기준 좌표변환이고,
`planes`는 원본 좌표, `planes_in_export`는 내보낸 좌표의 평면식입니다.
`trajectory_m.csv` 역시 내보낸 원점 기준이며 카메라 높이는 그대로 남습니다.
