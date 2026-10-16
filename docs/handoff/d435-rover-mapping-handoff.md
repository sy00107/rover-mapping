# D435 · Raspberry Pi · 로버 매핑 인수인계

갱신: **2026-10-15** · 작업 위치: **`/path/to/rover-mapping`**

최신 기구 기준: 사용자 확인 카메라 **하향 20°**, 새 장착 좌표와 headless 보정 지원은 [URDF 갱신 기록](../experiments/2026-10-14-rover-extrinsics-update.md)을 따른다. 기본 Qt 지도는 카메라 기준이다. 후속 [중력·지형 경로](../experiments/2026-10-15-gravity-terrain-pipeline.md)는 정지 IMU 기반 z-up 정렬과 GUI 지형 생성을 추가한다. 동적 융합은 실측 동기화·장착·잡음 검증이 필요한 별도 native offline 입력이며 GUI live는 stereo를 유지한다.

2026-10-15 추적 갱신: 노트북 FIFO256쌍·depth 대기 제거·최대5Hz 관측색 갱신·입력 지연을 포함한 stale 표시와 stereo15초 복구 대기를 추가했다. 기존 설치는 core patch와 runner를 재빌드한다. [복구 비교·검증 범위](../experiments/2026-10-15-tracking-recovery.md). IMU는 계속 동시 기록이며 관성 융합은 별도다.

후속 수정: KF0의 복구 후보 등록과 `Map/MapPoint::PreSave`의 삭제 중 순회 안전성을 추가했다. native6개 실제 검사·종료 충돌 재현과 범위는 [후속 검증](../experiments/2026-10-15-initial-recovery-and-save-safety.md)을 따른다. native 설치 patch는5개이며 초기 복구 marker와 저장 안전성 marker도 요구한다.

이 문서는 이번 D435/Pi/로버 작업의 현재 상태와 사용법을 합친 기준 문서다. 다른 PC로 옮기면 `/path/to/rover-mapping`와 Pi 경로를 실제 설치 위치로 바꾼다. 기존 날짜별 문서의 “실기 연결 전” 같은 표현은 작성 당시 상태이므로 최신 판단에는 이 문서와 연결된 2026-10-14 검증 자료를 사용한다.

공유본 안내: `/path/to/rover-mapping`은 본인의 clone 경로로 교체한다. 날짜별 RUN/실제 runtime은 Git에 없으며 [별도 자산 안내](../data-assets.md)를 따른다. 처음 사용할 때는 [사용 설명서](../user-guide.md)를 먼저 읽는다.

## 1. 목표와 현재 위치

경기는 다른 경기장에서 진행되고 **3D 사전 맵인 STL**이 제공된다. 현재 실기 장소에는 사전 맵이 없으므로 D435로 기준 3D 맵을 만든 뒤, **별도 주행**에서 그 고정 맵을 보고 누적 위치 오차를 보정하는 것이 목표다. 기준 맵 촬영 A와 보정 평가 주행 B는 분리한다.

형상에서 높이·경사·거칠기·단차·장애물·관측 정보를 얻고, 로버 조건에 따른 costmap을 만들어 향후 달탐사 로봇의 주행 판단에 활용하려 한다. 노트북에서 무거운 계산을 할 수 있으며 Pi에는 원본을 쌓지 않는 구성을 우선한다.

**현재 단계는 영상 기록을 확보한 1단계 후반~기준 맵 품질을 검증하는 2단계다.** 실시간 촬영 안내와 지형/costmap 기반 코드를 먼저 만들었지만, 실측 지도 정확도와 고정 맵 기반 보정을 완료한 것은 아니다.

| 항목 | 현재 구현·확인 | 남은 일 |
|---|---|---|
| D435 → Pi → 노트북 | 실제 IR 좌우/depth/보정값/시각 기록·정상 종료·재생 | 장시간 연속성, 새 계측을 적용한 실기 병목 확인 |
| ORB-SLAM3 | stereo 점 기반 추적, live 안내와 종료 후 전체 원본 재계산 | 입력 공백·맵 분리 감소, 실측 정확도 평가 |
| TSDF | 실제 D435 depth + 최종 keyframe pose로 map별 mesh/cloud | 중력 정렬, 재방문 중복 표면·거리·단차 검증 |
| 촬영 화면 | 원본 위/관측색 아래, 오른쪽 관측 격자·경로 | 현 Pi/USB/무선에서 새 기능 전체 부하 검증 |
| 조이스틱·STM | UDP → SPI 통신, 사용자 조작·정지 응답 확인 | 매 시험 연결·정지·조향 gate 상태 재확인 |
| IMU·encoder | 보정 IMU·원본 telemetry의 encoder 필드를 영상과 같은 RUN에 기록, 실제 짧은 촬영 확인 | acquisition clock·축/장착변환 검증·관성 융합 |
| ORB-LINE | 독립 offline 실행기, 노트북·서버 기능 재생 확인 | 공정한 정확도 비교, 실제 D435 이득, Pi 실시간 배포 |
| 지형/costmap | 고정 크기 격자·기록 재생·합성 검증 | 실제 중력/지면 분리·로버 한계 검증·live 연결 |
| 기준 mesh/STL 위치 보정 | 기존 벽/방향 실험과 로드맵 | 일반 스캔/STL 공통 입력과 별도 주행 보정 통합 |
| AI HAT | 이전에 Hailo-8 인식 확인 | 현재 ORB/LSD/TSDF 가속 연결 없음 |
| COLMAP | 촬영 후 pose/형상 비교 후보로 정리 | 설치·실행·통합·동일 기록 비교 전 |

IMU 동시 기록의 실행·저장·검증 범위는 [사용법](../experiments/2026-10-14-imu-co-recording.md)을 따른다. Qt에 IMU 옵션을 추가했으며, ORB는 계속 stereo다.

## 2. 현재 알고리즘과 데이터 흐름

```text
D435: 좌측 IR + 우측 IR + depth (640×480, 요청 15 FPS, projector OFF)
  ↓ Pi ROS 2 Jazzy: 수집·CameraInfo/TF/profile 검증
  ↓ bounded queue → zlib → SSH stdout binary packet
노트북: raw PNG + metadata 기록
  ├─ 촬영 중: native ORB-SLAM3 stereo → 잠정 pose → 관측 격자/IR 오버레이
  └─ 촬영 종료: 전체 저장 IR → native ORB-SLAM3 → 최종 map별 keyframe
                                           + 대응 D435 depth
                                           ↓ Open3D TSDF
                                           map별 mesh / cloud

후속 수동 작업: 선택 map + 검증한 중력 정렬 → 지형 층 → 로버별 costmap
향후 연결: 고정 기준 mesh/STL + 별도 주행 B의 관측 → 누적오차 보정
```

- pose 추정은 **ORB-SLAM3의 non-inertial `STEREO`**다. 우리 쪽 실행기·저장·보정 검증·종료 처리·시각화 계층을 추가한 구성이다.
- 현재 native runner는 좌우 IR만 `TrackStereo`에 넣는다. **D435 depth는 TSDF와 촬영 안내에 사용한다.** 현재 `IMU_STEREO` 융합은 없다.
- D435 depth는 장치의 stereo depth 출력이다. ROS Z16 원본을 uint16 PNG로 보존하며 현재 wrapper 출력 scale은 **0.001 m**다. sensor 자체 `depth_units`와 ROS 출력 단위를 혼용하지 않는다.
- TSDF는 최종 keyframe과 **2 ms 이내** 대응 depth만 융합한다. 기본 voxel **0.01 m**, 유효 depth **0.25~3 m**. voxel 1 cm는 실측 정확도 1 cm를 뜻하지 않는다.
- pose는 `T_world_left`, 보정은 `T_left_depth`. `T_world_depth = T_world_left @ T_left_depth`이고 Open3D에는 역행렬을 전달한다.
- 현재 RGB는 기록하지 않는다. 최종 PLY는 형상 확인용이며 RGB texture map이 아니다.
- ORB-LINE은 별도 fork의 point+line 비교 모드다. 선을 화면에 그리는 LSD overlay만으로 pose 추정이 선 기반이 되는 것은 아니다.
- COLMAP은 이미지에서 camera pose/구조를 구하는 SfM과 dense MVS를 제공한다. 우리 작업에서는 촬영 후 비교 후보이며 pose의 scale·좌표·카메라 대응을 검증해야 D435 depth와 결합할 수 있다. [공식 tutorial](https://colmap.github.io/tutorial.html).

## 3. 실행 환경과 필요한 파일

### 노트북

이 작업은 Conda의 `isaacsim`/`robot_lab` 대신 **`.venv-mapping`**을 사용한다. 사용자 site-package가 섞이지 않도록 wrapper를 우선 사용한다.

| 항목 | 현재 확인한 값 |
|---|---|
| OS | Ubuntu 22.04.5 LTS |
| Python | 3.10.12, `/path/to/rover-mapping/.venv-mapping/bin/python` |
| GUI / 영상 | PyQt5 5.15.6 / OpenCV 4.5.4 |
| 수치 / 3D | numpy 1.26.4 / scipy 1.13.1 / Open3D 0.19.0 |
| 추가 도구 | pygame 2.6.1 / matplotlib 3.5.1 |
| GUI wrapper 환경 | `PYTHONNOUSERSITE=1`, `OMP_NUM_THREADS=2`, `OPENBLAS_NUM_THREADS=1` |
| ROS | 로컬 ROS 작업은 Humble. 현재 Qt 매핑 앱은 로컬 ROS source 없이 실행 가능 |

필수 native 자산:

| 경로 | 역할 |
|---|---|
| `/path/to/rover-mapping/outputs/handheld_mapping/orb3_comparison_build/rover_orb3_compare` | live/offline 점 ORB 실행기 |
| `/path/to/rover-mapping/outputs/handheld_mapping/orb3_comparison_build/runtime/` | point ORB·DBoW2·g2o 공유 라이브러리 |
| `/path/to/rover-mapping/outputs/handheld_mapping/native_dependencies/pangolin/` | native 의존성 |
| `/path/to/rover-mapping/external/ORB-LINE-SLAM/Vocabulary/` | ORB/LSD vocabulary. 현재 point 경로도 이 위치를 사용 |
| `/path/to/rover-mapping/outputs/handheld_mapping/orbline_build/rover_orbline` | 선택적 점선 비교 실행기 |

venv 재생성의 기본 명령은 아래와 같다. `requirements.txt`에는 numpy/scipy/Open3D만 명시돼 있으므로 **시스템 PyQt5/OpenCV와 pygame/matplotlib도 별도로 준비**해야 한다. 이미 정상인 현재 환경을 문서 확인 목적으로 재설치할 필요는 없다.

```bash
cd /path/to/rover-mapping
PYTHONNOUSERSITE=1 /usr/bin/python3 -m venv --system-site-packages .venv-mapping
PYTHONNOUSERSITE=1 .venv-mapping/bin/python -m pip install -r experiments/handheld_mapping/requirements.txt
```

native runner를 다시 빌드할 때:

```bash
bash tools/build_mapping_baseline.sh /absolute/path/to/BUILT_ORB_SLAM3
bash tools/build_orbline.sh
```

첫 명령은 이미 빌드된 ORB-SLAM3 소스·공유 라이브러리를 요구하고, 두 번째도 external fork·의존성이 준비돼 있어야 한다. 현재 point core의 빌드 소스는 `/tmp/rover-native-mapping-deps/ORB_SLAM3-4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`에 남아 있다. **`/tmp`를 영구 백업으로 간주하지 않는다.** 이전할 때 소스·patch·빌드 옵션도 따로 보존한다. `-march=native` 자산은 다른 CPU에서 다시 빌드해야 할 수 있다.

### Raspberry Pi

| 항목 | 마지막 확인한 구성 |
|---|---|
| OS / ROS / Python | Ubuntu 24.04 / native ROS 2 Jazzy / `/usr/bin/python3` 3.12 |
| SSH | `pi@raspberrypi.local`. hostname이 안 되면 현재 Pi IP로 교체 |
| Camera | D435 serial `027322073612`, firmware 5.13.0.50 |
| Camera wrapper | RealSense ROS 4.58.4 / librealsense 2.58.4 |
| ROS Domain | 67, Fast DDS local profile |
| runtime | `/home/pi/handheld_mapping/runtime` |
| Pi ROS adapter | `/home/pi/handheld_mapping/ros2_ws/install` |
| DDS profile | `/home/pi/handheld_mapping/live/fastdds_stereo_local.xml` |
| 조종 checkout | `/home/pi/ksrc`, 확인 당시 commit `4bd158b` |

Pi에 노트북 Humble 설정을 source하지 않는다. 매핑 시작 때 Python worker/helper를 `/dev/shm/rover_mapping_ID/code`에 자동 배포하지만 **ROS·RealSense driver·native runtime을 새로 설치해주지는 않는다.** 경로가 바뀌면 `controller.py`와 `pi_worker.py`의 실제 설치 경로를 맞춘다.

2026-10-14 디버깅 마지막 확인에서는 `raspberrypi.local` 조회에 실패했다. 과거 hotspot IP를 현재 주소로 가정하지 않는다. USB2 촬영 기록은 있으며 USB3 케이블/포트의 실제 연결 속도는 다음 실기에서 재확인한다. PD 충전이라는 정보만으로 카메라 전원 안정성을 판정하지 않는다.

## 4. 빠른 시작: 촬영 → 지도 보기

### 연결 확인과 앱 실행

```bash
cd /path/to/rover-mapping
ssh -o BatchMode=yes -o ConnectTimeout=8 pi@raspberrypi.local hostname
bash tools/handheld_mapping.sh
```

SSH 명령이 실패하면 네트워크·현재 IP·인증을 먼저 확인한다. 앱의 **Pi 연결**에도 같은 대상을 입력한다. D435는 Pi USB에 연결한다. 이전 camera/ORB가 같은 토픽으로 실행 중이면 먼저 해당 실행을 정상 종료한다. 새 앱은 다른 실행을 임의 종료하지 않는다.

장비 없이 화면·저장·TSDF 흐름만 확인:

```bash
bash tools/handheld_mapping.sh --demo
```

합성 demo는 알려진 pose를 사용한다. 실제 ORB 정확도·Pi 성능 시험으로 집계하지 않는다.

### 기본 옵션과 조작 순서

| 옵션 / 버튼 | 사용 |
|---|---|
| Pi는 센서 전송만 · 종료 후 노트북에서 ORB 계산 | 기본 권장. native point 실행기가 있으면 기본 ON |
| 실시간 촬영 범위 표시 · 노트북에서 ORB 동시 실행 | 기본 ON. 촬영 중 잠정 추적·coverage 안내 |
| ORB-SLAM3 + TSDF | 기본 처리 방식 |
| ORB + TSDF → 점+선 비교 | 촬영 후 추가 offline 비교. 초기 실기 기본 선택은 아님 |
| 매핑 시작 | 새 RUN 생성, Pi 준비·수집·노트북 원본 저장 |
| 종료 및 지도 생성 | 수집 마감 → 최종 전송 → 전체 원본 ORB → TSDF |
| 저장 폴더 열기 | 현재 RUN 열기 |
| 기록에서 다시 생성 | 기존 최종 keyframe/depth로 TSDF 재생성. ORB를 다시 계산하는 버튼은 아님 |
| 3D 지도 보기 | 현재 RUN 또는 선택한 RUN의 저장된 PLY 열기 |

1. 처음에는 정지 상태로 시작하고 오른쪽 live 추적 정상 표시를 기다린다. vocabulary 초기화 동안 live 입력 일부는 건너뛸 수 있다.
2. 작은 구역을 천천히 이동하며 같은 표면을 다른 위치에서 겹쳐 촬영하고 시작점을 다시 본다. 화면이 초록이 된 것만으로 촬영 품질을 통과시키지 않는다.
3. 로버 촬영이면 **조이스틱에서 먼저 정지**한다. 매핑 종료 버튼은 모터 정지 버튼이 아니다.
4. **종료 및 지도 생성**을 누르고 `complete`/`partial`/`failed`가 될 때까지 기다린다. 저장 중 Pi 전원을 끄지 않는다.
5. `session.json`과 dense summary를 확인한 뒤 3D 지도를 연다. map이 여러 개면 독립 결과로 검사한다.

앱을 열기만 해서는 촬영·Pi 접속·모터 조종이 시작되지 않는다. 이미 열려 있던 이전 코드의 앱은 촬영·저장 완료 후 다시 열어야 새 로컬 코드가 적용된다.

### 화면 해석

- 왼쪽 위: 원본 IR. 왼쪽 아래: 같은 프레임의 관측색 오버레이. 오른쪽: 잠정 coverage·camera 경로·시야 방향.
- 회색: 미관측. 노랑: 1~2회 관측. 초록: 3회 이상 시점 변화 관측 참고값. 정지 프레임을 계속 세지 않으며 셀별 마지막 관측에서 투영 위치 10 cm 또는 방향 10° 이상 바뀌어야 횟수를 늘린다.
- 색은 유효 depth 표면을 투영한 것이다. 미관측 공간을 부채꼴로 채우지 않는다. 벽도 관측 표면에 포함된다.
- coverage는 **초기 optical world의 X-Z 투영**이며 물리적 수평 지도나 costmap이 아니다. 높이가 다른 표면이 같은 셀에 합쳐질 수 있다. map이 바뀌면 이전 격자를 저장하고 새로 시작한다.
- 카메라 요청은 15 FPS, UI/오버레이 갱신은 최대 5 Hz다. live가 밀리면 최신 입력 위주로 일부 pair를 건너뛰며, 이 수는 Pi 전송 drop과 별개다. raw는 별도 경로로 보존한다.
- live 오류가 나도 raw 기록과 종료 후 재계산을 이어간다. 추적 손실·유효 depth 없음·오래된 결과에서는 색을 제거한다.
- UI의 수신 후 ms는 노트북에 stereo가 저장된 뒤 pose 수신까지다. 센서→화면 전체 지연이 아니다.

## 5. 조이스틱과 SPI 배선

매핑 앱과 조종 앱은 별개다. 기존 경로는 **노트북 pygame 화면 조이스틱 → UDP 5005 → Pi bridge → SPI0.0, 1 MHz, 50 Hz → STM32F446RE**다. 조이스틱 monitor의 telemetry 요약은 약 5 Hz이고 전체 IMU 원본 기록이 아니다.

현재 firmware 기준 배선:

| 신호 | Pi GPIO / 물리 핀 | STM |
|---|---|---|
| SCLK | GPIO11 / 23 | PB13, SPI2 SCK |
| MOSI, Pi→STM | GPIO10 / 19 | PC3, SPI2 MOSI |
| MISO, STM→Pi | GPIO9 / 21 | PC2, SPI2 MISO |
| CE0 / NSS | GPIO8 / 24 | PB12, SPI2 NSS |
| 공통 GND | GND / 20 또는 25 | GND |

현재 firmware의 PB14/PB15는 모터 TIM12에 사용되므로 일반 SPI2 예제와 혼동하지 않는다. 배선 수정 후 유효 SPI 응답과 사용자 조작 응답을 확인했다. 초기 all-zero 응답 기록은 배선 수정 전 상태다.

**SPI owner는 하나만 실행한다.** bridge 실행 중 별도 `SpiLink` monitor/poll을 동시에 띄우지 않는다. 제어 코드를 변경하면 fake/시뮬레이션 검증을 먼저 하고, 실제 기체는 바퀴를 띄워 방향·정지부터 확인하는 것을 권고한다.

Pi의 별도 터미널에서 기존 bridge를 실행하는 저속 시작 예시:

```bash
cd /home/pi/ksrc
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -u pi/common/udp_spi_bridge.py \
  --hz 50 --max-lin 0.05 --max-ang 0.3
```

노트북에서 화면 조이스틱:

```bash
cd /path/to/rover-mapping
PYTHONNOUSERSITE=1 .venv-mapping/bin/python \
  teleop/ksrc_snapshot/pi/teleop/teleop_joystick.py \
  --udp raspberrypi.local --rate 50 --max-lin 0.05 --max-ang 0.3 --monitor
```

- 왼쪽 패드 드래그: 병진 `vx/vy`. 오른쪽 패드: 회전. Q/E를 누르며 왼쪽 패드를 끌면 병진·회전을 함께 줄 수 있다.
- Space를 누르는 동안 `--max-lin` 기준 선택한 1/2/3배 속도로 직진하며 횡이동·회전은 0이다. 숫자 1/2/3 또는 화면 버튼으로 배율을 고르며 시작은 1배다. Space를 떼거나 창 포커스를 잃으면 전체 0, Esc는 정지 후 종료다. 포커스 복귀 후에는 Space를 새로 눌러야 직진한다. 마우스를 놓으면 해당 패드는 0이다. 실제 기체의 정지 응답도 확인한다.
- 병진 제한은 m/s, 회전 제한은 rad/s. 속도를 바꿀 때는 **bridge와 조이스틱 양쪽 제한**을 확인한다. 이후 로그에는 사용자 조작의 병진 0.15 m/s 명령도 있으나 이를 모든 조건의 검증된 주행 한계로 간주하지 않는다.
- 입력 timeout의 정지 경로가 있고 과거 zero-only 점검을 했지만, 매 시험 연결 상태와 watchdog/조향 gate를 재확인한다. 명령이 보이는 것만으로 실제 적용을 판정하지 않는다.
- 매핑 촬영만 할 때는 이 bridge/조이스틱을 실행할 필요 없다. 이 문서 작성 중에는 실제 조종 프로세스나 모터 명령을 실행하지 않았다.

## 6. 저장 구조와 상태 확인

기본 저장 루트: `/path/to/rover-mapping/outputs/handheld_mapping/sessions`

```text
RUN = YYYYMMDD_HHMMSS_ID/
  session.json                    세션 상태·frame 수·drop·live/노트북/Pi 진단
  capture_calibration.json        이번 camera profile·CameraInfo·TF·depth scale
  frames.jsonl                    source stamp_ns·stream·상대 PNG 경로·단계 계측
  events.jsonl                    status/finished/pose 등 packet metadata
  raw/{left,right,depth}/*.png     원본 uint8 IR / uint16 depth
  worker.log                     SSH/Pi worker 오류
  orb/                           Pi artifact·camera.log; Pi ORB 선택 시 지도 포함
  orb_live/                      잠정 live pose·coverage_map_ID.npz·로그
  orb_offline/                   종료 후 full raw point 결과·Atlas·최종 keyframe
  dense_offline/                 기본 sensors-only 모드의 최종 TSDF mesh/cloud
  dense/                         Pi ORB 모드 또는 합성 demo의 TSDF
  orbline/                       선택적 offline point+line 결과·comparison.json
  dense_orbline/                  선택적 point+line pose의 TSDF
```

| 파일 / 필드 | 무엇을 볼지 |
|---|---|
| `session.json.state` | `complete`: 현재 절차의 저장 완료, `partial`: drop/미융합 등 일부 누락, `failed`: 수집/계산 오류. 정확도 합격 판정은 아님 |
| `session.json.counts` | 실제 노트북에 저장한 stream별 frame 수 |
| `session.json.dropped` | worker 전송 큐 거절 누적. source/USB에서 이미 사라진 모든 frame 수는 아님 |
| `session.json.live.dropped_pairs` | 촬영 안내에서 건너뛴 pair. raw 전송 drop과 다름 |
| `session.json.transport_queue` | worker 최종 queue snapshot. 새 코드에서는 finished에서 갱신 |
| `orb_offline/runner_complete.json` | native 실행의 정상 종료·저장 marker |
| `orb_offline/{summary.json,timings.csv}` | 처리한 pair·state·map ID·tracking 호출 시간 |
| `orb_offline/map_ID_keyframes.csv` | 최종 keyframe pose. 중간 live pose와 다를 수 있음 |
| `dense_offline/summary.json` | map별 keyframe 수·융합 수·누락·PLY 경로 |
| `orb/camera.log`, `worker.log`, `orb_offline/runner.log` | 장치/전송/native 오류 |

map ID `0/421`은 두 맵의 이름이지 422개 맵이라는 뜻이 아니다. ID는 초기 keyframe ID를 사용한다. 분리된 맵은 독립 좌표계이므로 같은 원점으로 임의 합치지 않는다. Atlas 파일이 저장된 것과 별도 주행 재지역화 또는 STL 보정이 되는 것도 다른 검증이다.

## 7. 저장된 3D 지도 보기와 다시 계산

GUI의 **3D 지도 보기** 또는 아래 CLI를 사용한다. 웹 브라우저는 필요 없다.

```bash
cd /path/to/rover-mapping
# 기본 sessions 아래 가장 최근 RUN
bash tools/view_mapping.sh

# 특정 실제 RUN, map 하나, point cloud
bash tools/view_mapping.sh outputs/handheld_mapping/sessions/20261013_204443_766235
bash tools/view_mapping.sh outputs/handheld_mapping/sessions/20261013_204443_766235 --map-id 0
bash tools/view_mapping.sh outputs/handheld_mapping/sessions/20261013_204443_766235 --cloud

# 다른 backend 또는 직접 PLY
bash tools/view_mapping.sh /absolute/path/to/RUN --backend orbline
bash tools/view_mapping.sh /absolute/path/to/map_0_mesh.ply
```

auto는 `dense_offline` → `dense` → `dense_orbline` 순서로 summary를 찾는다. 기본 sessions 밖에 저장한 RUN은 경로를 명시한다. 최신 RUN에 아직 결과가 없으면 오류를 안내하며 이전 지도로 조용히 돌아가지 않는다. 직접 PLY를 지정할 때는 `--backend/--map-id`를 함께 주지 않는다.

마우스: 왼쪽 드래그 회전, 휠 확대·축소, Shift+왼쪽 드래그 화면 이동. 두 창이 뜨면 map ID를 확인한다. wrapper는 DISPLAY가 있을 때 X11을 사용해 이전 Wayland/GLEW 문제를 피한다.

### ORB만 새로 계산

```bash
cd /path/to/rover-mapping
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv-mapping/bin/python -m experiments.handheld_mapping.offline /absolute/path/to/RUN
```

이 명령은 **ORB 결과만** 만들며 `orb_offline`이 이미 있으면 덮어쓰지 않고 실패한다. 실패 결과도 먼저 보존하고 별도 사본에서 재시도한다. native 최대속도 재생은 영상 timestamp를 유지하지만 wall-clock을 15 FPS로 pace하지 않는다. 동일 입력도 비동기 mapping 때문에 결과가 달라질 수 있다.

### 최종 pose로 TSDF만 재생성

가장 간단한 방법은 앱의 **기록에서 다시 생성**이다. 기본 sensors-only 결과를 Python에서 명시할 때는 아래처럼 실행한다.

```bash
cd /path/to/rover-mapping
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv-mapping/bin/python - /absolute/path/to/RUN <<'PY'
from pathlib import Path
import sys
from experiments.handheld_mapping.reconstruct import reconstruct
print(reconstruct(Path(sys.argv[1]), backend="orb_offline"))
PY
```

`backend="orb"`는 Pi ORB, `"orbline"`은 점선 결과다. `reconstruct`의 CLI는 backend 인자가 없고 기본 `orb`이므로 sensors-only RUN에는 위 API나 GUI를 사용한다. **TSDF 재생성은 해당 dense 산출물을 갱신한다.** 중요한 기준 맵은 RUN 전체를 복사해 보존하고 재생성한다. raw PNG와 source index는 변경하지 않는다.

### 점선 비교

```bash
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv-mapping/bin/python -m experiments.handheld_mapping.compare /absolute/path/to/RUN
```

ORB-LINE 실행기가 필요하다. `orbline`이 이미 있으면 덮어쓰지 않는다. 기존 point baseline을 재사용할 때 completion/calibration/pairs 일치를 검사한다. 점 core O3/native와 line core O1/native 차이가 있어 처리시간만으로 알고리즘 우열을 판정하지 않는다. 노트북·서버의 짧은 sim 재생 성공은 Pi 실시간 점선 또는 실측 정확도 개선을 뜻하지 않는다.

## 8. 지형 정보·costmap 사용법

현재는 **offline 기록 재생 도구**다. 촬영 화면의 오른쪽 coverage와 별개이며, Qt의 실시간 costmap 갱신이나 Nav2/경로 제어 연결은 아직 없다.

장비 없이 합성 결과 생성:

```bash
cd /path/to/rover-mapping
bash tools/terrain_mapping.sh --demo --output /absolute/path/to/NEW_TERRAIN_DEMO
```

실제 RUN에서 선택 map의 지형 생성:

```bash
bash tools/terrain_mapping.sh /absolute/path/to/RUN \
  --backend orb_offline --map-id 0 \
  --alignment /absolute/path/to/grid_from_world.json \
  --bounds -5 5 -5 5 --resolution 0.05 --stride 4 \
  --output /absolute/path/to/NEW_TERRAIN_OUTPUT
```

- `--alignment`는 ORB world를 **z-up 중력 좌표**로 옮기는 검증된 4×4 rigid transform의 JSON 배열이다. 실제 자세/원점을 확인해야 하며 임의 identity나 다른 촬영의 변환을 넣지 않는다. 현재 자동 중력 검출은 없다.
- `--backend`와 `--map-id`는 해당 최종 keyframe CSV에 맞춘다. 모든 map을 한 지형 격자에 무조건 합치지 않는다.
- output은 새 폴더여야 한다. 기본 범위 10×10 m, resolution 0.05 m, `max_cells=250000`, `max_points=100000`. 범위 밖 관측은 집계 후 버리며 자동 확장하지 않는다.
- `--rover-limits FILE.json`으로 아래 값들을 넘길 수 있다. **현재 수치는 prototype 시험값이며 실제 로버 한계로 확정하지 않았다.**

```json
{
  "max_slope_deg": 20.0,
  "max_step_m": 0.06,
  "max_roughness_m": 0.025,
  "safety_radius_m": 0.15
}
```

산출물은 `terrain.npz`, `summary.json`, `preview.png`다. NPZ는 `np.load(path, allow_pickle=False)`로 읽는다. 배열 축은 `(y, x)`이고 높이/거칠기/단차는 m, 경사/방향은 degree다.

| 주요 층 | 의미 |
|---|---|
| `height_m`, `variance_m2` | 셀의 프레임별 평균 높이 누적값과 표본 분산 |
| `slope_deg`, `heading_deg`, `normal_x/y/z` | 이웃 높이 차 기반 경사·방향·법선 |
| `roughness_m`, `step_m` | 3×3 국소 경사 평면 RMS 잔차, 이웃 최대 높이 차 |
| `vertical_span_m` | 셀 내 높이 범위. 여러 표면을 보수적으로 차단하는 데 사용 |
| `observations`, `last_stamp_ns`, `observed`, `valid` | point 수와 구분한 update 관측 통계·유효성 |
| `cost` | 유효 0..252, blocked 254, unknown 255 |

unknown을 평지로 채우지 않는다. 기본 cost는 경사/거칠기/단차의 정규화 가중합이며 에너지 비용이나 보정된 위험 확률이 아니다. 현재 지면 분할·다층 지형·전체 차체 footprint·토질/슬립 추정까지 검증한 것은 아니다. sensor/pose 오차도 거칠기와 높이 범위에 섞일 수 있다.

## 9. 저장 공간과 메모리

| 위치 | 현재 정책과 범위 |
|---|---|
| Pi 전송 큐 | 압축 전 payload+header 기본 4 MiB, 최대 24 packet. 차면 새 packet 거절 |
| Pi 저장 | worker/code/log는 `/dev/shm`. raw 영상 파일은 Pi에 쌓지 않음 |
| Pi RAM 여유 검사 | `/dev/shm` free 32 MiB 미만이면 실패 종료 |
| 노트북 disk 검사 | 시작 전 2 GB, 실행 중 256 MB 하한 |
| live mailbox | 최신 stereo 2쌍, 좌우 미매칭 각 8 stamp, depth metadata 32개 |
| coverage 배열 | 20×20 m / 5 cm, 지속 배열 2,240,000 B |
| terrain 통계 배열 | 10×10 m / 5 cm, 지속 배열 1,120,000 B |

4 MiB는 Pi 전체 RSS 한계가 아니다. ROS 버퍼·현재 frame·Python overhead·RAM log가 별도로 있고, 노트북 ORB 내부 map과 TSDF도 지도 크기에 따라 성장한다. 센서 전송 모드는 Pi Atlas 성장을 피한다. 격자 통계는 프레임/점군 전체를 메모리에 누적하지 않으며 인덱스 처리는 순차 스캔/임시 SQLite를 사용한다.

640×480 IR 2장+depth의 압축 전 영상은 15 FPS에서 약 **18.4 MB/s, 1.1 GB/min**이다. zlib/PNG의 실제 크기는 장면에 따라 달라진다. Wi-Fi·USB·CPU·disk 부하를 따로 측정한다. `/dev/shm`의 이전 촬영 폴더는 자동 삭제하지 않으므로 노트북 저장과 소유 실행 종료를 확인한 뒤 해당 이전 RUN만 정리한다. 실행 중 wildcard 삭제는 하지 않는다.

## 10. 입력 공백 디버깅과 문제 대응

최근 긴 실제 기록에서 left **0.93243초**, depth **2.99708초** 공백이 있었고 left 공백 직후 tracking이 손실됐다. 두 실제 기록의 추정 내부 영상 누락은 각각 queue drop **101개/13개와 일치**했다. 송신 차단 실험에서도 큐 포화와 stream별 비대칭 누락을 재현했다.

**큐를 처음 막은 압축·SSH/무선·노트북 저장 중 어느 단계인지는 미확정**이다. USB2 경고가 있다는 이유만으로 모든 누락을 USB 탓으로 확정하지 않는다. 특징 수 1200→2400 실험은 주 맵 분리를 없애지 못했고 한 번은 저장 중 SIGSEGV가 발생했다. 기본 설정은 유지한다.

새 촬영의 진단 위치:

| 기록 | 필드 / 해석 |
|---|---|
| `session.json.capture_metrics.streams` | 녹화 중 received/enqueued/dropped, 첫·마지막 source stamp, source/receipt 최대 gap |
| `frames.jsonl`의 Pi 필드 | `stream_seq`, `pi_receipt_monotonic_ns`, `pi_enqueue_monotonic_ns`, `pi_queue_wait_ms`, `pi_compress_ms`, `pi_send_start_monotonic_ns` |
| `capture_metrics.sender` | queued sender의 sent 수, compression/write last/max, 송신 진행 age |
| `frames.jsonl`의 노트북 필드 | `laptop_receipt_monotonic_ns`, `laptop_read_wait_ms`, `laptop_decode_ms`, `laptop_png_write_ms`, `laptop_image_saved_monotonic_ns` |
| `session.json.laptop_capture` | 완료 packet의 read/accept 및 stream별 저장 집계, 최대 처리 packet |

Pi/laptop monotonic은 다른 clock이므로 절대값을 서로 빼지 않는다. read wait는 pacing·전송 대기를 포함하며 latency가 아니다. legacy `source_frames`에는 시작 준비 callback도 포함된다. 저주파 status는 snapshot이고 개별 drop stamp/reason 전체 로그는 아니다. 실패한 PNG write나 미완료 read 시간은 마지막 성공 packet 집계에서 빠질 수 있다.

최종 queue/source/drop/capture_metrics는 finished 수신 때 즉시 반영되도록 수정했다. TSDF 후처리가 실패해도 최종 drop 수가 이전 status로 돌아가지 않는다. 과거 RUN에는 새 계측 필드가 없으며 현재 코드로 과거 누락 위치를 복구할 수 없다.

| 증상 | 먼저 확인할 내용 |
|---|---|
| `raspberrypi.local` 조회 실패 | 같은 네트워크·현재 IP·SSH 인증. 앱 host를 현재 주소로 수정 |
| 기존 camera/ORB 실행 오류 | 해당 소유 실행 정상 종료. 임의 `killall`로 다른 작업까지 종료하지 않음 |
| D435가 목록에서 사라짐 / `-71` | 양쪽 접촉·케이블·포트·실제 USB 속도·전원 및 camera/kernel 로그. 의도적 분리와 구분 |
| 움직일 때 끊김 / queue drop 증가 | source/receipt gap → queue wait/compress/write → 노트북 저장 순으로 같은 구간 비교 |
| 특징점은 많은데 추적 손실 | 검출 수와 map-point inlier는 다름. 공백·blur·반복/반사 표면·시야 변화를 확인 |
| 중력 미정렬 / 지도 기울어짐 | 현재 IMU 융합·자동 중력 정렬 없음. 실제 alignment 검증 필요 |
| 지도 창이 두 개 | 분리된 map ID와 summary 확인. 독립 좌표계를 임의 병합하지 않음 |
| 최신 지도 없음 | 촬영/후처리 종료 상태, RUN/backend, dense summary, runner log 확인 |
| Open3D 창 초기화 실패 | `tools/view_mapping.sh`, 로컬 DISPLAY/X11 환경 확인 |
| `FileExistsError`로 ORB/terrain 재실행 실패 | 기존 결과 보호 동작. 원본을 보존한 새 사본/새 output 사용 |
| 조이스틱은 보내는데 STM 적용 없음 | UDP 도달·유효 SPI 응답·CRC·watchdog/조향 gate 확인. 별도 SPI owner 동시 실행 금지 |

## 11. 대표 기록과 검증 범위

| RUN / 자료 | 확인한 내용 |
|---|---|
| `sessions/20261013_201134_fd506c` | 카메라 오류로 capture는 failed. 보존된 939 stereo 쌍 offline 추적·18 keyframe TSDF 재구성 |
| `sessions/20261013_203820_847029` | IR 1695쌍, depth 1691장, queue drop 13, map 0. ORB tracking 100% |
| `sessions/20261013_204443_766235` | IR 3604쌍, depth 3584장, queue drop 101, tracking 98.75%, map 0/421 분리 |
| `20261013_server_validation` | Threadripper 7960X에서 sim stereo 180쌍 point/line 정상 종료·저장. GT 정확도 비교 전 |
| `20261014_multia_debug/laptop_replay` | 실제 저장 영상 20초를 15 FPS로 재생, Qt/raw/live/final ORB/TSDF 확인 |

위 경로의 기준은 `/path/to/rover-mapping/outputs/handheld_mapping`이다. 재생은 현재 Pi/USB/무선의 실시간 시험이 아니다. 재생 worker drop=0도 과거 원본에 누락이 없었다는 뜻은 아니다. 추적 성공률·점 수·voxel 크기를 지도 정확도로 해석하지 않는다.

2026-10-14 최종 코드 검증: 관련 mapping/headless mapping **173 passed (10.11초)**, compileall/pyflakes/bash 문법 검사, 독립 review의 중요 지적 해소. 동일 입력 ORB 반복에서도 결과가 변했으므로 단회 A/B를 성능 개선으로 단정하지 않는다. 실제 계측 적용 Pi 촬영·실측 정확도·전체 지연은 남아 있다.

관련 회귀 실행:

```bash
cd /path/to/rover-mapping
QT_QPA_PLATFORM=offscreen PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv-mapping/bin/python -m pytest tests/handheld_mapping tests/headless_mapping -q
```

repository 전체 `pytest -q --maxfail=1`은 과거 output snapshot의 `pi/common/test_spi_link.py` 수집 시 `sim/libksrc_common.so` 부재로 중단됐다. 전체 repository 통과로 보고하지 않는다.

인수인계서 작성 때 CLI `--help`, shell 예제 13개의 문법, 로컬 링크 11개, 실제 RUN의 map 선택을 확인했다. `tools/terrain_mapping.sh --demo`도 새 폴더에서 실행해 `terrain.npz`·`summary.json`·`preview.png` 생성을 확인했다. 이 문서 확인을 위해 실제 Pi 촬영이나 조이스틱 주행을 실행하지 않았다.

## 12. 다음 담당자의 우선순위와 이전할 자료

1. **입력 연속성:** Pi 재연결 후 새 앱으로 정지·느린 회전·이동을 구분해 촬영하고 단계별 계측으로 최초 병목을 찾는다. 해당 단계만 수정하고 같은 조건에서 재검증한다.
2. **IMU/encoder 검증:** Qt의 같은 RUN 동시 기록은 연결했다. 실제 acquisition 시각·주기·clock·축·장착 extrinsic·bias/noise를 확인하고 영상과 맞춘다. 50 Hz SPI telemetry를 센서 원시 acquisition rate로 간주하지 않는다. 원본 telemetry의 encoder 필드는 보존하지만 encoder 정합·융합은 검증 전이다.
3. **지도 품질:** 알려진 거리·단차·시작점 재방문·중복 벽으로 검사한다. 영상단독/검증 후 관성융합/CLAHE/ORB-LINE/COLMAP은 같은 기록과 조건으로 비교한다.
4. **중력 정렬·지형:** 실제 z-up 정렬·지면 구분을 정하고 높이·경사·단차와 실측을 대조한다. 실제 로버 조건으로 cost 한계를 보정한다.
5. **고정 기준 맵 보정:** 선택한 촬영 A의 mesh를 고정하고 별도 B를 정합한다. 스캔 mesh와 경기 STL의 단위·축·원점·불일치를 공통 처리한다. ORB Atlas 재지역화만으로 STL 보정 검증을 대신하지 않는다.
6. **효과·운용 평가:** 같은 B에서 보정 OFF/ON 오차와 실패를 비교한 뒤 live 보정/costmap·통신 지연·경기 STL 검증으로 진행한다.

다른 담당자/PC에 넘길 때 함께 보존할 것:

- 이 문서, `/path/to/rover-mapping/experiments/handheld_mapping`, `/path/to/rover-mapping/tools`의 해당 wrapper/build script, `/path/to/rover-mapping/ros2_ws/src/orbslam3_ros2/scripts/prepare_mapping_run.py`와 관련 headless 자료.
- native 소스/patch/빌드 옵션·runtime·Pangolin·vocabulary·external fork. `outputs/`에 실제 runtime과 조이스틱 snapshot이 있으므로 소스 코드만 복사하면 실행 자산이 빠진다.
- 선택한 **RUN 전체**: raw/보정/index/events/최종 keyframe/Atlas/dense summary와 PLY/log. mesh만으로 원본 재융합이나 공백 분석을 할 수 없다.
- Pi `/home/pi/ksrc`의 실제 firmware/bridge 버전과 배선·카메라 장착 정보. 노트북 snapshot은 특정 시점 사본이므로 Pi 최신 checkout과 대조한다.
- Python/system dependency 버전, 실측·촬영 구간 메모. SSH private key 등 인증 비밀은 공유 문서/결과 묶음에 넣지 않는다.

`frames.jsonl`의 PNG 경로는 RUN 상대경로지만 native `pairs.tsv`에는 절대경로가 들어간다. PC/경로 이동 후 기존 pair index를 그대로 실행하지 말고 새 위치에 맞춰 재생성한다. 기존 point/line 완료 결과는 보존하며 잘못된 input hash를 성공으로 우회하지 않는다. `/dev/shm`과 `/tmp`만으로 자료를 인계하지 않는다.

원본 workspace는 Git 저장소가 아니었고 이번 공유 체크아웃은 별도 Git 저장소다. 원본에 commit이 있었다고 가정하지 않는다. external/Pi 저장소의 commit과 workspace 파일 사본·검증 결과를 구분해 보존한다.

## 13. 코드 위치와 상세 근거

| 코드 | 역할 |
|---|---|
| `app.py`, `controller.py` | Qt 화면·세션·SSH 배포·정상 종료·후처리 |
| `pi_worker.py`, `transport.py`, `protocol.py` | Pi 수집/계측·bounded FIFO·binary framing |
| `storage.py`, `recording.py` | 원본 PNG/metadata·disk index/시각 대응 |
| `offline.py`, `orbline/runner.cpp` | native point 재생·pose/최종 keyframe export |
| `live.py`, `coverage.py`, `overlap.py`, `live_preview.py` | 최신 pair 추적·잠정 coverage·IR 재투영·화면 |
| `reconstruct.py`, `viewer.py` | 최종 TSDF·저장된 3D 지도 보기 |
| `compare.py` | point+line offline 비교 |
| `terrain.py`, `costmap.py`, `terrain_replay.py`, `terrain_preview.py` | 고정 격자 통계·cost·offline 재생·PNG |

위 코드 기준 폴더는 `/path/to/rover-mapping/experiments/handheld_mapping`이다.

- [단계 로드맵](../experiments/2026-10-13-reference-map-roadmap.md)
- [현재 단계 요약](../experiments/2026-10-14-mapping-status.md)
- [화면·3D 뷰어 안내](../experiments/mapping-viewer.md)
- [지형/costmap 상세](../experiments/laptop-terrain-preparation.md)
- [입력 공백 조사·계측 보고](../experiments/2026-10-14-input-gap-investigation.md)
- [멀티에이전트 최종 검증](../evidence/20261014_multia_debug/final_verification.json)
- [실제 공백 정량 분석](../evidence/20261014_multia_debug/forensics/README.md)
- [ORB 반복·2400 특징 실험](../evidence/20261014_multia_debug/orb/README.md)
- [조종·배선·실기 기록](../evidence/20261013_rover_teleop_prepare/README.md)
- [기존 headless ROS/telemetry 경로](../experiments/headless-pi-manual-mapping.md)
- [실기 checklist](../experiments/first-real-mapping-checklist.md)
