# D435와 보정 IMU 동시 기록

2026-10-14: Qt 매핑 앱에 외부 ISM330DHCX의 원본·보정 telemetry 기록을 연결했고 실제 D435와 같은 RUN 저장·종료·ORB·TSDF 생성까지 확인했다.

## 현재 연결 범위

```text
노트북 조이스틱 → UDP5005 → Pi bridge(단일 SPI owner) → STM
                              └─ 비차단 UNIX datagram → 별도 SSH → 노트북 imu.jsonl
D435 → Pi 카메라 수집 → 기존 SSH → 노트북 raw IR/depth → ORB stereo → TSDF
```

IMU의 bias·offset·scale을 적용하고 실제 decoder의 축 정의가 보정 파일과 같은지 검사한다. decoder가 이미 body 축으로 바꾼 값에 X/Y 부호를 다시 반전하지 않는다. IMU는 영상 전송 queue를 공유하지 않으며 Pi 디스크에는 기록하지 않는다.

**2026-10-15 후속: [정지 IMU 기반 중력 정렬·지형 지도](2026-10-15-gravity-terrain-pipeline.md)를 추가했다.** GUI의 ORB는 계속 non-inertial stereo다. 영상과 IMU의 Pi receipt monotonic 시각은 정지 구간을 찾는 데 사용하며, MCU/D435 acquisition clock 동기화가 검증됐다는 뜻은 아니다. `imu_clock_verified=false`, `orb_fused=false`를 유지한다. 가속도계 단독 roll/pitch는 움직임 중 중력 외 가속도가 섞이는 진단값이다. 중력 기준 차체 기울기는 별도로 visual pose와 장착 모델로 계산한다.

**2026-10-16 후속:** [카메라–IMU 시간 정렬과 선택형 종료 후 융합](2026-10-16-imu-upstream-integration.md)을 도입했다. 기본 촬영은 그대로 stereo이고, 새 timing 형식과 품질 조건을 만족한 기록만 별도 관성 결과로 비교한다. 하드웨어 동기화 검증 플래그를 자동으로 true로 바꾸지 않는다.

## 매핑 전에 실행

1. 보정 JSON을 노트북에 별도 보관한다. 기본 경로는 `outputs/handheld_mapping/calibration/imu_calibration.json`이며 장비별 파일을 Git에 올리지 않는다.
2. 기존 bridge/IMU 확인 프로그램을 정상 종료한다. 다음 명령은 Pi KSRC 원본을 보존하고 `/dev/shm` 사본에 기존 telemetry tee patch를 적용한다. SPI가 사용 중이면 중복 실행하지 않고 거부한다.

```bash
bash tools/mapping_imu_bridge.sh pi@raspberrypi.local
```

이 터미널은 조종·촬영 중 열어둔다. 기존 노트북 조이스틱은 그대로 `--udp raspberrypi.local`로 연결한다. bridge 제한은 현재 병진 0.45 m/s, 회전 0.8 rad/s다. 조이스틱의 `--max-lin 0.15`를 기준으로 Space 직진 1/2/3배(0.15/0.30/0.45 m/s)를 허용하는 상한이며, 변경 후 bridge를 재시작해야 적용된다. 펌웨어나 모터 제어식은 수정하지 않는다.

3. 노트북에서 매핑 창을 연다.

```bash
bash tools/handheld_mapping.sh --imu-calibration /absolute/path/to/imu_calibration.json
```

4. **보정 IMU 함께 기록**을 켜고 실제 보정 경로를 확인한다. **매핑 시작**을 누르면 IMU의 실제 축 정의와 새 유효 샘플을 먼저 확인하고 카메라를 시작한다. IMU가 준비되지 않으면 오류를 표시하며 선택을 무시하고 영상만 촬영하지 않는다. 합성 demo에서는 IMU를 사용하지 않는다.
   **중력 기준 지도와 높이·경사·단차 생성**을 사용할 때는 촬영 시작 후 최소3초간 정지해 기준을 잡는다. 각 map에서 기준을 얻지 못하면 해당 map의 중력·지형 결과를 성공으로 표시하지 않는다.
5. 화면의 IMU 유효/고유 샘플 수, Hz, roll/pitch와 영상 증가를 함께 확인한다. Hz는 Pi 수신 monotonic의 고유 telemetry 기준이며 센서의 원시 208 Hz와 다르다. `imu_n`은 telemetry가 평균한 센서 샘플 수다.
6. 로버를 정지한 뒤 **종료 및 지도 생성**을 누른다. IMU를 마감한 후 ORB·TSDF 후처리를 수행한다. 종료는 joystick bridge를 끄지 않는다. 전원 재시작 후에는 RAM bridge를 다시 실행해야 한다.

## RUN에 저장되는 추가 파일

| 파일 | 내용 |
|---|---|
| `imu.jsonl` | 전체 원본 telemetry, 보정값, startup/duplicate/reset/jump 판정, MCU·Pi·노트북 시각의 구분 |
| `imu_calibration.json` | 이번에 쓴 보정 파일 원본 사본 |
| `robot.urdf` | 최신 20° URDF 사본 |
| `imu_manifest.json` | 보정·URDF SHA-256, 실제/기대 축 정의, 단위·시계·융합 상태 |
| `imu_summary.json` | 유효·고유·중복·초기 폐기·오류·이번 RUN tee 누락과 최종 상태 |
| `session.json`의 `imu` | 앱에서 사용한 최종 IMU 상태 |

첫 응답은 오래된 STM 송신 버퍼일 수 있어 raw로 보존하고 유효 초기화에서는 제외한다. 중복·MCU reset·clock jump를 구분하며 서로 다른 장치의 monotonic 시각을 직접 빼지 않는다. SSH 묶음 수신을 센서 Hz로 오인하지 않는다. listener가 없는 동안 누적된 bridge tee 누락은 새 RUN의 누락으로 세지 않는다.

IMU reader 오류는 촬영 실패로 전파하고 원본을 보존한다. 수신이 오래 멈추면 화면에 표시하고 해당 촬영을 partial로 분류한다. subscriber는 SPI를 열지 않고 자신이 만든 socket만 제거한다.

## 검증 결과와 남은 항목

- 현재 Pi bridge source에 기존 patch를 적용한 fake 검사: 명령·stale·종료 0 명령 동등성, receiver 부재/포화 등 11개 통과.
- 실제 IMU 단독 수집: 156 유효/155 고유, 약 49.92 Hz, 이번 RUN tee 누락 0.
- 실제 약 8초 영상 기록: left/right/depth 각 122개, IMU 958 유효/957 고유(카메라 준비·마감 시간 포함), 약 49.83 Hz, 영상/tee 누락 0. 같은 RUN 기록 및 정상 종료 확인.
- 같은 실제 입력의 offline ORB: 122 pairs, tracking OK 100%, map 1개. TSDF: keyframe 1개, 66,801 vertices/86,519 triangles. 정지 상태의 짧은 연결 시험으로 주행 정확도를 검증한 결과는 아니다.
- 매핑 범위 검사 205개 통과. 원본 작업 폴더의 bare pytest는 별도 시뮬레이션 의존성(`trimesh`, `mujoco`, `torch`, `libksrc_common.so`)과 기존 test basename 충돌 때문에 전체 통과를 주장하지 않는다.
- 다음 실기: 축별 기울이기·회전 확인, D435↔MCU acquisition 시간·누락·reset 검증, camera↔IMU 장착 변환/온도·bias/noise 확인 후 관성 융합을 연결한다. 6면 보정 JSON과 정지 std만으로 ORB IMU noise density·bias random walk를 확정하지 않는다.

배포 코드는 [imu.py](../../experiments/handheld_mapping/imu.py), [subscriber](../../experiments/handheld_mapping/imu_subscriber.py), [bridge 실행](../../tools/mapping_imu_bridge.sh)에 있다.
