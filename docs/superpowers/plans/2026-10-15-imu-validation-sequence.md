# Camera–IMU Validation Plan

> **For agentic workers:** Use superpowers:executing-plans to perform the checks in order. This plan covers prerequisite validation; it does not authorize calling unverified timestamps or extrinsics verified.

**Goal:** 시간 정렬 후보의 반복성, 센서 timestamp 의미, 장착 축을 차례로 확인해 관성 융합의 입력 조건을 확정한다.

**Architecture:** 기존 종료 RUN을 노트북에서 읽기 전용 분석한다. 새 실기는 같은 수집 경로와 단일 SPI bridge를 유지하며 비교 기록을 추가한다. 검증 단계에서는 기존 stereo ORB와 TSDF 설정을 유지한다.

**Tech Stack:** 노트북 `.venv-mapping`, NumPy/Matplotlib, Pi native ROS2 Jazzy/RealSense ROS4.58.4, 외장 ISM330DHCX, STM32 SPI telemetry.

**Spec:** `docs/experiments/2026-10-15-imu-time-alignment.md`와 사용자의 “순서대로 이제 나아가자”.

후속 시차 차이 조사는 [조사 결과와 코드 보완](../../experiments/2026-10-15-imu-timing-forensics.md)에 기록했다. yaw의 구간별 clock 잔차 변화가53.689ms여서 새 guard에서 기각됐고, roll/pitch9.752ms는 진단 후보로 남았다. 다음 촬영부터 camera callback의 MONOTONIC_RAW를 함께 저장한다. 고정 offset·관성 융합은 여전히 보류한다.

정지 RAW 기록도 수행했다. [125초 실기 결과](../../experiments/2026-10-15-raw-stationary-clock-probe.md)에서 MONOTONIC−RAW 변화약41ms, 선형 속도 보정 후 구간 중앙값 변화약9.8ms를 확인했다. RAW 기준에도 MCU 상대 속도 차이약0.57%가 남았다. 이 정지 시험으로 회전 offset은 정하지 않았으며, 다음은 firmware 평균 표본 시각과 packet 호환성 검토다.

## Global Constraints

- 영상·IMU 원본과 지도는 노트북에 저장한다. Pi의 영구 저장공간에 기록을 추가하지 않는다.
- SPI owner는 기존 joystick bridge 하나다. 관찰 프로그램이 SpiLink를 새로 생성하지 않는다.
- 모터 명령은 사용자가 조이스틱으로 조작한다. metadata 확인은 구독·설정 조회만 한다.
- native ORB는 현재 stereo 모드다. `imu_clock_verified=false`, `orb_fused=false`, `extrinsic_provisional=true`를 유지한다.
- 촬영 설정은 IRleft/right+depth640×48015FPS, projector OFF/color OFF인 기존 비교 조건이다. projector·RGB 시험은 별도 조건으로 분리한다.
- 시간 후보는 기록별 affine 시계 모델에 더한다. raw MCU timestamp에 고정 -45ms를 더하지 않는다.

## Review Focus

- legacy clock 변환과 새 callback clock pair 기록을 동일한 하드웨어 시각 증거로 취급하지 않는다.
- 정지한 절반·주기적 움직임·공백·reset은 추정 후보에서 제외한다.
- 같은 부팅 반복과 다른 부팅 반복을 구별한다. 부팅 여부가 기록되지 않았으면 미확인으로 둔다.
- yaw만 맞아도 3축 회전·lever arm까지 검증됐다고 하지 않는다.
- metadata의 clock_domain·frame_timestamp·노출 필드는 실제 메시지에 존재하는 값만 해석한다.

## 1. 기존 기록으로 재현성 범위 확인

- [x] 종료된9개 IMU RUN을 현재 분석기로 재검사하고 각 `imu_time_alignment_repeatability`에 별도 출력.
- [x] 9개 RUN의 입력 hash 일치와 미검증 flags 유지 확인.
- [x] 결과 집계: 추정6개(-70~-40ms), 기각3개(정지2개, half 시차 불안정1개).
- [x] 노트북 `outputs/handheld_mapping/20261015_imu_validation/repeatability_summary.json`과 README에 결과 보존.
- [ ] 새 callback clock pair를 가진 회전 기록을 같은 부팅에서2개 이상 추가하고, 다른 부팅에서1개 이상 추가.

수동 회전 후속 RUN `20261015_215301_23dbf4`를 추가했다. 초반 MCU reset이1회 있어 전체 RUN 시간 정렬은 기각했다. reset 전76개 accepted를 제외하고 이후segment1의4436샘플/112.429초만 별도 분석해 -40ms/상관0.99531 후보를 얻었다. 이전 -45ms와5ms 차이지만, half 후보는 -35/-50ms이고 clock 수신 잔차 p5/p95가 약-28.9/+33.8ms라 고정 offset 승인은 보류한다. 원본과 전체 기각 결과를 유지했고 후보 CSV는 내보내지 않았다.

이 RUN은 영상 좌우/depth 각1565장, queue 누락0, offline ORB1565쌍 전부 추적·map0 유지, TSDF48keyframe 융합이었다. 함께 기록한 camera metadata는 각stream1409개이며 전부 해당 원본 header와 매칭됐다. domain은 global_time, 노출 중앙값3.212ms/p954.067ms였다. 모든 IMU event의 `poll_only=true`, `selected_cmd=[0,0,0]`를 확인했다. 회전은 yaw 위주여서 roll/pitch 검증은 아직 필요하다. 자세한 파일은 노트북 `outputs/handheld_mapping/20261015_hand_rotation_repeat/README.md`에 있다.

새 기록에서는 5초 정지 후 속도·방향·간격이 다른 회전을 60~90초 수행하고 마지막5초 정지한다. 사용자가 로버를 먼저 정지한 뒤 종료한다. 비교에는 각 half의 상관·peak와 clock rate·잔차를 함께 사용한다. 후보가10ms 넘게 달라지면 고정 offset 적용을 보류하고 metadata와 MCU timing을 먼저 조사한다. 이10ms는 조사 기준이며 동기화 정확도의 보증이 아니다.

기존 분석기는 다음 명령을 사용한다. 이미 있는 출력은 덮어쓰지 않고 `--output`으로 새 경로를 지정한다.

```bash
bash /home/su/rover/tools/imu_time_alignment.sh /absolute/path/to/RUN
```

## 2. 실제 camera·MCU timestamp 의미 확인

- [x] 최신 camera.log에서 RealSense ROS4.58.4/LibRealSense2.58.4, D435 FW5.13.0.50, USB2.1 확인.
- [x] 해당 버전의 공식 driver source에서 timestamp 변환 분기와 metadata 필드 확인.
- [x] Pi 연결 복구 후 카메라 metadata를25초 수집. 좌우 각360장, depth362장, image/metadata header 전부 일치. domain은 모두 `global_time`, 실제 약15.016FPS.
- [x] Pi checkout의 decoder/firmware 소스를 수집하고 git commit `40499bcb1acaabf6bbbf604795caf28b6a952739` 및 source hash 보존. 실제 flash binary와 source의 동일성은 미검증.
- [x] source에서 `t_ms=now`는 제어 loop 초반 `HAL_GetTick()` 값이고, IMU는1ms poll로 누적해 `read_sensors()` 끝의 `Imu_TakeMean()`에서 평균/초기화함을 확인. 각 원시 표본 시각과 평균 구간의 실제 첫/끝 시각은 packet에 없다.
- [ ] camera clock domain과 ROS header 변환 관계를 실제 수집값으로 검증한 뒤 drift·offset 모델의 한계를 재평가.

공식 [frameSystemTimeSec/publishMetadata 구현](https://github.com/realsenseai/realsense-ros/blob/4.58.4/realsense2_camera/src/base_realsense_node.cpp#L673)은 HARDWARE_CLOCK일 때 camera 기준 시각을 ROS time base에 더하고, 다른 domain은 frame timestamp를 epoch로 사용한다. metadata는 `clock_domain`, `frame_timestamp`, `frame_number`와 지원되는 장치 metadata를 제공한다. 이 source 확인만으로 현재 Pi의 domain을 확정하지 않는다. frameset 경로의 공통 ROS stamp와 개별 frame metadata 관계도 확인한다.

새 촬영이 실행 중일 때 아래처럼 기존 토픽을 조회한다. 노트북 터미널에서 실행하며 출력 파일은 노트북에만 만든다. 토픽 이름은 먼저 `ros2 topic list -t`로 실제 존재 여부를 확인한다. SSH 대상은 연결 확인된 주소로 바꾼다.

```bash
mkdir -p /home/su/rover/outputs/handheld_mapping/20261015_imu_validation/camera_metadata
ssh -T -o BatchMode=yes -o ConnectTimeout=5 pi@raspberrypi.local \
  'bash -c "source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=67 RMW_IMPLEMENTATION=rmw_fastrtps_cpp; ros2 topic list -t"'
ssh -T -o BatchMode=yes -o ConnectTimeout=5 pi@raspberrypi.local \
  'bash -c "source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=67 RMW_IMPLEMENTATION=rmw_fastrtps_cpp; timeout 10s ros2 topic echo /camera/camera/infra1/metadata realsense2_camera_msgs/msg/Metadata --once --qos-reliability best_effort"' \
  > /home/su/rover/outputs/handheld_mapping/20261015_imu_validation/camera_metadata/infra1.yaml
```

첫 snapshot은 domain 확인용이다. 정밀 timing 확인에는 여러 연속 프레임의 frame_number·hardware timestamp·노출 metadata와 image header를 함께 비교해야 한다. 샘플이 없거나 지원되지 않는 필드가 있으면 미확인으로 남긴다.

이번 후속 실기에서는 위 snapshot보다 긴25초 구독으로 header/metadata 대응을 확인했다. `actual_exposure`는 시작 직후 최대64921µs, 전체 중앙값3018µs/p953592µs였다. `hw_timestamp-sensor_timestamp` 중앙값1509µs는 노출 중앙값의 절반과 대응했다. 공식 [SDK metadata 정의](https://github.com/realsenseai/librealsense/blob/v2.58.4/include/librealsense2/h/rs_frame.h#L27)상 `sensor_timestamp`는 노출 중간, `hw_timestamp`의 원래 FRAME_TIMESTAMP는 readout/transmit 시작 시각이다. 다음 회전 비교에는 노출·sensor timestamp를 동시 보존해야 한다. 이번 global_time 확인이 MCU와의 하드웨어 동기화 정확도를 검증한 것은 아니다.

## 3. 장착 축·중력 방향 확인

- [x] latest RUN의 저장된 robot.urdf로 optical→body 회전을 구성하고, 알려진 +Z 회전 합성값으로 변환 방향·부호 확인.
- [x] 시간 정렬된 latest RUN의 signed gyro와 영상 회전을 비교: 활성467구간, yaw 부호100% 일치, yaw 상관0.99970, 크기 회귀비1.00757.
- [x] 오프라인 진단 스크립트·그림·원본 hash를 노트북 `outputs/handheld_mapping/20261015_imu_validation`에 보존.
- [ ] 실물 장착 방향·IMU 중심 위치를 URDF와 비교하고, 가능한 범위에서 roll/pitch도 별도 회전 기록으로 확인.
- [ ] 수평계 등 독립 기준으로 정지 자세를 확인하고 중력 방향·gyro bias를 측정. 가속도 평균이 곧 정확한 장착각이라고 간주하지 않는다.

별도 재부팅 정지 시험에서는599회 POLL만 수행했다. 첫 응답은 이전 SPI 교환의 오래된 프레임이라 기존 runtime 규칙대로1개 제외했다. 유효 고유474개, 중복124개, IMU 무효0/CRC 실패0, 고유 수신약39.49Hz, `imu_n` 중앙값4였다. 기존 보정 후 gyro 평균 XYZ는 약0.01683/0.01894/0.00554°/s, 표준편차0.03484/0.03571/0.03634°/s, 가속도 크기 평균1.00218g였다. calibration은 변경하지 않았다. 이 별도 정지 시험은 회전 시차 반복성을 대체하지 않는다. 초기 SPI 응답을 포함한 초안 집계 대신 `imu_reboot_stationary_validated.json`을 사용한다.

현재 yaw 진단은 동일 기록에서 추정한 시차와 URDF를 사용했다. 3축 회전 보정과 camera↔IMU translation을 식별한 결과가 아니다. 노트북 전용 진단이며 실기 runtime 입력은 바꾸지 않았다.

수동 roll/pitch RUN `20261015_221059_5e303d`를 추가했다. 영상 좌우/depth 각1630장·누락0, offline ORB1630쌍 모두 추적·map0 유지, IMU accepted4595/reset0/tee 누락0이었다. 시차 후보-55ms/상관0.99632이며 앞·뒤 절반도 동일했다. URDF body 축의 영상/gyro 비교는 roll_X 상관0.99775·크기 회귀비1.00644, pitch_Y 상관0.99477·회귀비1.00734로 방향과 크기가 잘 대응했다. 단, 장착 위치 실측과 독립 extrinsic 검증을 대신하지 않는다.

이전 수동 yaw의 post-reset 후보-40ms와 이번 -55ms는15ms 차이로, 고정 offset 적용을 보류하는 조사 기준10ms를 넘는다. 평균 IMU 구간 시각과 camera 노출 중심·SPI 시차를 추가 확인해야 한다. 사용자의 기울임 동작 기록은 충분히 확보했으며 다음 확인은 원본 기반 소프트웨어 작업이다. 결과·그림·분석 script는 노트북 `outputs/handheld_mapping/20261015_roll_pitch_repeat`에 보존했다.

## 4. 관성 융합 비교로 넘어가는 조건

단계1~3의 미확인 항목을 먼저 해결한다. 이후 실제 IMU 표본 시각·단위·축·sampling/averaging 특성·noise를 명시한 별도 구현 계획을 작성한다. 같은 영상과 IMU로 stereo baseline과 stereo-inertial을 offline 비교하고, 추적 손실·map 분리·재방문 오차·초기화 안정성을 평가한 후 실시간 적용한다. 비교 실패 시 기존 stereo 경로를 유지한다. 이 문서 시점에는 관성 융합 구현을 시작하지 않았다.

## 현재 실기 조건

초기 DNS 조회 실패 후 사용자가 배터리 대신 어댑터로 Pi를 재부팅하고 STM에도 별도 전원을 공급했다. SSH는 복구됐다. 사용자는 현재 조향·구동을 사용할 수 없고 카메라는 가능하다고 알려줬다. D435 재연결 후 위 metadata 구독과 IMU POLL-only 시험을 수행했다. UDP joystick bridge는 다시 시작하지 않았고 모터 속도 명령은 전송하지 않았다. 다음 회전 재현성 검증은 실제로 회전할 수 있는 조건에서 진행해야 한다. 카메라 probe는55초 제한으로 자동 종료했다.
