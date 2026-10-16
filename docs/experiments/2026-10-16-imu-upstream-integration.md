# 조직 저장소의 카메라–IMU 시간 정렬·선택형 융합 도입

2026-10-16. [조직 main](https://github.com/lee-robot-lab/rover-mapping/tree/e42c5db)의 변경을 검토하고 시간 정렬과 별도 offline IMU 융합 경로를 현재 체크아웃에 도입했다. 구현 기준은 [82f61c7](https://github.com/lee-robot-lab/rover-mapping/commit/82f61c7655f6011fa16d13ff6f8f9fa20eafb482)이며 시간 진단·MCU timing 준비 변경도 포함한다.

현재 코드는 IMU 기록·정지 중력 정렬과 검증된 개별 IMU profile 입력은 있었지만, 일반 촬영 로그의 시계 드리프트·영상 회전과 gyro 시차를 검사하는 경로가 없었다. 따라서 시간 관계를 진단하고 품질 조건을 통과한 기록을 별도 관성 재생으로 비교하는 기능은 필요하다고 판단했다. 실제 하드웨어의 동기화 정확도나 지도 개선이 검증됐다는 뜻은 아니다.

## 현재 체크아웃에서의 실행

```bash
# 종료된 촬영의 시각 관계만 진단한다. 기존 출력이 있으면 새 --output을 지정한다.
bash tools/imu_time_alignment.sh /absolute/path/to/RUN

# 새 촬영: 기본 stereo 결과를 저장한 다음 별도 IMU 융합을 시도한다.
bash tools/handheld_mapping.sh --imu-fusion

# 기록된 새 timing 형식의 데이터를 별도로 재생한다.
bash tools/imu_fusion.sh /absolute/path/to/RUN --auto
```

GUI의 `종료 후 IMU 융합 시험`은 기본 꺼짐이고 선택하면 IMU 동시 기록을 켠다. 기존 IMU calibration과 SPI owner/tee가 필요하다. 자동 융합은 MCU 평균 읽기 시각과 카메라 callback 시각을 보존한 기록을 요구한다. 구형 기록은 자동 모드에서 거부하며, 명시적인 legacy 실험을 동기화 검증으로 취급하지 않는다.

촬영 중에는 기존 stereo 경로를 사용한다. 종료 후에는 기본 3D와 지형을 먼저 보존하고 `orb_inertial/`, `dense_inertial/`에 별도 결과를 생성한다. 융합 실패·관성 초기화 실패는 partial과 이유를 표시하고 기본 지도를 유지한다. 실제 최종 export에 초기화된 관성 map이 있는 경우에만 해당 map을 TSDF에 사용한다.

## 통합하면서 유지한 기능

- depth-first 영상 묶음과 수신 버퍼, 시각화의 1/2/3m 비교, 중력·지형·장애물 분류를 유지했다.
- 원거리 표면 보정, 주황 장애물 평활화, 작은 초록 구멍 보완을 유지했다.
- 기존 실측 개별 표본 profile 계약은 `inertial_profile.py`로 보존하고 `inertial.prepare_input` API도 유지했다.
- 기존 stereo/검증 profile native 실행기는 변경하지 않았다. 조직의 관성 배치·우측 bracket·재생 pacing은 `inertial_runner.cpp`와 별도 `rover_orb3_imu_compare`에 도입했다.
- RGB/reference-map/editor, 기본 depth를1m로 바꾸는 변경, projector ON 옵션은 이번 시간 정렬 작업에 포함하지 않았다.

실험용 관성 결과는 `imu_clock_verified=false`, 외부변환·noise 검증 false를 유지한다. MCU/Pi/카메라의 시계 속도와 시차는 기록별로 추정하며 다른 촬영에 고정 offset을 복사하지 않는다.

## IMU RAW host clock이 없는 기존 촬영 복구

`mapping_imu_bridge.sh`의 telemetry patch에는 IMU의 `host_monotonic_raw_ns`가 없었고, 카메라에만 RAW clock pair가 저장되는 경우가 있었다. bridge patch를 수정해 다음 중계 실행부터 realtime·MONOTONIC·RAW와 조회 bracket span을 함께 저장한다. 이미 실행 중인 Pi 중계나 MCU firmware는 변경하지 않았다.

카메라에 완전한 clock pair가 있는 기존 기록은 다음 명시적 옵션으로 복구를 시도할 수 있다.

```bash
bash tools/imu_fusion.sh RUN --auto --recover-camera-clock
```

같은 Pi boot의 MONOTONIC→RAW 차이를 카메라 callback pair 사이에서 선형 보간한다. 카메라 clock pair 범위 밖 IMU는 버리고 외삽하지 않는다. 이 보간은 host clock domain 변환이며 IMU의 물리적 측정 시각을 복구하는 것은 아니다. 보고서 `clock_recovery.imu_raw_clock_measured=false`와 동기화 검증 false를 유지한다. 기본 자동 경로는 계속 직접 측정한 시계를 우선 사용하며 복구 옵션은 기본 꺼짐이다.

최소 40개·10초의 증가하는 clock pairs, 최대 pair 간격1초, 최대 조회 bracket100µs, clock 차이 인접 변화1ms·변화율2000ppm 이하, 짝수 pair로 홀수 pair를 예측하는 검증 오차250µs 이하를 요구한다. IMU와 camera의 realtime−MONOTONIC 관계가5ms 이내여야 한다. 일부 IMU에만 RAW가 있는 불완전한 기록을 보간으로 숨기지 않는다. 복구 후에도 기존20ms affine 잔차·IMU 공백·시차 상관·절반 구간 일관성 검사를 그대로 수행한다.

## 실제 기존 기록 진단

`20261015_152413_6fea4a`에서 IMU18,727개와 영상4,589장을 분석했다. 이 기록의 카메라에는 새 realtime/RAW callback 필드가 없다. legacy clock 관계로 진단했으며, MCU→Pi affine 잔차의 구간 중앙값 변화가 약158.6ms로20ms 기준을 초과해 `clock_model_unstable`로 기각했다. 자동 적용할 offset은 `null`이며 원본·기존 map/지형을 변경하지 않았다.

진단은 `outputs/handheld_mapping/reviews/20261016_imu_upstream_integration/existing_run_alignment/`에 보존했다. 통합 전 미커밋 파일은 같은 검토 폴더의 `before/`와 SHA-256 manifest에 보존했다.

## 검증 범위

별도 관성 native 실행기를 현재 `external/ORB_SLAM3`와 기존 비교 runtime으로 빌드했고 `ctest`의 실제 C++ 입력 검사가 통과했다. 입력 단위·중복·역행·공백·영상 우측 표본·map pointer 중복 export를 확인한다. 기존 stereo 실행기는 덮어쓰지 않았다.

알려진 시간차·드리프트·wrap/reset·RAW/NTP slew·운동 부족·불안정 모델의 기각, 기존 실측 profile, 자동 구간 선택, GUI opt-in, IMU 종료 후 처리 순서, 실패 시 기본 지도 보존을 회귀 검사한다. STM 패치의 portable C 검사는 실행하며 별도 baseline이 필요한 firmware 통합 항목은 환경이 없으면 skip한다.

현재 체크아웃 전체 회귀는 **437 passed,14 skipped**(53.69초)였다. 기존 촬영·전송·중력/지형·2m 보정·주황/초록 표시 검사도 포함한다. skip된 항목은 이번에 검증했다고 주장하지 않는다. `git diff --check`와 새 shell wrapper3개의 `bash -n`도 통과했다.

새 실제 촬영, Pi/STM 배포·flash, 동기화 정확도·관성 지도 품질의 실측 검증은 이번 작업에서 수행하지 않았다. 가져온 조직의 실기 문서에 등장하는 `/home/su/rover` 경로와 성공 기록은 조직 측 실행 기록이며 이 노트북에서 재현했다고 해석하지 않는다.
