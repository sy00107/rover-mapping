# 외장 IMU 융합 실험과 다음 촬영

## 현재 사용법

노트북의 mapping venv를 사용한다. Conda 환경은 사용하지 않는다.

```bash
cd /home/su/rover
bash tools/handheld_mapping.sh --imu-fusion \
  --imu-calibration /home/su/rover/outputs/handheld_mapping/calibration/imu_calibration.json
```

`종료 후 IMU 융합 시험`을 켜면 IMU 기록도 켜진다. 기본값은 꺼짐이다. 촬영 중 화면은 기존 stereo 추적이고, 종료하면 기존 stereo/TSDF를 보존한 다음 노트북에서 별도 stereo-inertial 결과와 TSDF를 계산한다. 융합에 실패하면 기본 지도를 보존하고 경고·partial 상태를 표시한다. RGB 기록·컬러 TSDF는 아직 없다. projector ON은 별도 선택 사항이며 기본 OFF다.

실기 촬영에는 기존 SPI owner와 telemetry tee가 필요하다. 두 번째 SPI owner를 띄우지 않는다.

```bash
# 별도 노트북 터미널에서 기존 중계를 실행하고 유지한다.
bash tools/mapping_imu_bridge.sh pi@raspberrypi.local
```

저장한 촬영의 자동 후처리만 실행하려면 다음을 사용한다. 결과 폴더가 이미 있으면 새 `--output` 경로를 지정한다.

```bash
bash tools/imu_fusion.sh /absolute/path/to/RUN --auto
# 원본 기록에서 수동으로 구간을 지정하는 실험
bash tools/imu_fusion.sh /absolute/path/to/RUN \
  --segment 1 --correlation-window-s 107.41360845439564 134.24168929950233
```

`--auto`는 움직임이 충분한 MCU boot 하나와 회전 상관 구간을 선택한다. reset 경계를 이어 적분하지 않고 시간 정렬 품질 조건도 유지한다. 정지 기록만으로 시차를 만들어 초기화를 강제하지 않는다. 자동 모드는 새 firmware timing 필드가 필요하다. 오래된 데이터는 명시적 `--allow-legacy` 실험만 가능하다.

## 빌드

기존 native core·Pangolin·vocabulary와 같은 mapping runtime을 준비한 뒤 실행한다.

```bash
bash tools/build_mapping_inertial.sh /absolute/path/to/ORB_SLAM3
```

기존 GUI의 stereo binary를 덮어쓰지 않고 `outputs/handheld_mapping/orb3_imu_build/rover_orb3_imu_compare`를 만든다. native 입력 검사는 `ctest`로 실행된다. body axes로 보정된 IMU를 URDF의 IMU 원점에 놓고 20° camera mount 및 저장된 CameraInfo rectification을 적용한다.

## 실기 기록과 결과

원본: `/home/su/rover/outputs/handheld_mapping/sessions/20261015_235533_69b722`.
180초 촬영 중 STM reset 1회가 있었으며, 서로 다른 boot의 데이터를 분리했다. 전체 raw는 그대로 보존했다.

- segment 0: accepted IMU 4,458개.
- segment 1: accepted IMU 2,842개.
- 두 boot 사이 약 1.329초를 이어 적분하지 않았다.
- 긴 정지 padding 때문에 전체 구간의 시차 추정은 half-motion 조건에서 거부됐다. 실제 움직임 구간만 상관 분석에 사용하면 segment 0 보정 −40ms, segment 1 −35ms 후보가 나왔다.
- segment 1 상관 0.99836, 앞·뒤 절반의 gyro query shift 모두 +35ms. 이는 IMU 시각에 −35ms를 더하는 부호다.

최종 수동 구간 지정 재생 결과는 RUN의 `orb_inertial/summary.json`에 있다.

- 1,079 frames, tracking OK 약98.98%.
- 실제 관성 초기화가 남아 있는 최종 map 244, keyframes 10개.
- 실제 IMU 전달 2,841개, 입력 부족으로 건너뛴 frame 0.
- `orb_fused=true`, `final_export_has_inertial_map=true`.
- 별도 TSDF: `dense_inertial/map_244_mesh.ply`, `dense_inertial/map_244_cloud.ply`.
- 10/10 keyframes 통합, vertices 7,576 / triangles 12,126. 원본 depth와 최대 timestamp 차이 295ns.

자동 선택 경로도 실제 native 재생을 완료했다. `/home/su/rover/outputs/handheld_mapping/imu_fusion_work/automatic_verified`에서 segment 1을 선택했고 1,079 frames, tracking OK 약98.70%, 최종 초기화 map 223을 확인했다. 비동기 ORB 재생에 따라 map ID와 최종 keyframe 선택이 달라질 수 있으므로 서로 다른 실행의 map ID를 같은 좌표계 식별자로 간주하지 않는다.

```bash
# 이번에 저장한 IMU 융합 지도 보기
bash tools/view_mapping.sh \
  /home/su/rover/outputs/handheld_mapping/sessions/20261015_235533_69b722 \
  --backend orb_inertial
```

## 해결한 실행 오류와 검증 한계

- 약39–50Hz 평균 IMU에서 영상 시각까지의 표본만 전달하면 pinned ORB core가 적분 구간을 확보하지 못하고 `PredictStateIMU`에서 SIGSEGV가 날 수 있었다. 실제 우측 표본으로 영상 시각을 bracket하고, 같은 표본의 중복 전달을 방지했다. IMU 재생만 원본 시간 간격으로 pacing한다.
- RAW와 realtime 사이의 정상 NTP slew를 전체 offset range로 거르던 오류를 수정했다. RAW는 인접 clock-pair step을 검사하고, monotonic 경로의 기존 검사도 유지한다.
- reset으로 같은 Map pointer의 초기 keyframe ID가 바뀌면 동일 map을 여러 번 export하던 문제를 pointer 기준으로 제거했다.
- 마지막 timing frame이나 한때의 초기화 상태로 성공을 주장하지 않는다. 실제 최종 export map의 `isImuInitialized()`를 확인한다.
- capture 당시 calibration·URDF hash와 실제 axis permutation을 replay 전에 확인한다.

관성 초기화 전에는 map reset이 반복됐고 최종 지도는 살아남은 map의 관측만 포함한다. 전체 장면의 완전한 지도가 됐다는 뜻은 아니다. 기본 stereo 결과보다 정확하거나 안정적이라고 아직 주장하지 않는다. `imu_clock_verified=false`, 외부변환·noise 검증 false를 유지한다. noise는 ORB EuRoC 시작값이며 센서 Allan variance 보정값이 아니다.

모든 계산과 결과는 노트북에 저장한다. 다음 구조 검토에서는 사전 지도 제작을 별도 offline reconstruction으로 분리하고, ORB·IMU는 주행 추정과 사전 지형 대조 보정에 사용하는 방안을 평가한다. STL mesh 자체는 ORB Atlas를 대체하지 않는다.
