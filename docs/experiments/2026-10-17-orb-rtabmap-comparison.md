# 저장된 stereo IR의 ORB-SLAM3 / RTAB-Map 비교

원본 RUN: `outputs/handheld_mapping/sessions/20261016_174639_b1d8ed`.
결과 폴더: `outputs/handheld_mapping/comparisons/orb_rtabmap_20261016_174639`.

기존 ORB offline 결과와 RTAB-Map 0.23.13의 별도 replay를 비교한다. 원본과 기존 ORB 산출물은 보존한다. RTAB-Map은 모든 5,447 stereo IR 쌍을 순서대로 처리하며 IMU/encoder는 사용하지 않는다. 보정은 촬영 당시 rectified CameraInfo의 focal length / principal point / baseline이다. RTAB-Map local camera transform은 identity이므로 출력은 ORB와 같은 camera optical pose다.

RTAB-Map 설정: GFTT/BRIEF visual odometry 및 loop features (`Vis/FeatureType=6`, `Kp/DetectorStrategy=6`), detection rate 제한 없음, STM 30, depth occupancy grid 생성 꺼짐. 그 외 설치 버전 기본값이다. `rtabmap.db`에 데이터와 graph를 저장한다. 추적 실패 시 자동 pose reset을 넣지 않는다.

- `trajectories.png`: 시작 camera SE3만 정렬한 궤적. scale fitting 없음.
- `maps_comparison.html`: 자체 포함 Plotly 3D 두 지도; 각 패널 회전·확대 가능.
- `maps_comparison.png`: 공통 축 범위의 X–Z 지도 투영. 색은 camera optical Y이며 중력 높이가 아니다.
- `orb_common_depth*.ply`, `rtab_common_depth*.ply`: 같은 관측으로 생성한 TSDF cloud / mesh.
- `runner_summary.json`, `timings.csv`: RTAB 추적 성공과 처리 시간.
- `map_comparison_summary.json`: 실제 공통 depth 수와 두 알고리즘의 위치 차이.

지도 비교는 시간순 ORB keyframe 중 최대 1Hz로 선택하고, RTAB 최적화 graph를 해당 시간에 translation 선형 보간 + quaternion SLERP한다. graph node 사이가 2초를 넘거나 범위를 벗어나면 제외한다. 원본 depth는 2ms 이내로 대응시킨다. 두 지도에 정확히 같은 depth와 공간 지지점 필터를 적용한다. 최대 깊이 2m / TSDF voxel 2cm / truncation 8cm. 알고리즘별 pose를 사용하는 temporal filter는 사용하지 않는다. 결과는 기존 고밀도 ORB 지도와 별개다.

정답 궤적이 없으므로 위치 차이는 정확도 오차가 아니다. ORB는 기존 실행 결과이며 RTAB은 현재 실행이므로 속도 수치를 엄밀한 동시 benchmark로 비교하지 않는다. RTAB graph 보간으로 순간 움직임 일부가 평활화될 수 있다.

재실행 (새 output 필요):

```bash
sudo apt-get install -y ros-humble-rtabmap
bash tools/compare_orb_rtabmap.sh \
  outputs/handheld_mapping/sessions/20261016_174639_b1d8ed \
  outputs/handheld_mapping/comparisons/orb_rtabmap_new
```

실행 결과: 5,447 frame 모두 RTAB odometry pose 반환, lost 0, 최적화 graph 388 nodes. ORB offline keyframe은 712개다. 지도에 사용한 공통 depth는 188개다. 시작 SE3 정렬 후 공통 관측 시각에서 위치 차이 중앙값 4.234m / p95 6.476m다. 현재 설정의 RTAB 궤적은 ORB에 비해 크게 휘고 Y 방향으로도 벌어지며, 공통-depth 지도에서도 표면이 분산된다. 이는 이 RUN과 이 설정의 비교 결과이며, 정답 데이터 또는 RTAB parameter sweep에 기반한 일반 성능 순위는 아니다.

검증: native Release build 완료, 3개 회귀 테스트 통과(키프레임 시간 정렬, 역순 TUM 거부, SE3 보간·시간 공백 제외). 최종 PNG 두 개를 시각 확인했고, 자체 포함 HTML과 PLY 산출물 생성을 확인했다.

## 휘어짐 원인 조사

SQLite를 read-only로 열어 Node.pose(최적화 전 5,447 pose), Link, Statistics, Feature를 검사했다. `diagnosis.png/json`, `rtab_odometry_raw.tum`, `stereo_depth_diagnosis.png/json`, `stereo_depth_samples.csv`에 근거를 저장했다.

- 첫 기록된 전역 loop 이벤트는 288.406초. 그 전인 약30초에 raw odometry는 ORB 기준 위치 2.238m / 방향 18.91° 차이, 약60초에 4.593m / 53.95° 차이였다. 따라서 최초의 휘어짐은 전역 loop 최적화 이전 odometry에 존재한다. ORB는 정답이 아닌 비교 기준이다.
- DB에는 map 0 하나만 있고 lost는 0이다. 자동 reset/map 간 좌표 혼합은 이번 현상의 설명이 아니다.
- 최적화 graph node에서 raw 대비 위치 변화 최대는 1.865m다. 후반 graph 보정도 결과에 영향을 주지만, 초기 오차를 설명하지 않는다. neighbor 외 closure link가 있으므로 모든 graph 변경을 전역 loop 하나로 단정하지 않는다.
- 초기 retained node 35개의 Feature 3D point를 같은 IR/depth 픽셀과 대조했다. Feature xyz를 카메라 내부행렬로 재투영하면 원래 특징점 위치와 중앙 오차 약 0.000003 pixel로 일치한다. depth/left 보정 TF는 기록상 identity이다. 원본 depth의 공간 지지점 필터 후 0.3–2m인 8,670개 관측에서 stereo depth / measured depth 비율 중앙값 4.387, 2배 초과 79.20%, ±20% 이내 11.30%다. 물리적 깊이가 정답이라는 독립 검증은 없지만, 현재 stereo 대응/삼각측량에 심각한 불일치가 있다는 강한 증거다. 잘못된 특징점 3D가 odometry drift를 일으켰다는 설명이 유력하다. 특징점 일부가 실제 odometry inlier였는지는 이 기록만으로 확인하지 못했다.
- 보정 fx/fy/cx/cy와 baseline은 ORB 설정과 일치한다. 현재 stereo OpticalFlow=true, Vis feature=GFTT/BRIEF 구성이다. 이 한 설정의 실패를 RTAB-Map 일반 성능으로 해석하면 안 된다.
- 비교 실행기 결함도 확인했다. `slam.process(data,pose)`가 info.reg.covariance를 전달하지 않아 DB에 odometry linear/angular variance=1로 기록됐다. source를 실제 covariance 전달로 수정하고 재빌드했다. 기존 결과 DB/PNG/PLY는 수정 전 실행 결과 그대로 보존했다. 이 결함은 graph constraint 신뢰도에 영향을 주지만, raw odometry 자체가 이미 휘어지는 원인은 아니다.

다음 분리 실험은 동일 초기 구간에서 stereo optical-flow 대신 descriptor matching을 사용하고, 원본 depth를 직접 사용하는 RGB-D odometry를 비교하는 것이다. 원인 확정에는 그 ablation이 필요하다.
