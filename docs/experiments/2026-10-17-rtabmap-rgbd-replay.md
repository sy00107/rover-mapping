# RTAB-Map 원본 depth 재처리

기존 stereo IR 실행에서 발생한 깊이 불일치를 피하기 위해, 저장된 좌 IR을 texture로 사용하고 촬영 때 보존한 Z16 depth를 RTAB-Map의 RGB-D API에 직접 넣는다. 컬러 촬영이나 재촬영은 필요 없다. IMU/encoder는 사용하지 않는다.

소스 RUN: `outputs/handheld_mapping/sessions/20261016_174639_b1d8ed`

- 초기 검증: `outputs/handheld_mapping/comparisons/rtab_rgbd_20261016_174639_pilot` (900 frame / 약60초)
- 전체 재처리: `outputs/handheld_mapping/comparisons/rtab_rgbd_20261016_174639_full`
- 기존 stereo 결과는 그대로 보존한다.

## 입력과 보정

좌 IR 5,447장과 depth 5,447장은 전부 timestamp 차이가 0이다. depth/left의 크기 640×480과 K가 같고 T_left_depth=identity, distortion=0인 기록을 확인했다. 그 조건을 코드에서도 검사한다. 다른 보정의 촬영은 임의로 같은 픽셀을 사용하지 않고 등록/reprojection을 요구하며 중단한다. depth_scale=0.001m를 float32 metre로 변환하고 0.25–2m만 사용한다.

GFTT/BRIEF와 기본 F2M odometry를 유지한다. 원래 실행기에서 누락했던 `info.reg.covariance`를 SLAM graph에 전달하고 raw odometry TUM, inliers/matches/covariance와 timing을 저장한다. 최초 covariance가 큰 것은 초기화 상태다. 초기 DB에서 이후 linear/angular variance가 기존의 상수1이 아닌 실제 추정값으로 저장되는 것을 확인했다.

## 초기 검증

동일 시각의 ORB offline keyframe을 참고 기준으로 사용한다. ORB는 정답 궤적이 아니다.

| 초기 약60초 비교 | 기존 stereo raw odometry | 측정 depth RGB-D raw odometry |
|---|---:|---:|
| 공통 시각 수 | 184 | 184 |
| 위치 차이 중앙값 | 3.044m | 0.00493m |
| 위치 차이 p95 | 4.584m | 0.01406m |
| 약60초 끝 위치 차이 | 4.593m | 0.01321m |
| 약60초 끝 방향 차이 | 53.949° | 1.162° |

900 frame 모두 추적 성공, graph 25 nodes. 초기 결과를 근거로 전체 처리에 진행했다. 이 실험은 depth 입력 방식과 depth 범위 제한을 함께 변경했으므로, 개별 설정 하나의 인과 효과를 분리한 실험은 아니다. raw odometry의 개선은 graph covariance 전달 변경으로 설명되지 않는다.

## 지도와 실행

최적화된 최종 RTAB graph node 시각의 원본 depth를 직접 융합한다. graph 사이의 pose 보간은 하지 않는다. 지면을 평평하게 만드는 보정이나 mesh 평활화를 적용하지 않는다. 공간 지지점 필터, depth 2m, TSDF voxel 1cm, truncation 4cm를 사용한다. 중력 정렬은 적용하지 않은 초기 camera optical 좌표다.

새 결과 생성:

```bash
bash tools/replay_rtabmap_rgbd.sh \
  outputs/handheld_mapping/sessions/20261016_174639_b1d8ed \
  outputs/handheld_mapping/comparisons/rtab_rgbd_new
```

RTAB 지도만 native Open3D 창으로 열기:

```bash
bash tools/view_rtabmap.sh
# 특정 결과:
bash tools/view_rtabmap.sh outputs/handheld_mapping/comparisons/rtab_rgbd_new
# 점군:
bash tools/view_rtabmap.sh --cloud
```

`rtab_map_mesh.ply`, `rtab_map_cloud.ply`, `rtab_map_summary.json`이 native 지도 결과다. 원본 촬영과 기존 맵을 덮어쓰지 않는다. `rgbd_evaluation.json/png`는 기존 stereo 대비 ORB와의 차이를 기록한 진단용이며 정확도 인증이 아니다.

## 전체 처리 결과

5,447 frame 모두 추적 성공, lost=0, 최종 graph 186 nodes. 해당 node의 원본 depth 186장을 모두 융합했고 timestamp 누락은 없다. 최종 mesh는 784,149 vertices / 1,201,970 triangles다.

같은 ORB keyframe 시각 711개에서 raw odometry 위치 차이는 기존 stereo 중앙값4.236m / p95 5.738m에서 RGB-D 중앙값0.0559m / p95 0.1125m로 줄었다. 끝 시각 위치 차이0.06595m / 방향1.641°다. 이는 정답 오차가 아니라 두 알고리즘 결과의 차이다.

최적화 graph의 경우 2초 이하 보간 구간에 있는 ORB keyframe 97개에서 위치 차이 중앙값0.0844m / p95 0.1176m, 방향 차이 중앙값2.390°다. raw 비교와 표본 수가 다르므로 두 수치만으로 graph 개선/악화를 판단하지 않는다. 실제 지도 융합은 이 보간 표본이 아닌 최적화 node 전체186개의 실제 depth로 수행했다.

검증: native Release build 성공; 입력 정렬·보정조건·원본 보존·키프레임 순서·pose 보간 관련5개 테스트 통과. RGB-D API 실제900-frame pilot 및5,447-frame 전체 replay 성공. 최종 map과 trajectory preview를 시각 확인했다. `view_rtabmap.sh` 기본값을 새 전체 결과로 변경했다. 대규모 궤적 휘어짐은 크게 줄었으나 원본 depth 노이즈, 가림 영역, 중력 미정렬 및 수cm 수준의 알고리즘 간 차이는 남는다.

## 지면·장애물 분류와 표면 보완

`surface_refined_v1/`에 원본과 분리된 보정 지도 생성. 같은 RGB-D 최종186 pose와 depth를 기존 ORB용 `DepthFilter(surface_cleanup=True)` / `GroundGrid` / surface classification / display correction 함수에 넣었다. 별도 ORB pose나 ORB terrain은 사용하지 않는다.

동일 Pi receipt로 RTAB pose와 겹치는 정지 calibrated IMU 구간을 검증하고 보존 URDF의 mounting을 이용해 Z-up으로 회전·이동한다. IMU 동적 융합 또는 검증된 실측 extrinsic을 의미하지 않는다. 전역 평면으로 지형을 펴는 변형은 하지 않는다.

원거리 inverse-depth 보정, 최대3초 이내 주변4개 frame의 다중시점 확인, 2m depth / 1cm TSDF. terrain과 mesh는 동일 filtered depth로 생성한다. 작은 분리 component(100 triangles 미만)를 제거한다. GroundGrid는5cm 셀 / 최소2회 관측이며 지면 모드를 카메라 주변에서 연결한다. surface 색 분류는 국소 지면 높이±2cm, 장애물은 지면 위6cm 이상이며, 분류가 확인되지 않은 영역은 회색이다. 실제 경사와 단차는 유지한다.

- 색상: 초록 지면210,933 / 주황 장애물51,051 / 회색 미분류30,227 vertices.
- filtered depth186장 모두 융합; 최종292,211 vertices / 522,392 triangles.
- 작은 component24,354 triangles 제거.
- 지름5cm 이내 초록 지면 내부의 닫힌 평면 구멍1,116개에 추정 표시용4,081 triangles 추가. topology 위반 후보10개 제외.
- 주황 내부23,321 vertices 평활화, 최대이동0.00643m. 외곽·혼합색 경계·뚜렷한 모서리 고정.
- 원본 map/depth/RTAB trajectory는 그대로 유지. 작은 지면 패치는 새 관측이나 통행 가능 판정이 아니다.

기본 `bash tools/view_rtabmap.sh`는 색이 보존된 refined mesh를 native Z-up 창으로 연다. 점군은 `--cloud`, 이전 단색 결과는 `--raw` 옵션이다. 원본 일반 viewer는 직접 PLY의 색을 단색으로 바꾸므로 refined용 viewer가 저장된 색을 그대로 읽는다.

```bash
# 현재 결과 보기
bash tools/view_rtabmap.sh
# 보정 전 지도
bash tools/view_rtabmap.sh --raw
# 다른 촬영/새 revision에 같은 처리 적용 (새 output 필요)
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv-mapping/bin/python -m experiments.handheld_mapping.rtabmap_surface build \
  outputs/handheld_mapping/comparisons/rtab_rgbd_20261016_174639_full \
  outputs/handheld_mapping/comparisons/rtab_rgbd_20261016_174639_full/surface_refined_v2
```

검증: 기존 depth/지면색/작은 지면 구멍/장애물 평활화/중력 관련67개 테스트 통과(Qt offscreen). 실제 저장된 PLY의 palette와 분류 수, 보정 최대이동·hole 제한을 검사했고 preview를 시각 확인했다. 출력 summary에는 input checksum과 보정/분류 통계를 기록했다.

## 지면의 검은 점·줄무늬 표시 개선

`surface_refined_v4`가 기본 표시 결과다. 실제 native 렌더링 비교에서 v1의 검은 줄무늬는 대부분 가늘게 연결된 메시 공백에서 보였다. 지면-only face381,691개 중31,420개는 법선 Z가 음수였지만, 음영/법선 수정과 닫힌 구멍 보완만 적용한 v2는 시각적 개선이 작았다. 따라서 최종v4에서는 확인된 지면 점의 국소2.5D 연결을 재구성했다. v2/v3은 내부 비교용 중간 결과로 보존했고 기본으로 사용하지 않는다.

v4는 원본v1을 별도로 읽어 초록 지면만 normal-only smoothing12회(최대6.21mm; 상한8mm) 후 색 변화/face collapse guard를 적용했다. 지면 근거가 있는8cm 이내 닫힌 구멍69개를 보완한 뒤, 지면-only face를 국소XY Delaunay sheet로 대체한다. 최대3D edge6cm,5mm XY 셀의 실제 관측 median 대표점, 관측된 지면인 triangle centre/edge midpoint만 허용하며 near-vertical triangle은 제외한다. 장애물·미분류 위치 및 색, 비지면-only triangle은 보존한다. 외곽/미관측/큰 공백을 무제한 채우는 전체 평면 mesh는 만들지 않는다. class seam에서 nonmanifold를 만들 후보1,986 faces를 제외했고 최종 nonmanifold edge0이다.

초록210,933 vertices의 음영 법선은 local terrain slope로 안정화한다. 표시의 ambient0.35 / light당 diffuse0.35 / specular0.04로 설정했다. ground sheet는 추론된 연결이며 새로운 깊이 관측 또는 주행 가능성 근거가 아니다. 구멍이 완전히 사라진 지도는 아니며 큰 미관측 영역은 유지한다.

기존 명령으로 새 결과가 열린다:

```bash
bash tools/view_rtabmap.sh
# 첫 색상 보정 결과와 비교
bash tools/view_rtabmap.sh outputs/handheld_mapping/comparisons/rtab_rgbd_20261016_174639_full/surface_refined_v1
```

검증: 지면 법선의 장애물/입력 보존, 미관측 마스크·연결 길이·nonmanifold 방지 회귀 추가. 관련15개 테스트 통과 후 topology guard 변경에 대한 신규2개 재검사 통과. native screenshot을 같은 viewpoint로 생성·시각 비교하여 지면 내부 검은 줄무늬 감소를 확인했다. 비교 이미지는v4의 `before_view.png` / `after_view.png`다. source mesh checksum과 비지면 좌표·색 불변 조건을 검사했다.
