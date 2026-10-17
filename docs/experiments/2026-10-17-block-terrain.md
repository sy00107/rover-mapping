# 결합 지도의 사각 장애물 지형 STL

사용자 요청에 따라 물체 종류나 책상 다리 의미를 구분하지 않고, 관측된 장애물을 모두 사각 기둥/직육면체로 가정한 근사 모델을 만들었다. 원본은 `orb_rtab_fusion_20261016_174639/surface_refined_v4`이며 결과는 `outputs/handheld_mapping/exports/orb_rtab_block_terrain_v3`다. v1/v2는 내부 생성 시도 결과이며 완성본은v3이다.

초록 지면의3cm downsample 점에서2.5cm RANSAC plane을 맞춰94.6% support의 주 바닥을 찾았다. 전체 geometry를 이 바닥의 normal 기준으로 회전·이동한다. 바닥판은 약7.46×5.57m / 두께0.10m의 근사 rectangle이다. 큰 별도 경사 구간이 확인되지 않아 이번 모델에는 임의 경사면을 추가하지 않았다.

주황 관측은3cm voxel / DBSCAN eps10cm / 최소5점으로 묶고60점 이상 cluster 중7cm 이상 높이인 것만 사용했다. retained20개에2D minimum-area oriented rectangle을 맞추고 각 축2–98% extent 및 높이97% quantile로 크기를 근사했다. 최소폭6cm. 모든 장애물은 바닥까지 닿는 직육면체로 채워졌으며, 가려진 뒤쪽과 내부는 관측값이 아니라 명시적으로 추정한 부피다. 작은 분리 관측·미분류는 모델링하지 않는다. 근접한 실제 여러 물체가 하나의 cluster/box가 될 수 있다.

manifold3d3.5.4의 robust Boolean union으로 바닥판과 장애물을 단일 solid로 합쳤다. Open3D 원본 mesh Boolean은 작은 box 교차 시험에서 topology 검증에 실패해 최종 출력에 사용하지 않았다. 최종348 triangles,1 connected shell,watertight, self-intersection0. `terrain_m.stl`은 metre 수치, `terrain_mm.stl`은1000배 millimetre 수치다. STL 자체에 단위 정보는 없으며 출력시 원하는 축척을 적용한다. 각 primitive의 metre STL도 `parts/`에 남겼다. `summary.json`에는 모든 box 위치/크기/yaw와 source checksum이 있어 수정·재생성할 수 있다.

보기:

```bash
cd ~/rover-mapping
bash tools/view_block_terrain.sh
```

초록 바닥판, 주황 사각 장애물. O 키로 같은 floor frame으로 변환한 원본 점군을 겹친다. drag 회전 / wheel 확대 / Shift+drag 이동. 실제 native render는 `native_view.png`다.

새 폴더로 재생성:

```bash
.venv-mapping/bin/python -m pip install -r experiments/handheld_mapping/requirements-block-terrain.txt
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv-mapping/bin/python -m experiments.handheld_mapping.block_terrain build \
  outputs/handheld_mapping/comparisons/orb_rtab_fusion_20261016_174639/surface_refined_v4 \
  outputs/handheld_mapping/exports/block_terrain_new
```

검증: floor transform의 SE3/planeZ=0, 긴 rectangle의 방향 보존, 회전·중첩 box와 바닥의 단일 watertight union3개 테스트 통과. 실제 STL binary 좌표 검증 및 m/mm STL 재로딩의 watertight / self-intersection / 축척 확인. 원본 cloud checksum 불변 확인. native rendering 시각 확인. 이 모델은 정밀 측량/실제 obstacle 단면의 복원이 아니라 요청한 사각 장애물 지형 시안이다.

블록 지형 뷰어에도 파란 camera 경로, 초록 출발구, 빨강 도착구를 기본 표시한다. source camera pose에 기존 지도 Z-up 변환을 적용한 뒤 단순화 모델의 floor_from_source를 추가 적용한다. 시각 표시만 추가하며 terrain STL 부피에는 경로나 마커를 넣지 않는다. `--no-trajectory`로 숨길 수 있다. `native_view_endpoints.png`에서 실제 렌더링을 확인했다.
