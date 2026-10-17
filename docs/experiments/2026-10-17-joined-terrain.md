# 원래 윤곽을 살린 근접 조각 연결 지형

사각 box20개가 지나치게 단순하다는 요청에 따라 원본 classified source의 XY 윤곽과 높이를2cm 격자로 복원하고 가까운 작은 틈만 연결했다. 결과는 `outputs/handheld_mapping/exports/orb_rtab_joined_terrain_v2`이며 기존 box STL과 원본은 보존한다.

입력: `orb_rtab_fusion_20261016_174639/surface_refined_v4`의 실제 classified cloud. 주 바닥 plane으로 동일하게 rigid 정렬한다. 지면은 셀별 median, 장애물은85% height quantile을 사용한다. footprint를 rectangle로 감싸지 않는다. 각 장애물 셀에는 원본2점 이상/지면보다4.5cm 이상을 요구한다.

XY closing의 최대 짧은 틈8cm(반경4cm disk)를 사용하고, 인근 관측 top 높이 차이도8cm 이하일 때만 추가 셀을 허용한다. 단순 XY closing으로4/8/12cm를 조사했을 때 mask components40→36/32/30, 추가229/514/806cells였고 중간8cm를 선택했다. 실제 최종에서는 height agreement와 tiny group 필터까지 적용하여41→35 components,106cells(약0.0424m²) 연결,6개 tiny observed cells 제거다. 이 component는 높이를 floor까지 채우기 전 obstacle footprint 개수이며 최종 closed solid는 바닥을 통해1 shell로 연결된다.

지면은 원본 높이와 국소 경사를 유지하며 sigma3cm smoothing. 미관측 base로 작은 높은 ground fragment가 퍼지지 않도록 ground 관측에서4cm 이내는 유지,4–12cm는 fitted floor로 fade,12cm 밖은 nominal plane이다. obstacle top의3×3 closing/median 및 height-aware local smoothing은2cm comb 노이즈를 줄인다. 전체 box fitting은 하지 않는다.

top2.5D heightfield / perimeter wall / bottom fan으로 닫힌 volume을 생성하고 manifold3d simplify로 근사한다. 검증에 실패하면 tolerance를 줄이며 실제 최종은2mm.31,092 triangles,watertight,self-intersection0,1 connected shell. topography의 undercut/overhang와 가려진 underside는 복원하지 않고 바닥까지 채운다. 연결된 작은 틈은 추정이지 새 관측이 아니다. 직사각형 base는 미관측 영역을 포함하는 모델링용 지지판이다.

```bash
cd ~/rover-mapping
bash tools/view_joined_terrain.sh
```

초록 지면/주황 장애물. 파랑 camera 경로·초록 출발·빨강 도착을 floor frame에 맞춰 표시. O 키로 source cloud overlay. STL은 `terrain_m.stl` / `terrain_mm.stl`; STL 자체에는 단위가 없고 파일명에 맞는 수치를 저장했다. 원본 cloud checksum을 보존하고 heightfield.npz에 관측/추정 연결 mask를 남겼다. native_view.png는 실제 뷰어 검증 이미지다.

재생성(새 output 필요):

```bash
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv-mapping/bin/python -m experiments.handheld_mapping.joined_terrain build \
  outputs/handheld_mapping/comparisons/orb_rtab_fusion_20261016_174639/surface_refined_v4 \
  outputs/handheld_mapping/exports/joined_terrain_new
```

검증3 tests:6cm gap연결/14cm gap보존,높이20cm 차이에서 연결 거부,경사+column heightfield의 watertight/단일shell/좌표보존. 실제 STL m/mm reimport watertight/self-intersection/1000배scale 검사 통과. native snapshot 시각 확인. v1은 내부 비교용 결과, v2가 기본 최종 결과다.
