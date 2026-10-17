# 최신 경사 촬영 STL 분석

대상 RUN: 20261017_231613_d7b716. 분석만 수행하며 기존 산출물은 보존한다.

## 실제 확인

- joined 출력: exports/slope_joined_app_check_20261017/Map_0/joined.
- floor_from_source 회전은 중력 Z축 대비 9.557°. ground 전체에서 최대 RANSAC 평면을 고르므로 실제 경사면을 기준면으로 선택했다.
- 원본 XYZ ROI 평면 재측정(1cm 미만 잔차, y ±35cm): 평지 x 0.4–1.1m는 입력 2.764° → 출력 좌표 8.513°. 경사면 x 1.5–2.5m는 입력 9.859° → 출력 좌표 0.468°. rigid 회전이라 두 면 사이 각도는 보존하지만 중력 기준 자세가 훼손된다.
- heightfield ground의 중앙차분 기울기: median 7.11°, p90 41.78°, p95 54.53°, p99 68.42°. 31,872 셀 중 5,189 셀이 30° 초과. 전체 셀에는 추정 지면/외곽도 포함하므로 실제 주행 지면의 경사 통계로 해석하면 안 된다.
- 추가 연결 셀은 118개(0.0472m²). 전체 사각 격자는 12.7488m². 장애물 footprint 1.192m². 단순한 연결 개수만으로 전체 형상 오차를 설명할 수 없다.
- joined는 watertight/단일 shell/self-intersection 없음이나, 이는 원본 형상 정확도를 보장하지 않는다.
- 일반 refined STL은 220개 구멍을 메웠지만 boundary edge 13,268개가 남아 닫힌 solid가 아니다.
- 비교 입력도 달랐다: joined는 기본 depth 3m, latest_slope_pink는 depth 2m + cleanup. 동일 입력/좌표/시점 비교가 필요하다.

## 코드상 원인

1. joined_terrain.build의 dominant plane 정렬: 중력 대신 가장 큰 관측 평면을 수평으로 만듦.
2. 지면 nearest-neighbour extension 후 관측 4–12cm 밖에서 z=0으로 fade: 높은 지면/잘못 선택한 기준면의 경계에 급경사 생성.
3. XY당 단일 높이 + 바닥까지 volume 채움: 수직면/얇은 조각/중첩면을 실제와 다른 solid로 만듦.
4. 장애물 최소 두 점은 독립된 두 프레임 관측 조건이 아님. TSDF vertices 밀도가 관측 신뢰도를 대신하지 못함.
5. XY closing + 높이 span만 검사하며 표면 법선/가시성/독립 관측/단차 경계를 검사하지 않음.
6. 기존 테스트는 gap/height mismatch/topology 중심. 중력 자세 보존, 평지+경사 공존, 관측 경계 fade의 형상 검증이 없음.

## 권장 변경 순서

- 중력 정렬을 유지하고 Z translation만 허용. 임의의 dominant-plane 회전 제거.
- 같은 2m cleanup 원본으로 비교를 고정.
- 평지/경사/단차를 분리한 국소 robust plane 또는 piecewise terrain fitting. 경사 접합부를 보존.
- 미관측 영역은 기본적으로 비워 두고 작은 닫힌 구멍만 표면 연속성 근거로 보간. z=0 fade 제거.
- 지면은 terrain mesh, 물체는 기존 3D TSDF 표면으로 따로 처리. 모든 물체를 단일 heightfield로 압출하지 않음.
- 출력용 닫힌 부피는 사용자 선택 ROI의 외곽에 명시적으로 측벽/바닥을 추가. 관측 구멍을 전부 억지로 채워 하나의 solid로 만드는 것을 성공 기준으로 사용하지 않음.
- 검증은 경사 보존, 높이/점-메시 거리 오차, 관측/추정 면적, 경계 유지 + topology를 함께 측정.

외부 참고: Open3D surface reconstruction 공식 문서는 Poisson이 저밀도/미관측 영역에도 삼각형을 생성하므로 density 기반 pruning을 설명한다. 따라서 Poisson 전역 적용만으로 신뢰할 만한 지형 복원이 되지는 않는다.
https://www.open3d.org/html/tutorial/geometry/surface_reconstruction.html
