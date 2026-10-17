# 외곽선·평면 기반 지도형 STL 시안

최신 경사 RUN의 동일 2m cleanup 입력을 사용한다. 결과는
`outputs/handheld_mapping/exports/latest_structured_map_v3`이며 원본과 이전 결과는 보존했다.

- 지면: 기존 중력 정렬 및 약 9.96°/2.75° 평면을 유지한다. 관측 지면의 2.5cm
  support cells를 polygon union하고 반경 5.5cm closing과 2.5cm contour simplify를 적용한다.
  큰 미관측 내부 구멍은 남긴다. 지면 관측에서 XY 16cm 이내인 장애물 아래쪽 일부는
  지면으로 추정해 연결하며 추정/제거 면적을 summary.json에 저장한다.
- 평면 접합선을 polygon split 제약으로 넣고 Shapely 2.1.2의 constrained Delaunay로
  삼각분할한다. 내부 격자를 그대로 STL 삼각형으로 남기지 않는다.
- 장애물: 3D density cluster별로 1.2cm RANSAC 및 normal agreement로 지지되는
  plane patch를 추출한다. local 2D contour와 제한된 틈 연결로 실제 관측 윤곽을 살린다.
  비평면 잔여는 작은 조각을 제거하고 QEM으로 줄인다. 모든 물체를 상자로 만들지 않는다.
- 평지/경사 색은 삼각형별로 분리해 큰 면에서 초록/핑크가 번지는 것을 막는다.
- 출발점 아래 평면 원점으로 모델·경로를 같은 translation으로 이동한다. 원본 중력축은 유지.

지면 top 약 8,270 → 72 triangles, 장애물 약 35,140 → 3,137 triangles.
지지되는 장애물 plane patch 6개. 이는 실제 물체 개수를 뜻하지 않는다.
미관측 물체 뒷면과 내부는 복원하지 않은 시안이다.

검증: synthetic concave boundary/large-hole preservation, plane junction triangles의
planarity/XY area 보존, 기울어진 obstacle plane의 단순화·방향 유지 검증.
planar terrain 회귀 포함 6 tests 통과. source SHA256 보존.
실제 binary STL 좌표 및 재로딩 topology 검사 결과를 stl_reload에 저장한다.
기존처럼 메모리상의 mesh 검사만으로 파일 watertight 성공을 선언하지 않는다.
현재 ground_solid_mm.stl은 재로딩 후 watertight/self-intersection 없음 통과.
metre 버전은 동일 topology지만 재로딩 self-intersection 검사에서 차이가 발생하므로
그 파일을 검증 완료한 출력용 solid로 간주하지 않는다.
map_m/mm.stl은 열린 장애물 표면을 포함해 닫힌 출력용 assembly가 아니다.

보기/원본 비교:

```bash
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
 .venv-mapping/bin/python -m experiments.handheld_mapping.structured_terrain \
 outputs/handheld_mapping/exports/latest_structured_map_v3 --view
```

O 키로 원본 source를 같은 원점에서 겹친다. 회전/확대/이동은 기존 Open3D 조작과 같다.
현재 앱 기본 내보내기는 이전 평면 보정 방식이며 이 결과는 별도 지도형 시안이다.

추가 실험: 각 장애물 관측 삼각형에 대칭 1cm 두께를 주고 manifold Boolean union.
단순 surface 3,429 faces에서 약 17만 faces로 증가해 지도 단순화 목적에 맞지 않았다.
전체 STL self-intersection 검사는 과도한 처리시간으로 중단했다. 결과는
`experimental_shell/`에 미검증으로 분리했으며 기본 시안에는 사용하지 않는다.
