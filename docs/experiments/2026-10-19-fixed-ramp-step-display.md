# 고정 경사로 사이 단차 표시

사진에서 지정한 10°·30° 경사로 사이 검은 구간을 **경사로 높이 차이에 따른 단차**로 표시한다.
주황색은 기존 장애물 표면과 함께 단차 측면에도 사용한다. 추가 면의 의미는
`ramp_step`이며 별도의 장애물 부피나 통행 금지 구역을 만들지 않는다.

## 현재 적용 결과

`outputs/handheld_mapping/merged/slope_angle_assumed_20261018`의 세 배치 후보에 적용했다.
기존 가정 정렬은 그대로 사용하며 원래 상대 배치가 검증된 것으로 바꾸지 않는다.
기존 `view.py` 진입점은 새 단차 뷰어를 사용하도록 갱신했다.

```bash
# 저장소 루트에서 현재 결과 보기
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv-mapping/bin/python -m experiments.handheld_mapping.slope_step_viewer \
  outputs/handheld_mapping/merged/slope_angle_assumed_20261018
```

- 기본: 주황색 단차 표시 켜짐.
- S: 단차 표시 켜기/끄기. 끄면 원래 관측 지도.
- F: 전체 문맥/분류된 표면 전환. 두 모드 모두 단차 표시 지원.
- 1/2/3: 가정 배치 후보 전환.

## 경사면 빈틈·요철 표시 보정 (후속 요청)

`slope_polish.py`에서 동일하게 선택한 두 경사로의 관측 셀 중심 convex hull 안을
각각 적합 평면으로 채운다. 경사로 내부의 빈 구간을 메우며 표면 요철을 제거한다.
단차와 같은 평면 계수를 사용하므로 주황색 단차의 양쪽 높이와 연결된다.
기존 삼각형은 세 꼭짓점 모두 해당 hull 안에 있고 평면과 6cm 이내일 때 교체하며,
주황색 정점이 포함된 장애물 면은 보존한다. 외부 지형도 유지한다.

이는 선택된 고정 경사로가 연속된 평면이라는 사용자 요청 기반 표시 가정이다.
convex hull 내부의 미관측 구간과 오목한 가장자리도 평면으로 채워질 수 있으며,
관측 높이·통행 비용·원본 mesh 및 정렬은 변경하지 않는다.

- 기본 표시: `hypothesis_N_mesh_polished_with_step.ply`.
- F 문맥 표시: `hypothesis_N_full_context_polished_with_step.ply`.
- R: 경사면 보정 전후 전환. 주황색 단차는 유지한다.
- S: 보정 및 단차를 함께 끄고 원래 관측 표면과 비교한다.
- `hypothesis_N_polished_step_preview.png`: 보정 결과 실제 Open3D 렌더링.
- `ramp_steps.json`의 `ramp_polish`: 각 평면, 보정 영역 면적, 교체 면 수, 추정 표시 명시.

세 배치 및 전체 문맥 파일을 재생성하고 Open3D에서 미리보기 3개를 렌더링했다.
추가 검증은 구멍 전체 면적 충전, 평면 높이, 장애물 좌표 보존, 원본 불변을 확인한다.
기존 28개와 새 2개 테스트가 통과했다.

## 생성 방식과 범위

기준 지도에서 5~20°인 최대 연결 경사 구간, 추가 지도에서 25~35°인 최대 연결 구간을
선택한다. 각 지형 셀을 원래 mesh 좌표로 복원하고 기존 정렬 행렬을 적용한다.
두 평면의 맞은편 경계를 공유 오르막 범위 안에서 2cm 이하 간격으로 샘플링한다.
경계 사이 거리 35cm 이하인 구간만 공통 경계선까지 짧게 연장하고,
그 선의 양쪽 높이를 잇는 **수직 단차 면**을 만든다. 이 모델 면은 모두 주황색이다.
평면 적합 RMS가 2.5cm를 넘거나 공유 범위가 없으면 생성하지 않는다.

이는 사용자가 확인한 고정 경사로 연결 관계를 반영하는 표시 모델이다.
관측 경계의 convex hull과 적합 평면을 사용하므로 실제 시공 경계를 실측한 결과는 아니다.
관측 원본, 높이·경사 raster, SLAM pose, 통행 비용은 변경하지 않는다.
다른 지도나 새 촬영에 이 좌표를 자동 복사하지 않는다. 새 촬영의 자동 적용에는
해당 촬영과 고정 지도 사이의 좌표 정합이 별도로 필요하다.

출력:

- `hypothesis_N_step_riser.ply`: 추가된 단차 면만 저장.
- `hypothesis_N_mesh_with_step.ply`: 기존 분류 표면 + 단차.
- `hypothesis_N_full_context_with_step.ply`: 기존 전체 문맥 + 단차.
- `hypothesis_N_step_preview.png`: 실제 Open3D 렌더링.
- `ramp_steps.json`: 의미·추정 여부·평면·높이 차 범위·변환·입력 SHA-256.

원래 `hypothesis_N_mesh.ply` 등 측정 기반 파일은 그대로 보존한다.
뷰어는 정렬 파일과 원래 표면의 SHA-256이 달라지면 오래된 단차를 표시하지 않는다.
지도 재생성 후에는 다음 명령으로 단차를 다시 만든다.

```bash
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv-mapping/bin/python -m experiments.handheld_mapping.slope_step \
  outputs/handheld_mapping/merged/slope_angle_assumed_20261018
```

## 검증

단차 기하·좌표 회전·원본 보존·넓은 틈 거부·불안정 평면 거부와 기존 경사 표시·색상
회귀 합계 **28 passed**. 세 실제 배치의 추가 면 및 전체/문맥 결합 파일을 생성하고
실제 Open3D에서 세 배치의 렌더링을 확인했다. 원본 파일 SHA-256도 일치한다.
