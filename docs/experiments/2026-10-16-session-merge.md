# 저장된 촬영들의 영상 검증과 원본 깊이 통합

기준 촬영은 `20261016_174639_b1d8ed`의 기존 stereo 지도다. 개별 mesh를 이어 붙이지 않고, 촬영 간 좌표를 영상/깊이로 검증한 다음 각 촬영의 원본 Z16과 최종 keyframe pose를 같은 TSDF에 다시 융합했다. 원본 세션은 수정하지 않고 `outputs/handheld_mapping/merged/`에 별도 결과를 저장했다. 이 시험은 IMU 관성 융합이 아니다.

## 정렬과 검사

첫 점군 FPFH/RANSAC 후보는 평평한 바닥 때문에 seed에 따라 위치가 크게 달라져 병합 근거로 충분하지 않았다. 후보는 비슷한 시점의 영상 쌍을 찾는 데만 사용했다. 실제 상대 좌표는 다음으로 결정했다.

1. 보존된 좌측 IR 영상의 RootSIFT 특징을 양방향 ratio test로 대응.
2. 해당 촬영의 depth 보정값·좌측 카메라 TF·최종 pose로 특징의 metric 3D 좌표 계산.
3. PnP RANSAC, 영상 내 특징 분포와 중력 방향 검사.
4. 여러 source 영상의 일관된 변환을 선택하고 강건한 재투영 최적화 수행.
5. 미리 분리한 source 영상은 consensus 및 최적화에 사용하지 않고 별도로 검사. 동일 source 이미지의 다른 target 대응도 train/held-out 사이에 섞지 않음.

기준은 최소 train source 영상4개, held-out source 영상2개, 카메라 위치 지지 범위0.5m, held-out 오차 중앙값3px·90백분위6px 이하, 중력 방향 차3° 이하다. 변환은 rigid SE3이며 scale은 바꾸지 않는다. 정확도를 GT로 검증했다는 의미는 아니다.

| 추가 촬영 | train source 영상 | held-out 영상 | held-out 중앙값 / p90 | 처리 |
|---|---:|---:|---:|---|
| `20261015_152413_6fea4a` | 14 | 8 | 0.966 / 3.372 px | 병합 |
| `20261016_164429_1e50c4` | 8 | 4 | 0.461 / 0.879 px | 병합 |
| `20261016_153305_253eb4` | — | — | 일관성 검사 실패 | 제외 |

등록 보고서와 영상 쌍의 인덱스는 `outputs/handheld_mapping/reviews/session_merge_trial/*_validated.json`에 보존했다. 실패한 조합을 수동 강제 변환으로 포함하지 않았다. 나머지 예전 촬영들까지 모두 병합했다고 주장하지 않는다.

## 원본 융합과 결과

촬영마다 독립적으로 같은2m surface-cleanup depth 필터를 적용하므로 다른 촬영의 프레임을 시간상 이웃으로 취급하지 않는다. 검증한 상대 변환과 촬영별 고정 중력 변환으로 pose를 옮긴 뒤1cm TSDF에 융합했다. 지면 층과 장애물 층도 같은 필터·공통 좌표에서 다시 누적했다. viewer의 주황 평활화와 작은 초록 구멍 보완도 사용할 수 있다.

서로 다른 촬영의 경로는 별도 그룹으로 표시한다. 촬영 종료점과 다음 촬영 시작점을 직선으로 연결하지 않는다.

| 결과 | 사용한 유효 keyframe | 정점 / 삼각형 | 관측 XY 셀(5cm) |
|---|---:|---:|---:|
| 최신 촬영 단독 | 712 | 412,067 / 728,704 | 10,409 |
| 두 촬영 | 964 | 493,703 / 872,745 | 10,973 |
| 세 촬영 | 1,297 | 505,742 / 891,669 | 11,124 |

세 촬영 결과는 최신 촬영에 없던715개의 관측 XY 셀이 추가됐다(약6.9%). 이 수치는 바닥과 장애물의 관측을 함께 포함하며, 검증된 주행 가능 면적이나 지도의 정확도 증가율은 아니다. 오래된 `20261015_152413_6fea4a`의364개 keyframe 중 원래 depth가 매칭되지 않는112개는 사용하지 않았다. 따라서 `merge.json`의 status는 `partial`이며 유효 관측의 지도·분류는 생성됐다. 없는 원본을 만들어 채우지 않았다.

```bash
# 두 촬영 결과
bash tools/view_mapping.sh outputs/handheld_mapping/merged/two_sessions_2m --backend orb_offline

# 세 촬영 결과: 이미2m·surface cleanup·지면/장애물 분류로 생성됨
bash tools/view_mapping.sh outputs/handheld_mapping/merged/three_sessions_2m --backend orb_offline
```

거리 설정을 바꾸려면 원본 촬영들과 검증 보고서로 병합을 새 폴더에 다시 실행한다.

```bash
bash tools/merge_mapping.sh \
  --reference outputs/handheld_mapping/sessions/20261016_174639_b1d8ed \
  --registration outputs/handheld_mapping/reviews/session_merge_trial/20261015_152413_6fea4a_validated.json \
  --registration outputs/handheld_mapping/reviews/session_merge_trial/20261016_164429_1e50c4_validated.json \
  --output outputs/handheld_mapping/merged/NEW_RESULT \
  --depth-max 2
```

`session_registration.SessionImages`와 `visual_registration`이 정렬 보고서를 만들고, `merge_mapping.sh`는 보고서의 품질 항목과 입력 hash를 검사한 뒤 결과를 생성한다. 단순 점군 정렬 후보 JSON은 받지 않는다. 입력 calibration/index/pose/gravity와 사용한 모든 depth의 hash를 저장한다.

## 検証と限界

알려진 metric rigid 변환 복원, held-out 영상 불일치 거부, 카메라 이동 범위 부족 거부, 실제 복수 TSDF 입력과 두 촬영 pose 변환, 원본/기존 출력 보존, 반사·scale 변경·오래된 입력 hash 거부, 경로 그룹 분리를 검사했다. 전체 회귀470 passed,14 skipped. `git diff --check`와 shell 구문 검사도 통과했다.

실제 mesh 표시 층을 읽어 세 결과 모두에서 초록/주황 분류·표시 평활화·작은 구멍 보완과 개별 경로를 확인했다. 표시 처리 전후 mesh hash와 사용한 원본 depth 및 입력 hash가 동일했다. 비교 그림과 자세한 결과는 `outputs/handheld_mapping/reviews/session_merge_trial/{comparison.png,viewer_validation.json}`에 있다.

이 방법은 촬영 간 하나의 rigid 변환을 사용한다. 촬영 내부의 비선형 드리프트, 움직인 물체, 전역 오대응을 완전히 해결하지는 않는다. 겹침이 약한 영역에는 미분류와 노이즈가 남을 수 있다. GT 정확도·측량 정확도·로버의 안전한 주행 가능성은 미검증 상태를 유지한다.
