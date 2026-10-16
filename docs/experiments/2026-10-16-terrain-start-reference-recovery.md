# 촬영 시작 정지 구간으로 지면 색상 복구

`20261016_174639_b1d8ed` 기록은 기본 stereo TSDF가 완성됐지만, 고정 중력 기준이 없어 지면/장애물 분류가 생성되지 않았다. 첫 keyframe 다음 keyframe은 약10.1초 뒤에 있었다. 첫 keyframe의 전후1초를 보는 검사가 촬영 직전 움직임을 포함해 거부됐고, 시작 직후의 충분한 정지는 검사하지 못했다.

offline `derive_alignment`에 `stationary_anchor_reference`를 도입했다. 먼저 기존 가운데2초를 검사하고, 실패하면 pose부터 이후2초 또는 pose까지 이전2초를 검사한다. 창은 반드시 해당 pose에 닿아 있어야 한다. 최소 표본·2초 범위·최대 공백120ms·단일 MCU segment·중력 크기·가속도 변동·gyro 상한은 기존 `stationary_reference` 기준을 유지한다. 멀리 떨어진 정지 구간이나 움직이는 pose의 중력 기준을 가져오지 않는다. 촬영 중 live 처리와 IMU 융합은 바꾸지 않는다.

실제 첫 keyframe에서 이후2초의99표본을 사용했다. 평균 가속도 크기1.00308g, 최대 평균 대비 가속도 차0.00393g, 최대gyro0.21751°/s로 기존 기준을 만족한다. 새 metadata에 `window_placement=forward`, anchor와 window center Pi 시각을 기록한다. 이는 정지 중력 정렬이며 카메라/IMU acquisition 시각 동기화 또는 관성 융합 성공을 뜻하지 않는다.

기존 raw·calibration·최종 ORB pose·기본 TSDF를 보존하고, 새 중력 기준 및 별도의2m surface cleanup revision으로 지형 분류를 다시 생성한다. viewer의 기존 표면 높이 분류, 장애물 표시 평활화, 작은 지면 구멍 보완을 사용한다. 과거 촬영 상태의 실패 메시지는 원래 `session.json`에 남겨두며 재처리 결과는 별도 파일에 저장한다.

```bash
bash tools/view_mapping.sh outputs/handheld_mapping/sessions/20261016_174639_b1d8ed --backend orb_offline --depth-max 2 --surface-cleanup
```

알려진 시작/종료 경계 정지창의 복구와, pose 시점 움직임·가속·시각 공백·MCU reset·짧은 창·분리된 정지 구간의 거부를 검사한다. 실제 시작 keyframe 경로를 통한 복구와 pose/IMU 파일 보존도 검사한다. 실행 결과와 checksum 검토는 `outputs/handheld_mapping/reviews/20261016_174639_b1d8ed_terrain_recovery/`에 저장한다.
