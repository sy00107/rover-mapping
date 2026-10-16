# 새 촬영의 host clock 복구와 관성 지도 재생

대상: `outputs/handheld_mapping/sessions/20261016_164429_1e50c4`.

원래 자동 후처리는 MCU→Pi MONOTONIC affine 잔차의 8구간 중앙값 범위21.119ms가 기존20ms 제한을 넘어 중단됐다. IMU 고유 정상 표본9416개가 있었으나 IMU의 RAW host clock은 저장되지 않았다. 카메라에는2815개의 MONOTONIC·RAW·realtime callback pair가 있었다.

## 적용과 결과

`inertial_timing.aligned_inputs(..., recover_camera_clock=True)`와 CLI `--recover-camera-clock`을 추가했다. 카메라 callback clock pair를 이용해 같은 Pi host의 MONOTONIC→RAW 변환을 보간한다. camera pair 범위 밖444개 IMU 표본은 버리고 외삽하지 않았다. 원본 표본/타임스탬프 파일은 수정하지 않았다.

- camera pair 최대 간격534.520ms, 최대 조회 bracket17.257µs.
- 홀수 pair를 제외한 보간 교차 검사 최대 오차6.783µs. **이는 host clock 변환의 내부 일관성 수치이며 센서 동기화 정확도가 아니다.**
- 복구 후 affine 구간 중앙값 범위19.018ms. 기존20ms 기준을 유지했다.
- 영상 회전/gyro 상관0.95927. IMU 시각에 적용한 보정−40ms; 앞/뒤 절반의 query shift+40/+35ms.
- 실제 native stereo-inertial 재생2813쌍, 입력 부족 skip0, 최종 초기화된 map404, `orb_fused=true`.
- native 초기화/재시작이 초반에 반복돼 최종 export는90개 keyframe, 촬영 시작 약114.8–188.2초의 약73.4초 구간이다. 전체 촬영을 포함한 융합 지도라고 해석하지 않는다.
- map404의2m TSDF, 원거리 표면 정리, 중력 정렬, 지면/장애물 분류를 별도로 생성했다. keyframe90개 모두 융합, 미융합0; 정점96,194개·삼각형172,419개.

```bash
bash tools/view_mapping.sh outputs/handheld_mapping/sessions/20261016_164429_1e50c4 --backend orb_inertial
```

이 결과는 이미2m·surface cleanup으로 생성돼 있다. 기존 viewer의 주황 장애물 표시 평활화와 작은 초록 지면 구멍 보완도 사용한다.

`imu_clock_verified=false`, `extrinsic_verified=false`, `noise_verified=false`를 유지한다. 이 시험은 초기화 성공을 확인한 것이며 기존 지도보다 정확해졌다는 실측 증거는 아니다. 촬영 당시 `session.json`의 실패 기록은 보존했고 재시도 성공은 `orb_inertial/summary.json` 및 `dense_inertial/summary.json`에 저장했다.

## 다음 기록과 검증

`ksrc_telemetry_tee.patch`와 배포용 `telemetry_tee.py`에 IMU receipt realtime·MONOTONIC·RAW 조회 bracket을 추가했다. 다음 `mapping_imu_bridge.sh` 실행부터 적용된다. 현재 실행 중인 Pi 중계·모터·MCU firmware는 조작하지 않았다. 직접 측정한 RAW가 완전하면 보간 없이 기존 경로를 사용한다. 복구 옵션은 기본 꺼짐이다.

알려진 시계 기준과 복구값 비교, 외삽 금지, 부분 RAW·잘못된 clock pair·긴 공백·조회 지연·step·다른 clock 관계·비선형 MCU 잔차 거부를 검사했다. 전체 회귀451 passed,14 skipped. bridge 명령/stale/종료 명령 동등성 검사도 통과했다. `git diff --check`, shell 구문 검사도 통과했다.

코드 변경 전 사본, 원본과 기존 결과19파일의 SHA-256, 생성 지도/뷰어 검증은 `outputs/handheld_mapping/reviews/20261016_164429_1e50c4_clock_recovery/`에 보존한다.
