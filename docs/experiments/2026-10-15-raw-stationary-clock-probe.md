# 정지 상태 RAW clock 실기

2026-10-15. 이전 MCU→Pi 모델의 시차 변화에서 Pi 시각 기준의 영향을 분리하기 위해 약125초 동안 카메라/IMU를 정지 상태로 읽었다. source와 결과는 노트북 `outputs/handheld_mapping/20261015_raw_stationary_probe`에 보존했다.

## 수집 경로

- 카메라는 기존 IRleft/right+depth640×48015FPS, emitter OFF/color OFF 조건이다. 영상 화소는 저장하지 않고 header와 RealSense metadata만 노트북으로 전송했다.
- IMU는 기존 decoder의 `exchange(..., poll_only=True)`만 사용했다. UDP bridge·모터 속도 명령은 사용하지 않았다.
- production `pi_worker.receipt_clock` 함수를 수집기에 삽입해 같은 monotonic bracket 안에서 realtime과 MONOTONIC_RAW를 읽었다. IMU·camera 양쪽에 동일한 함수가 들어갔다.
- Pi에는 수집 파일을 저장하지 않았고 ROS 로그는 RAM에만 생성했다. 카메라는140초 timeout으로 종료했고 SPI도 해제했다.
- 원본 두 JSONL과 calibration의 SHA256을 분석 전후 확인했다. calibration은 변경하지 않았다.

## 관측 결과

| 항목 | 결과 |
|---|---|
| IMU POLL | 6229회, startup1개 제외, 유효 고유4932개/중복1296개 |
| IMU 무효/CRC decode 실패 | 0 / 0 |
| 고유 수신 속도 | 약39.46Hz |
| camera image/metadata | 각stream1878개씩 |
| camera clock domain | global_time |
| IMU 쪽 MONOTONIC−RAW 변화 | 125.037초 동안 -40.959ms |
| camera 쪽 MONOTONIC−RAW 변화 | 124.995초 동안 -39.099ms |
| IMU clock pair의 평균 속도 차이 | 약-302.20ppm |
| 속도 차이 제거 후8구간 중앙값 범위 | IMU9.818ms / camera9.481ms |
| MCU→Pi MONOTONIC 모델 scale | 0.99399033(차이-6009.67ppm) |
| MCU→Pi RAW 모델 scale | 0.99429003(차이-5709.97ppm) |

camera와 IMU는 시작 시각이 달라 각각의 처음을0으로 잡은 변화량·회귀 속도도 조금 다르다. 같은 RAW 시각에서 비교하면 clock delta 차이 p5/p50/p95는 약-0.315/+0.073/+2.656µs였다. 서로 다른 센서 callback에서도 동일한 Pi clock 변화가 관측된다는 뜻이다. 이는 카메라와 IMU의 물리적 측정 시각이 수µs로 동기화됐다는 뜻이 아니다.

최대 clock bracket span은 IMU23462ns, camera15774ns였다. clock delta 변화 수십ms에 비해 충분히 작다. 노출 p5/p50/p95는2.944/3.252/3.682ms였다.

기존 calibration 적용 후 정지 gyro 평균 XYZ는0.05267/0.06311/0.04142°/s, 표준편차0.03612/0.03461/0.04392°/s였다. 가속도 크기 평균1.00479g였다. 독립 수평·무진동 기준을 측정한 것은 아니다.

## 해석과 한계

1. Pi의 일반 monotonic과 RAW 사이 관계는 이번125초에도 일정하지 않았다. [Linux clock 정의](https://man7.org/linux/man-pages/man3/clock_gettime.3.html)에서 일반 MONOTONIC은 점진적인 시간 보정의 영향을 받고 RAW는 그 보정 영향을 받지 않는다. 현재 Pi는 timesyncd를 사용하며 clocksource는arch_sys_counter였다.
2. -40.959ms 전체가 동기화 오차는 아니다. 상당 부분은 선형 속도 차이로 모델에 흡수된다. 선형 모델을 제거한 뒤에도 구간 중앙값 변화 약9.8ms가 남는다는 점을 구별해야 한다.
3. RAW를 기준으로 해도 MCU 시계는 약0.57%의 상대 속도 차이가 있다. NTP만 고치면 전체 시차 문제가 해결된다고 판단하지 않는다. MCU oscillator·counter·수신 경로와 sensor 내부 clock의 기여를 아직 분리하지 못했다.
4. MCU 모델의 구간별 잔차 범위는 MONOTONIC6.354ms, RAW12.752ms였다. RAW로 바꾸면 항상 더 작은 잔차가 된다는 결과도 아니다. camera timestamp와 MCU timestamp를 RAW 기준에 옮길 때 각각의 변환을 함께 다뤄야 한다.
5. 지금의 기록은 부팅 약50분 이후다. 부팅 직후의 과거 yaw 기록에서 어떤 보정이 일어났는지 소급 확정할 수 없다. timesync-status는 시험 후 snapshot이며 시험 중 보정 이력이 아니다.
6. 정지 기록은 camera–IMU 고정 offset을 추정할 수 없다. -50ms 또는 -55ms를 새로 확정하지 않았고, `imu_clock_verified=false`, `orb_fused=false`를 유지했다.

## 검증과 다음 단계

실기 script는 `.venv-mapping`에서 분석했다. RAW와 일반 monotonic은 정수 ns origin을 뺀 뒤 float로 변환했고, 독립적인100ppm 합성 예제로 회귀 부호를 확인했다. 수집 종료 marker·샘플 단조 증가·startup discard·POLL-only·축 메타데이터·입력 hash를 검사했고 Python pyflakes와 shell 문법 검사가 통과했다. production code는 이번 시험에서 변경하지 않았다.

다음 firmware 보완의 요구는 다음과 같다.

- IMU 평균에 실제로 들어간 표본들의 첫/마지막 MCU 시각과 유효 표본 수를 함께 확보한다. 가능하면 표본 시각의 가중 평균도 보존해 불규칙 간격을 구별한다.
- 제어 loop 초반의 `t_ms`와 측정 표본 시각을 구분한다. sensor 읽기 완료 시각을 저장한다면 acquisition 시각이라고 부르지 않고 지연 상한을 명시한다.
- reset·wrap·평균에 표본이 없는 fallback을 명시적으로 표시한다.
- 기존144바이트 SPI frame과 decoder를 임의로 변경하지 않는다. 남은 reserved 공간과 version 호환성을 검토한 뒤 packet 확장 방식을 정한다.
- 노트북에서 합성 입력·frame pack/decode를 검증한 뒤 실제 firmware 적용을 진행한다. 이번 작업에서는 STM firmware 수정·flash를 수행하지 않았다.

결과 파일: `summary.json`, `raw_clock_comparison.png`, `imu.jsonl`, `camera_metadata.jsonl`, `pi_time_status.txt`, `analyze_raw.py`. 현재는 사용자에게 추가 기울임 동작을 요청할 필요가 없다.
