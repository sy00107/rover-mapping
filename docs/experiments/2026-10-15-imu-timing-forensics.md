# 수동 회전 시차 차이 조사

2026-10-15. yaw RUN `20261015_215301_23dbf4`의 reset 이후segment1과 roll/pitch RUN `20261015_221059_5e303d`를 비교했다. 목표는 -40/-55ms 차이를 분해하는 것이며, 미검증 시각을 ORB에 투입하지 않는다.

## 같은 영상 구간으로 비교

원본 header와 camera metadata가 모두 있는 연속 pose 쌍만 사용했다. yaw1408구간, roll/pitch1577구간이다. camera 시각을 바꿀 때 같은 프레임 간 회전각을 유지하고 새 dt로 각속도를 재계산했다. MCU reset은 가로지르지 않았다.

| IMU 시각 / camera 시각 | yaw 후보 | roll/pitch 후보 |
|---|---:|---:|
| MCU affine / 기존 header | -40ms | -55ms |
| Pi 수신 / 기존 header | -50ms | -50ms |
| MCU affine / 노출 중심 | -45ms | -55ms |
| Pi 수신 / 노출 중심 | -50ms | -55ms |

Pi 수신/header 조건 상관은 yaw0.99775, roll/pitch0.99602였다. 수신 시각으로 바꾸면 두 기록의 후보가 가까워지므로, 전 구간에 하나의 MCU→Pi 직선을 맞추는 과정이 관측된 차이에 기여한다는 증거다. 수신 시각은 측정 시각이 아니므로 -50ms를 실제 camera–IMU offset으로 확정하거나 runtime에 적용하지 않는다.

노출 중심과 기존 시각의 차이는 yaw 중앙값1.606ms, roll/pitch1.730ms다. 이 자료에서 노출 중심 차이만으로15ms의 전체 차이를 설명하지 못한다. 5ms 검색 grid 때문에 표의 후보가5ms 단위로 바뀌며, 이 간격은 정확도가 아니다.

## 시계 모델의 시간에 따른 오차

yaw 수신 잔차의8구간 중앙값 범위는53.689ms로, 구간 중앙값이 약-23→+31→-23ms로 변한다. roll/pitch 범위는9.752ms다. 정상적인 짧은 수신 지터만으로 취급하기 어려운 저주파 변화가 yaw 기록에 남는다. 국소 clock rate도 기록 안에서 변했다.

Pi의 timesyncd journal에는21:52:39 최초 동기화가 있고 yaw 촬영은 그 직후였다. Linux의 [CLOCK_MONOTONIC은 NTP의 점진적 보정 영향을 받고 CLOCK_MONOTONIC_RAW는 받지 않는다](https://man7.org/linux/man-pages/man3/clock_gettime.3.html). 다만 기존 자료에는 RAW 시각·시간 보정량·MCU 개별 측정 시각이 없어, 이 변화를 NTP·MCU oscillator·수신 경로 중 하나로 확정하지 않는다.

Pi checkout의 source상 `PiSpi_HoldIrq`는 EXTI15_10만 막는다. 서보 읽기 동안 모든 IRQ를 막아 SysTick이 멈춘다는 설명은 이 source와 맞지 않는다. MCU firmware는 각 sensor 표본의 시각을 내보내지 않고 제어 주기 평균값/imu_n만 제공한다. source와 STM flash binary 동일성은 아직 확인하지 않았다.

## 적용한 코드 보완

- `pi_worker.receipt_clock`: 기존 monotonic/realtime/span 필드를 유지하고 같은 bracket 안에 `pi_receipt_monotonic_raw_ns`를 추가했다. 다음 촬영의 callback 시각에만 추가되며 기존 촬영 시각은 변경하지 않는다. IMU와 RAW의 대응은 같은 Pi의 camera clock pairs를 이용해 후속 검증할 수 있지만 현재 MCU의 표본 시각으로 대체하지 않는다.
- `fit_clock`: 전체 잔차 분포에 더해8개 시간 구간의 중앙값과 범위를 보고한다. 각 구간20샘플 미만이면 trend 판정은 `None`이다.
- `analyze_run`: 중앙값 범위가20ms를 넘으면 `clock_model_unstable`로 기각하고 후보 CSV를 생성하지 않는다. 20ms는 현재 제어 주기 기준의 보수적인 진단 임계값이며 허용 VIO 동기화 오차가 아니다. 이 조건을 통과해도 `imu_clock_verified=false`, `orb_fused=false`다.

새 guard로 yaw post-reset 결과는 `post_reset_time_alignment_clock_guard`에서 기각되고, roll/pitch는 `imu_time_alignment_clock_guard`에서 -55ms/상관0.99632 후보로 유지됐다. 초기 보고서와 원본 로그는 보존했다.

## 검증과 결과 파일

TDD에서 미구현5건 실패를 확인한 뒤 관련26개 검사 통과. 공유 checkout 전체 회귀는 `239 passed, 7 skipped`(37.94s), 변경 파일 pyflakes와 diff check 통과. 독립 review에서 추가 Critical/Important 없음 확인.

production `receipt_clock` 함수만 Pi에서25회 실행해 RAW 증가와 최대 bracket2704ns를 확인했다. 카메라·SPI·모터는 열지 않았다. 장시간 RAW/monotonic rate 비교를 수행한 것은 아니다.

노트북 진단 폴더 `outputs/handheld_mapping/20261015_timing_forensics`:

- `summary.json`: 같은 frame support의4조건과 구간별 clock rate·잔차, 입력 hash.
- `compare_timing.py`: 오프라인 재현 스크립트. 출력은 실험용 진단이며 runtime 기능이 아니다.
- `timing_comparison.png`: 잔차 변화와 후보 시차 비교.
- `pi_clock_service.txt`, `pi_raw_clock_probe.json`, `firmware_reference/`: 읽기 전용 조사 자료.

다음은 새 RAW clock pairs를 포함한 정지 센서 기록으로 Pi 보정 여부를 분리하고, firmware에서 개별 IMU 표본/평균 구간의 시각을 확보하는 것이다. 기존 기록만으로 물리적 원인을 확정할 수 없으므로 고정 시차 적용·관성 융합은 보류한다. 추가 기울임 동작은 현재 필요하지 않다.
