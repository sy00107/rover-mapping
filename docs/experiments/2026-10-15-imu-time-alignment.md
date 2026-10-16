# 카메라–외장 IMU 시간 정렬 준비와 첫 실기

2026-10-15. D435 IR 15FPS와 외장 ISM330DHCX를 같은 RUN에 기록하고, 노트북에서 시간 관계를 추정한다. IMU 관성 융합이나 자동 지도 수평화는 이 작업에 포함하지 않는다.

## 준비된 기능

- 카메라 callback에서 Pi realtime을 두 monotonic 조회로 감싸고 midpoint·조회 구간 폭을 원본 image header에 추가한다. 기존 `stamp_ns`를 바꾸지 않는다.
- MCU `t_ms`의 uint32 wrap을 처리하고 Pi 수신 시각에 대한 affine clock rate·offset을 추정한다. reset을 가로지르는 모델은 거부한다.
- 같은 map의 연속 유효 영상 pose에서 회전 속도 크기를 구하고, 해당 구간의 평균 자이로 크기와 비교한다. 장착 축 회전 검증과는 별개다.
- 모든 후보 시차에 동일한 영상 구간을 사용한다. 큰 IMU 공백을 가로지르는 보간, 정지·모호한 반복 움직임·약한 상관·검색 경계·앞뒤 절반의 불안정한 시차를 거부한다. 각 절반도 회전 변화·상관·검색 경계를 검사하며, 근거 없는 절반에는 임의 peak 대신 `null`을 남긴다.
- 종료된 RUN만 분석하며 기존 파일·기존 결과를 덮어쓰지 않는다. 로그는 한 줄씩 읽고 필요한 수치만 유지하며 IMU 200,000개·카메라 100,000장 한도를 명시적으로 검사한다.

## 실제 촬영

이미 기존 SPI bridge가 실행 중이면 두 번째 bridge나 SPI reader를 시작하지 않는다. 새 부팅에서 기존 설명서대로 `tools/mapping_imu_bridge.sh`를 켠 뒤 노트북 조이스틱과 매핑 앱을 연다. 모터 명령은 사용자가 조이스틱으로 조작한다.

매핑 앱에서 `보정 IMU 함께 기록`, `실시간 촬영 범위 표시`, `Pi는 센서 전송만`을 켠다. native live ORB의 `orb_live/live_poses.csv`가 시차 분석에 필요하다.

1. `매핑 시작`을 누르고 영상·IMU 수신을 확인한다.
2. 약 5초 정지한다.
3. 짧은 좌회전, 정지, 조금 긴 우회전을 속도·간격을 달리하며 1분 이상 수행한다. 일정한 주기를 반복하지 않는다. 정지·직선만으로 회전 시차를 추정하지 않는다.
4. 로버를 먼저 정지한 다음 `종료 및 지도 생성`을 누른다. 매핑 종료가 모터 정지는 아니다.
5. IMU 기록과 live ORB가 종료된 뒤 아래 분석 명령을 실행한다.

```bash
bash /home/su/rover/tools/imu_time_alignment.sh /absolute/path/to/RUN
```

공유 checkout에서는 `/home/su/rover` 대신 그 checkout 경로를 사용한다. 기본 분석 출력은 `RUN/imu_time_alignment`다. 이미 있으면 오류를 내므로 다시 분석할 때는 새로운 `--output /absolute/path/to/NEW_DIRECTORY`를 지정한다. GUI에서 자동 실행되거나 ORB 설정에 자동 반영되지는 않는다.

## 결과 읽기

- `summary.json`: 상태·기각 사유·clock rate·시차 부호·상관·앞뒤 절반 결과·샘플/누락 수·입력 hash.
- `alignment.png`: 회전 속도 비교와 시차 검색 곡선. 비교 가능한 신호가 있는 경우에만 생성한다.
- `README.md`: 해당 RUN의 판정과 적용 제한.
- `imu_alignment_candidate.csv`: 품질 조건을 통과했을 때만 생성하는 **미검증 후보**다. rad/s·m/s²로 변환한 측정과 공통 Pi monotonic 기준 시각을 담으며 ORB에 자동 입력하지 않는다.

시각 보정의 부호는 다음과 같다.

```text
aligned_imu_time = fitted_imu_pi_time + imu_time_correction_s
```

`-0.045`이면 MCU→Pi affine 모델로 얻은 IMU 시각을 45ms 앞당기는 후보다. Pi↔노트북 전송 지연이나 카메라의 절대 지연을 45ms로 측정한 뜻이 아니다. 이 값을 다른 RUN의 raw MCU timestamp에 그대로 더하지 않는다.

## 첫 실기 결과

| RUN | 입력/판정 | 결과 |
|---|---|---|
| 20261015_211049_7fac29 | 정지 연결 확인, 시차 거부 | 영상 15FPS, IMU 고유 패킷 약39.6Hz. 회전 변화 부족으로 `insufficient_motion` |
| 20261015_212049_bc26d3 | 회전 기록, 추정 후보 | IMU 4116개 고유 샘플, 약49.7Hz. 영상 비교1118구간, 시각 보정-45ms, 상관0.9991, 앞/뒤 절반 각각-45ms |

최신 회전 기록은 MCU→Pi 시간 속도 비 약0.99691(차이 약-3090ppm)이며, 수신 시각에 대한 affine 잔차 p5/p95는 약-9.4/+9.6ms다. 고정 offset만 적용하지 않고 기록별 시계 속도를 함께 고려한다.

review에서 정지한 절반의 임의 peak를 거부하도록 보완한 뒤 최신 기록을 `RUN/imu_time_alignment_reviewed`에 재분석했다. 보정 후보는 동일한 -45ms이고, 앞/뒤 절반 상관은 각각0.9989/0.9994다. 초기 분석과 원본 파일은 보존했다.

같은 기록의 offline ORB는1121쌍 전부 추적하고 map0을 유지했다. 최종 TSDF는33개 keyframe을 융합했다. 다만 수집 queue에서23개 image packet이 누락돼 전체 세션은 `partial`이다. IMU tee 누락·MCU reset은 없었다. 이 결과가 위치 정확도 검증은 아니다.

과거 두 주행 기록도 같은 도구로 비교했다. 20261015_000854_aa8850은-55ms/상관0.912, 20261015_001142_c10be6은-40ms/상관0.955 후보였다. 이 기록은 새 카메라 callback realtime 필드가 없어서 같은 Pi의 IMU realtime/monotonic 관계로 변환한 legacy 진단이다.

## 아직 필요한 검증

후속 실행 순서와 현재 결과는 [검증 단계표](../superpowers/plans/2026-10-15-imu-validation-sequence.md)에 기록했다. 기존9개 RUN을 현재 분석기로 재검사한 결과6개는 -70~-40ms 후보, 3개는 기각이었다. 최신 RUN의 body 좌표계 yaw 부호 비교는100% 일치하고 상관0.99970이었다. 새 clock pair를 가진 회전 RUN은1개라 고정 offset·3축 외부 보정·다른 부팅 재현성은 아직 미검증이다.

- 첫 회전 RUN에는 실제 camera metadata를 보존하지 않았다. 후속 정지 시험에서는 `global_time` domain과 image/metadata header 대응을 확인했다. 노출 중간 시각을 반영한 회전 시차·MCU 동기화 정확도는 아직 미검증이다.
- 현재 IMU는 여러 센서 측정의 평균을 제어 패킷에 담는다. 평균 구간·실제 측정 timestamp·SPI 응답 지연의 절대값을 확인해야 한다.
- 5ms 검색 간격은 5ms 정확도 보증이 아니다. 15FPS 영상과 약50Hz 평균 패킷의 시간 해상도가 제한된다.
- 다른 회전 기록과 다른 부팅에서 결과를 재검증한다. 실측 clock·축·camera↔IMU 변환과 noise가 확인되기 전 `imu_clock_verified=false`, `orb_fused=false`를 유지한다.
- 장착 변환과 중력 정렬·관성 융합은 후속 단계다. 이번 후보 CSV를 만들어 ORB가 IMU를 사용한다고 설명하지 않는다.

## 검증 방법

실행 환경은 기존 `.venv-mapping`이며 Conda/ML 환경을 섞지 않는다. 알려진 양/음 시차, 시계 속도 차, uint32 wrap/reset, 정지·정지한 절반·주기적 모호함·센서 공백·검색 경계, 새로운/legacy clock header, 원본 보존·출력 덮어쓰기 거부를 회귀 검사한다.

2026-10-15 공유 checkout 검증: 시간 정렬17개와 capture 계측6개 통과, 전체 회귀 `236 passed, 7 skipped`(37.22s). 변경 Python 파일의 pyflakes와 wrapper의 `bash -n`도 통과했다. 독립 review의 정지 half finding을 수정하고 재검토에서 추가 finding 없음 확인. 이 소프트웨어 검증은 하드웨어 시각 정확도나 지도 위치 정확도의 검증을 대신하지 않는다.
