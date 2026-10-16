# Rover Mapping

D435와 Raspberry Pi로 영상을 수집하고, 노트북에서 ORB-SLAM3·TSDF 기준 3D 맵을 만드는 프로젝트다. 촬영 중 coverage/오버레이, 촬영 후 지형 층·prototype costmap 분석, 기존 STM 조이스틱 연동을 포함한다.

**현재는 기준 맵 품질을 검증하는 단계다.** 보정 IMU와 원본 encoder telemetry를 영상과 같은 RUN에 기록하는 경로를 실제 Pi에서 확인했다. 관성 융합, 중력 정렬 자동화, 범용 스캔/STL 위치 보정, live costmap은 후속 작업이다. 실제 주행의 입력 공백 원인도 새 계측으로 확인해야 한다.

## 먼저 읽을 문서

| 목적 | 문서 |
|---|---|
| 처음 설치 | [설치 설명서](docs/installation.md) |
| 촬영·조종·지도·costmap 운영 | **[사용 설명서](docs/user-guide.md)** |
| 구현·검증 상태와 다음 작업 | [통합 인수인계서](docs/handoff/d435-rover-mapping-handoff.md) |
| 새 담당자/팀원 공유 | [공유·기여 안내](docs/sharing.md) |
| 제외된 원본/실행 자산 | [데이터·runtime 안내](docs/data-assets.md) |
| 입력 공백 조사 | [조사 보고](docs/experiments/2026-10-14-input-gap-investigation.md) |
| 최신 20° 카메라 장착 URDF | [기구 기준·보정 적용 범위](docs/experiments/2026-10-14-rover-extrinsics-update.md) |
| 보정 IMU와 D435 동시 기록 | [실행·검증 결과](docs/experiments/2026-10-14-imu-co-recording.md) |
| 카메라–IMU 시간 정렬·종료 후 선택형 융합 | [도입 내용·사용법·현재 검증 범위](docs/experiments/2026-10-16-imu-upstream-integration.md) |
| 기존 촬영들의 별도 통합 지도 | [영상 검증·원본 깊이 병합·결과 보기](docs/experiments/2026-10-16-session-merge.md) |
| 전체 촬영 검토·촬영 간 충돌 관측 필터 | [사용한 기록·노이즈 검사·새 통합 지도](docs/experiments/2026-10-16-session-consensus.md) |
| upstream 버전·patch | [외부 의존성](third_party/README.md) |

## 빠른 시작

GitHub 접근 권한이 있는 계정으로 clone한다.

```bash
git clone git@github.com:lee-robot-lab/rover-mapping.git
cd rover-mapping
```

[설치 설명서](docs/installation.md)의 Python 환경을 준비한 뒤:

```bash
# 장비 없이 GUI → 원본 저장 → 합성 pose TSDF
bash tools/handheld_mapping.sh --demo

# 장비 없이 지형·costmap 파일 생성 (새 output 필요)
bash tools/terrain_mapping.sh --demo --output outputs/terrain_demo_01

# 저장된 최신 촬영의 3D 지도
bash tools/view_mapping.sh
```

실제 촬영은 native point runner와 Pi runtime을 준비한 뒤 `bash tools/handheld_mapping.sh`로 시작한다. runner가 없으면 live 안내·노트북 ORB 옵션은 활성화되지 않는다. 합성 GUI/terrain demo는 native ORB 없이도 사용할 수 있다.

기본 실제 구성:

```text
Pi: D435 IR좌/우 + depth 640×480, 요청15FPS, projector OFF
  → bounded queue → zlib → SSH
노트북: 원본 저장 ┬→ live ORB → 잠정 coverage/IR 오버레이
                 └→ 종료 후 full IR ORB → 최종 keyframe + depth → TSDF
선택 map + 검증한 중력 정렬 → 지형 층 → 로버별 costmap
```

## 저장소 범위

| 폴더 | 내용 |
|---|---|
| `experiments/handheld_mapping` | Qt·수집/전송·live/offline ORB·TSDF·지형·costmap |
| `tools` | GUI/뷰어/terrain wrapper, native runner build |
| `ros2_ws/src/orbslam3_ros2` | ROS adapter·headless 준비 도구·native patch |
| `ros2_ws/src/rover_description/urdf` | 좌표/보정 preflight에 사용하는 URDF 참고본 |
| `teleop/ksrc_snapshot` | 확인한 KSRC의 조이스틱·SPI bridge 소스 일부 |
| `tests` | 매핑/headless/fake SPI 회귀 |
| `docs` | 설명서·인수인계·작은 검증 보고서 |
| `third_party` | 외부 프로젝트 버전과 라이선스 안내 |

촬영 PNG/PLY/Atlas, native 바이너리·vocabulary, venv, 빌드 결과, 원본 전체 firmware/대형 로버 mesh는 Git에 포함하지 않는다. 결과는 로컬 `outputs/`, 외부 소스는 로컬 `external/`에 준비한다. [필요한 별도 자산](docs/data-assets.md)을 확인한다.

## 회귀 테스트

```bash
KSRC_REVIEW_ROOT="$PWD/teleop/ksrc_snapshot" \
QT_QPA_PLATFORM=offscreen PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv-mapping/bin/python -m pytest -q
```

2026-10-14 원본 workspace의 해당 범위는 **173 passed**였다. 이 저장소는 optional native binary를 제외하므로 그 실행기 검사 1개는 빌드 전 skip될 수 있다. 실제 하드웨어를 열지 않는 fake SPI 테스트를 포함한다. 공유본의 실제 결과는 [공유본 검증](docs/share-validation.md)에 기록한다.

Python-only demo·회귀 성공은 실측 지도 정확도나 현재 Pi 전체 처리율 검증을 뜻하지 않는다. voxel 1cm/추적 성공률/초록 coverage를 정확도·통행 가능성의 보증으로 사용하지 않는다.

## 환경·버전

노트북 Ubuntu22.04/Python3.10, Pi Ubuntu24.04/native ROS2 Jazzy/Python3.12가 기준이다. 노트북 매핑은 `.venv-mapping`을 쓰며 Conda/ML 환경과 섞지 않는다. Pi에 Humble를 source하지 않는다. 새 머신의 native/RealSense 설치는 별도 확인해야 한다.

이 저장소는 private 팀 공유용이다. ORB-SLAM3/ORB-LINE과 ROS adapter 등의 기존 라이선스는 [외부 의존성 안내](third_party/README.md)를 따른다. 별도의 프로젝트 전체 라이선스를 임의로 지정하지 않았다.
