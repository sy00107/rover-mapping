# IMU timestamp 벤치 적용 계획

**Goal:** 검증한 평균 읽기 시각 패치를 ARM 빌드하고, STM 적용 후 모터 명령 없이 수신을 확인한다.
**Spec:** experiments/imu_timing_patch/README.md
**Architecture:** Pi의 고정 commit 원본을 노트북에 복사하여 원본/패치 버전을 별도로 빌드한다. 현재 STM flash를 백업한 뒤 적용하고, POLL-only로 원본·신규 telemetry를 비교한다.
**Tech Stack:** STM32F446RE, GNU Arm Embedded GCC, OpenOCD, Pi Python/spidev.

## 제약과 검증

- 실제 주행 명령 전송 금지. Pi 저장 공간 대신 노트북 outputs에 로그와 빌드 결과 보관.
- native mock 11개를 다시 확인하며, 전체 ARM build와 CRC·신구 decoder 호환 검증을 추가한다.
- baseline commit/hash 불일치, target 식별 실패, flash 백업 실패 시 flash하지 않는다.
- ST-LINK 접근 권한이 없으면 구체적인 사용자 조치를 요청하고 빌드는 계속한다.
- timing flag 비율, signed offset 범위, 표본 수, CRC 실패, MCU reset·주기를 수신 로그에서 확인한다.

## 순서

- [x] 원본 source 보존 및 작업 폴더에 ARM/OpenOCD 도구 준비.
- [x] 원본과 패치 firmware를 별도 build하고 크기·warning 비교.
- [x] 기존 POLL-only 로그와 STM flash 백업 확보.
- [x] 검증된 binary 적용·verify 및 Pi decoder 백업 후 교체.
- [x] POLL-only 수신 확인, 결과와 복구 경로 기록.
