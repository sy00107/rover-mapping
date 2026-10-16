# IMU 평균 읽기 시각 STM 실기 적용

2026-10-15, 사용자 요청에 따라 STM32F446RE에 평균 표본의 MCU 읽기 시각 패치를 적용했다. 사용자에게 구동·조향 모터 전원이 꺼진 것을 확인받고, Pi·STM·IMU만 켠 상태에서 POLL-only로 확인했다. 속도·조향 명령은 전송하지 않았다.

## 결과와 검증 범위

기존 firmware 전체 512KB를 노트북에 백업했다. ST-LINK에서 device id `0x10006421`, Flash 512KB를 확인하고 새 ELF를 program/verify/reset했다. OpenOCD는 `Verified OK`를 보고했다. Pi의 수정 파일 일곱 개 SHA256이 실제 ARM build source/header/decoder와 일치한다.

| 항목 | 적용 전 | 적용 후 |
|---|---:|---:|
| 시험 길이 | 30초 | 30초 |
| POLL 응답 | 1,496 | 1,496 |
| 디코딩 실패 | 0 | 0 |
| 첫 pipeline 응답 제외 유효 고유 IMU | 1,180 | 1,190 |
| 고유 IMU 수신률 | 39.34Hz | 39.67Hz |
| timing 유효 표본 | 미지원 | 1,190/1,190 |
| 고유 응답 MCU 간격 p5/중앙값/p95 | 20/20/40ms | 20/20/40ms |
| 고유 응답 MCU 최대 간격 | 60ms | 60ms |
| 시험 중 MCU 시각 역행 | 0 | 0 |
| MCU 수신 오류 누적값 시작→끝 | 1→1 | 0→0 |

첫 SPI 응답은 이전 세션 cache 또는 초기 frame일 수 있어 steady 비교에서 분리했다. 실제 적용 전 첫 응답은 오래된 유효 cache였고, 적용 후에는 IMU flag가 없는 초기 0 frame이었다. 약 50Hz로 polling해도 동일 telemetry가 중복 수신되므로 고유 표본만 계산했다. 수신된 command는 모두 0이었고 TELEOP_ACTIVE는 설정되지 않았다.

새 signed offset의 p5/중앙값/p95는 **−24,667/−9,800/−8,500µs**, 전체 범위는 **−24,667~−7,750µs**였다. 즉 평균 표본의 대표 읽기 시각이 frame의 제어 reference보다 보통 9.8ms 앞선다. 이 값은 camera–IMU offset이 아니다.

시험은 정지 상태·카메라 비동시 운용이다. 카메라 동기화, 움직임 중 추적 안정성, ORB fusion, 실행 시간의 최악값을 검증한 것은 아니다. gyro/acc 로그는 Pi decoder 출력이며, 노트북의 보정 calibration을 적용한 값으로 해석하지 않는다.

## 빌드

- source baseline: KSRC `40499bcb1acaabf6bbbf604795caf28b6a952739`.
- native mock/UBSan, driver 및 protocol 호환 테스트: 새로 복사한 baseline 대상으로 **11 passed**.
- GNU Arm Embedded GCC 10.3.1, binutils 2.38, newlib 3.3, OpenOCD 0.11.0. Ubuntu 패키지를 관리자 설치 없이 `/home/su/rover/.tools/arm-build/root`에 풀어 사용했다.
- Debian alternatives가 만드는 `arm-none-eabi/include → ../../include/newlib`, `arm-none-eabi/lib → newlib` 링크를 로컬 도구 폴더에 구성했다.
- Makefile의 `--no-warn-rwx-segments`는 binutils 2.38에서 지원하지 않아 두 build 모두 command-line `LDFLAGS`에서 그 warning 억제 옵션만 뺐다. source Makefile은 바꾸지 않았다.
- 패치 전체 compile/link에 warning/error 없음.

| 크기 | 원본 ARM build | 패치 ARM build | 증가 |
|---|---:|---:|---:|
| text | 75,832 | 76,488 | 656 bytes |
| data | 600 | 600 | 0 |
| bss | 44,632 | 44,680 | 48 bytes |

이 비교는 동일 toolchain으로 빌드한 source 간 비교다. 교체 전 STM binary와 재빌드한 baseline binary가 동일하다는 주장은 하지 않는다. 정적 크기는 stack의 최대 사용량을 포함하지 않는다.

재현 명령은 작업 source의 `firmware/src2026-rover-firmware`에서 실행한다.

```bash
PATH=/home/su/rover/.tools/arm-build/root/usr/bin:$PATH \
make -j4 BUILD_DIR=build_timestamp \
  LDFLAGS='-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -specs=nano.specs -u _printf_float -TSTM32F446RETX_FLASH.ld -lc -lm -lnosys -Wl,-Map=build_timestamp/rover_motor.map,--cref -Wl,--gc-sections'
```

## 저장 위치와 상태

전체 로그·source·binary·백업은 노트북 `/home/su/rover/outputs/handheld_mapping/20261016_imu_timestamp_deploy`에 저장했다. 폴더 이름의 날짜와 별개로 이 시험은 10월 15일 수행했다.

- `summary.json`: 전후 수신 통계, compiler 및 artifact SHA256.
- `imu_before.jsonl`, `imu_after.jsonl`: POLL-only 원본 기록.
- `baseline.tar`: 수정 전 Pi tracked source archive.
- `stm32_flash_before.bin`: 수정 전 실제 STM Flash 512KB.
- `flash_backup.log`, `flash_program.log`: 백업·program/verify/reset 기록.
- `build_baseline.log`, `build_timestamp.log`: ARM build 기록.
- `deployed_sources.json`: Pi source 일곱 개의 SHA256.
- `source/firmware/src2026-rover-firmware/build_timestamp/rover_motor.{elf,bin,hex}`: 적용한 build.

백업 Flash SHA256: `19b5ae2f491d720294af2f010f291e15fb1f5715dcbf09c4c5e244d3d29b05dc`.
적용 ELF SHA256: `bbb0f9d2d3468e49947c0cc541746d5a88cae29610067793be2e82a1a4446b9b`.

Pi `/home/pi/ksrc`에는 패치만 적용했고 Git commit/push는 하지 않았다. 대용량 로그는 Pi에 저장하지 않았다. 시험 프로세스는 종료했고 SPI device 점유는 해제했다. GUI 녹화·실제 조종은 시작하지 않았다.

현재 mapper는 raw telemetry 전체를 기록하므로 새 timing 필드를 보존할 수 있다. 기존 정렬 코드가 이 offset을 자동으로 사용하거나 `imu_clock_verified`/`orb_fused`가 true로 바뀌는 것은 아니다. 다음 작업은 카메라 동시 기록, offset을 반영한 시간 정렬 비교, offline stereo-inertial 비교다.

## 복구

복구도 모터 전원을 끈 상태에서만 진행한다. 원본 binary는 위 노트북 폴더에 보관되어 있다. OpenOCD의 [program/verify/reset 절차](https://openocd.org/doc/html/Flash-Programming.html)를 사용한다.

```bash
cd /home/su/rover
LD_LIBRARY_PATH=/home/su/rover/.tools/arm-build/root/usr/lib/x86_64-linux-gnu \
  .tools/arm-build/root/usr/bin/openocd \
  -s .tools/arm-build/root/usr/share/openocd/scripts \
  -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c 'program outputs/handheld_mapping/20261016_imu_timestamp_deploy/stm32_flash_before.bin 0x08000000 verify reset exit'

# Pi source 변경이 이 패치뿐인지 먼저 git diff로 확인하고 되돌린다.
ssh pi@raspberrypi.local 'cd /home/pi/ksrc && patch -R --batch --fuzz=0 -p1' \
  < experiments/imu_timing_patch/ksrc_imu_mean_time.patch
```

USB 권한은 사용자가 현재 ST-LINK device에 `setfacl`로 부여했다. 재연결 시 bus/device 번호와 권한을 다시 확인해야 한다.
