# Pi 원본 기록 / 종료 후 노트북 연산 아키텍처 리뷰

아래 내용은 후속 수정 전 발견 기록이다. 사용자 승인 범위 1·2·3·4·6·7의 수정 결과는
[연결 내성 보완 기록](2026-10-18-capture-resilience.md)을 참고한다. 재부팅 후 복구 실행 경로(5번)는 보류했다.

검토 대상은 2026-10-18 현재 작업 트리의 `local_recording.py`, `local_client.py`,
`controller.py`, `pi_worker.py`, `imu.py`, IMU 브리지 실행 스크립트와 관련 테스트다.
이번 작업은 리뷰와 로컬 오류 재현이며, 촬영 코드 수정이나 실기 촬영은 하지 않았다.

판정: 원본 기록과 무선 미리보기의 분리는 적절하다. 하지만 현재 전체 실행 경로에서는
통신 단절 내성과 실패 원본 회수가 IMU 수명 및 복구 코드 때문에 완성되지 않았다.
"Wi-Fi 때문에 생기던 원본 누락을 줄이는 구조"이며 "원본 누락이 없어지는 보장"은 아니다.

## 실제 구성과 정상 경로

1. 노트북이 코드를 Pi RAM 작업 폴더에 배포하고 영구 recording 폴더에 설정을 둔다.
2. Pi의 독립 supervisor가 카메라 worker와 IMU subscriber를 실행한다.
3. 카메라 worker는 좌/우 Y8와 깊이 Z16을 32 MiB 유한 큐, 무손실 압축, 로컬 pipe로 보낸다.
4. supervisor가 영구 저장장치의 `capture.bin`에 기록하고 약 2초마다 fsync한다.
   IMU 원본·보정값·각 시계 정보는 같은 세션 아래에 따로 기록한다.
5. 별도 미리보기 스레드는 좌우 320×240 JPEG와 최대 2 FPS의 무손실 깊이 snapshot을 만든다.
   SSH attachment는 최신 snapshot과 상태만 읽는다. 미리보기 SSH 종료 자체는 supervisor 종료 조건이 아니다.
6. 촬영 종료 및 파일 마감 뒤 size/SHA256 manifest를 생성하고 노트북으로 offset 이어받기한다.
7. 노트북은 체크섬 확인 후 PNG/인덱스를 복원하고 개수를 대조한다. 그 후 원본으로
   offline ORB, 중력 기준, TSDF 및 지형 후처리를 실행한다.

실시간 범위 표시를 켜면 노트북의 미리보기 ORB도 촬영 중 동작한다. 따라서
"최종 지도는 촬영 후 원본으로 계산"은 맞지만 "모든 계산을 촬영 후에만 함"은 아니다.
IMU 저장과 ORB 관성 융합도 별개다. 기본 영상 ORB와 선택형 관성 후처리를 구분해야 한다.

## 발견 사항

### P1-1. IMU 송신 브리지가 여전히 SSH 수명에 묶여 있다

- 위치: `tools/mapping_imu_bridge.sh:33`, `local_recording.py:216`, `imu.py:426`.
- 터미널 1은 `ssh -tt ... exec udp_spi_bridge.py`로 실행된다. 카메라 기록과 달리
  detached 서비스가 아니며, 적용 패치/저장소 브리지는 SIGHUP 독립 처리를 제공하지 않는다.
- SSH 세션이 실제로 끊어져 원격 PTY가 닫히면 브리지가 종료될 수 있다. Pi subscriber의
  IMU unique 표본은 0.5초 뒤 stale이 되고 supervisor는 전체 카메라 촬영을 정지한다.
- 짧은 Wi-Fi 지연으로 TCP 세션이 유지되는 경우와 SSH가 끊어진 경우는 구분해야 한다.
  모든 짧은 무선 지연이 이 문제를 일으킨다는 뜻은 아니다.
- 재현: 실제 카메라 명령 대신 지속 synthetic 영상 공급, 독립 IMU 공급을 사용하고
  IMU만 0.9초 일시 정지했다. 사용자의 stop 요청 없이 81/81/80장 뒤 자동 종료되고
  `Pi IMU 기록 중단: fresh sample timeout`이 기록됐다. 별도 로컬 PTY 종료 시험도
  기본 Python 자식이 SIGHUP(1)으로 종료되는 것을 확인했다. 실제 Pi SSH 절단 시험은 아니다.
- 권장: SPI 소유 브리지를 단일 Pi 서비스로 관리하고, 터미널 1은 상태/로그를 보게 한다.
  UDP 명령 stale 시 0 명령 및 MCU watchdog 동작은 유지한다. IMU 누락은 영상 원본 기록과
  분리하고 구간 품질을 기록한다. IMU가 필수인 실험에만 명시적인 엄격 종료 정책을 둔다.

### P1-2. IMU 오류를 받으면 원본 회수보다 먼저 노트북 세션이 종료된다

- 위치: `controller.py:256`, `controller.py:276`.
- `local_status.imu.error`를 RemoteImu에 복사한 뒤 다음 반복 시작에서 즉시 예외를 낸다.
  Pi 마감 manifest 및 원본 전송을 기다리지 않고 local_client를 정리한다.
- 재현: IMU 오류 상태 뒤에 검증된 원본 import 패킷을 보내도 노트북은 원본 0장으로
  failed가 된다. 오류가 있는 촬영의 회수 경로가 카메라 오류 테스트와 다르게 끊긴다.
- 권장: Pi 로컬 기록 모드의 IMU 오류는 종료 사유/품질 상태로 보관하되, 저장된 원본
  회수와 검증을 먼저 끝낸다. 그 뒤에 실패·부분 성공을 판정한다. IMU 문제 때문에
  영상 원본 회수나 영상 전용 후처리까지 막지 않는다.

### P1-3. 한 스트림의 수신 중단과 수신 전 누락을 완전성 판정에서 놓친다

- 위치: `pi_worker.py:475`, `pi_worker.py:496`, `controller.py:305`, `controller.py:438`,
  `offline.py:36`.
- 실행 중 watchdog은 left 시각만 본다. left가 계속 오면 right/depth가 멈춰도
  카메라 노드가 살아 있는 한 이 watchdog은 동작하지 않는다.
- recorder는 worker의 `saved`와 저장 개수를 받아들이고, 노트북은 "Pi에 저장된 개수와
  동일하게 복사됐는가"를 확인한다. 이 검사는 촬영 자체의 좌우 쌍 완전성을 검증하지 않는다.
- offline ORB는 대응하지 않는 영상을 건너뛰어 남은 쌍으로 계산한다. 그 짧은 구간의
  재구성이 성공하면 촬영 전체가 누락됐다는 상태가 최종 판정에서 빠질 수 있다.
- 재현: right 공급만 8장 이후 멈춘 로컬 시험에서 left=40/right=8/depth=40인데
  recorder manifest의 error=null, worker status=saved, dropped=0이 그대로 통과했다.
  이는 recorder 계층 재현이며 실제 ROS watchdog의 left-only 동작은 소스로 확인했다.
- DDS sensor-data subscription과 callback 이전의 누락도 큐 dropped에 포함되지 않는다.
  max_source_gap_ms 계측은 있으나 최종 complete/partial 판정에 사용하지 않는다.
- 권장: stream별 최대 공백, 좌우 exact timestamp 매칭률, 깊이 대응률, 가능하면
  카메라 원래 frame counter와 누락 구간을 저장한다. 전송 검증 성공과 촬영 완전성을 분리한다.

### P2-1. recorder 비정상 종료 후 남은 원본을 일반 복구 명령으로 회수할 수 없다

- 위치: `local_recording.py:360`, `local_recording.py:414`, `local_client.py:156`.
- recorder가 done.json 작성 전에 죽으면 attach는 파일 목록이 빈 완료 오류를 반환한다.
  fetch는 done.json을 필수로 요구하고 copy_originals는 capture.bin 항목이 없으면 거부한다.
- 재현: recorder만 강제 종료한 후 11,762바이트의 capture.bin이 남았지만,
  attach는 files=[]를 반환하고 정상 회수 경로는 `Pi 원본 영상 파일이 없습니다`로 실패했다.
- 파일 자체가 반드시 사라지는 문제는 아니다. 현재는 직접 파일을 복사하고 검사하는 별도 수작업이 필요하다.
- 권장: 마감된 청크별 manifest를 먼저 남기고, 미마감 청크는 완전한 packet prefix를
  스캔해 salvage한다. 복구를 위해 카메라 촬영을 다시 시작하지 않는다.

### P2-2. Pi 재부팅 뒤 복구 명령이 사라진 RAM 배포 경로에 의존한다

- 위치: `controller.py:36`, `controller.py:60`, `local_client.py:21`, `local_client.py:332`.
- recordings는 영구 저장되지만 fetch/start 명령의 cwd와 Python 모듈은
  `/dev/shm/rover_mapping_RUN_ID/code`다. --recover는 이를 다시 provision하지 않는다.
- 따라서 재부팅 전 촬영이 정상 마감됐더라도 재부팅 후 일반 복구 명령은 cd/import 단계에서 실패한다.
  Pi 원본이 남았는지와 복구 프로그램이 실행 가능한지는 별개다.
- 권장: 버전이 기록된 recorder/recovery 코드를 영구 runtime에 두거나, 복구 전용 코드를
  안전하게 재배포한다. 기존 recording 설정/원본을 덮어쓰는 일반 촬영 provision과 분리한다.
- 코드 경로 분석으로 확인했으며 실제 Pi 재부팅은 수행하지 않았다.

### P2-3. 앱 재실행 후 기존 .part 다운로드를 이어받는 진입점이 없다

- 위치: `local_client.py:332`, `storage.py:59`.
- receive_file은 같은 작업에서 offset 이어받기를 지원하지만 --recover는 항상 새 RunWriter를
  만들며 기존 폴더를 거부한다. 다운로드 실패나 앱 종료 뒤 기존 .part를 지정해 재개할 수 없다.
- 재현: pi_originals/capture.bin.part가 있는 폴더를 --recover --run에 주면
  SSH 연결 전에 FileExistsError로 종료된다. 새 폴더를 지정하면 기존 부분 다운로드는 재사용되지 않는다.
- 권장: 검증된 run_id/host/manifest checkpoint로 기존 다운로드를 재개하고, 원본 import도
  이미 생성된 PNG/인덱스를 검증해서 중복 없이 재시작할 수 있게 한다.

### P2-4. 원격 종료 확인 전에 전송 상태로 넘어가며, 연결 불가 시 취소 경로가 없다

- 위치: `local_client.py:238`, `local_client.py:286`, `controller.py:128`,
  `controller.py:152`, `app.py:453`.
- stop을 로컬에서 읽는 즉시 transfer_begin을 emit한다. 이때 원격 recorder가 stop을
  실제 받았는지는 확인하지 않는다. 전송 상태가 되면 75초 종료 watchdog도 대기를 끝낸다.
- 이후 manifest가 없으면 영구적인 SSH/인증/경로 오류에서도 재연결을 무한 반복한다.
  앱 닫기는 active 동안 거부되고 transferring 상태의 stop은 별도 취소 동작이 없다.
- 실제 Pi에서는 아직 촬영 중인데 노트북은 "촬영 종료·원본 전송 중"으로 보일 수도 있다.
- 권장: 종료 요청/원격 종료 확인/파일 전송을 구분하고, 장기 단절 시 원본 위치와
  재개 checkpoint를 남겨 앱만 안전하게 종료하는 동작을 제공한다. 로컬 취소를
  원격 촬영 종료 성공으로 오인해서는 안 된다.
- 상태 흐름 분석으로 확인했으며 실제 장기 단절/GUI 강제 종료 실험은 수행하지 않았다.

## 유지할 설계와 추가 개선

유지할 것: Pi 영구 원본, SSH와 분리된 기록 supervisor, 좌우 원본과 JPEG 미리보기의
물리적 경로 분리, 깊이 대기 없는 stereo 미리보기, size/SHA256 검증, 최종 원본 기반 계산.
네트워크 프로토콜 전체 교체나 GPU 도입보다 위 실패 경로 보완이 먼저다.

다음 개선이 효과적이다.

- 원본은 적당한 크기/시간의 청크로 마감하고 체크섬·인덱스를 저장한다. 단일 capture.bin
  때문에 촬영 전체가 최종 마감에 의존하지 않게 한다.
- preview.bin/seq/status처럼 매초 여러 번 바뀌는 임시 snapshot은 한정된 RAM 공간에서
  교체한다. 현재는 원본과 같은 영구 디스크에서 반복 write/rename하므로 CPU뿐 아니라 I/O도 경쟁한다.
- 압축/원본 writer/미리보기를 동시에 실행한 실제 Pi CPU·저장장치 지연을 측정한다.
  640×480×15 FPS의 좌우 mono8와 Z16 깊이는 무압축 약 18.432 MB/s다.
  큐가 압축 전 32 MiB라 최악 입력 기준 약 1.82초 분량이며 긴 쓰기 지연을 흡수하는 양은 제한된다.
  현재 큐는 포화 시 drop하고 partial로 남기는 설계다. 무손실 압축과 프레임 누락 0은 다른 조건이다.
- 카메라 로그는 생성 단계부터 순환/용량 제한한다. 최종 256 KiB 진단본 수집은 있지만
  pi_worker.py:502에서는 여전히 원본 camera.log 전체를 메모리에 읽어 packet으로 보낸다.
  로그가 커지면 RAM, `/dev/shm`, protocol MAX_PAYLOAD 제한에 걸릴 수 있다.
- 노트북 공간은 전송 파일만이 아니라 미리보기, capture.bin 사본, 압축 해제 PNG,
  IMU 사본과 최종 지도를 합해 검사한다. 현재 단일 파일의 남은 byte+256 MB 검사는
  이후 압축 해제 공간까지 보장하지 않는다.
- IMU와 카메라 시계 출처를 계속 보존하되, 기록 성공을 관성 융합 성공으로 표시하지 않는다.
  임의 보간으로 IMU 빈 구간을 유효 실측값처럼 채우지 않는다.

## 검증 결과와 제한

- 전체 handheld 회귀: **509 passed, 9 skipped (61.31초)**.
- 새 로컬 오류 주입 5건: 모두 현재 실패/누락 판정의 한계를 재현했다.
- 증거/재현: `outputs/handheld_mapping/reviews/capture_architecture_20261018/probes.py`,
  `probe_results.json`. 실제 subprocess를 사용하되 영상과 IMU 공급은 synthetic이다.
- 기존 연결 종료 테스트는 camera attachment만 닫고 synthetic IMU 공급은 계속 켜 둔다.
  따라서 터미널 1의 SSH 브리지 종료 문제를 검증하지 않는다.
- read-only SSH 점검은 raspberrypi.local 이름 해석 실패로 실행되지 못했다.
  이번 리뷰에서 실제 Pi의 CPU/디스크/카메라 상태는 확인하지 못했다.
- 실제 카메라, SPI, 모터, Wi-Fi 차단, 재부팅 명령은 실행하지 않았다.

실기 확인은 영상/IMU 지속 입력 중 미리보기 SSH만 끊는 시험과 전체 무선 단절 시험을
구분해야 한다. 각 stream의 timestamp/sequence 공백, 원본 기록 FPS, 큐 drops,
재연결 뒤 파일/해시/좌우 쌍을 대조해야 실제 무누락 여부를 판단할 수 있다.
