# 촬영 시작 후 자동 종료 — USB 연결 끊김

## 확인한 사실

- `20261018_150045_99e56e`: left/right 71장씩, depth 95장 기록 후 영상 수신 중단으로 실패.
  약 25.1 MB 전송은 정상 촬영 완료가 아니라 남아 있는 원본과 IMU의 회수였다.
- `20261018_150509_6df868`: 시작 시 `actual image profile/rate mismatch`로 실패.
  영상은 0장, IMU 1122건이 남았으며 약 2.4 MB 전송의 대부분은 IMU였다.
- Pi의 두 RUN `run/camera.log`에 `The device has been disconnected!`,
  `No such device`, `Depth stream start failure`가 반복됐다.
- 같은 로그의 `Device USB type: 2.1`과 Pi sysfs의 D435 `speed=480`을 확인했다.
  USB 단절은 확인됐지만 케이블·포트·전원 중 정확한 물리 원인은 확정하지 않았다.
- 첫 RUN camera.log 전체 복사는 SSH timeout으로 중단됐다. 해당 노트북
  `diagnostics/camera.log.partial`은 불완전한 사본이며 전체 로그로 취급하지 않는다.
- Qt의 Wayland 관련 경고는 위 촬영 실패 원인에 대한 증거가 아니다.

## 수정

- 실패 manifest를 받은 즉시 터미널/앱에 촬영 중단 원인과 원본 회수 중임을 표시한다.
- 회수한 capture의 오류 packet은 원본 이벤트로 보존하고 나머지 영상·IMU도 가져온 뒤
  실패를 보고한다. 실패 촬영을 정상 저장/최종 맵 생성으로 표시하지 않는다.
- Pi의 camera.log 진단본도 체크섬 검증 대상에 포함한다. 오류 반복으로 로그가 커지면
  첫 부분/끝부분을 합쳐 256 KiB 이내로 제한하고 잘린 진단본임을 명시한다.
- 카메라 시작 검사는 최근 15개 수신 시각으로 12~18 FPS를 확인할 때까지 기다린다.
  초기 지연이 영구적으로 평균을 낮추지 않게 하되, 지속적으로 느린 카메라는 통과시키지 않는다.
  시간 초과 시 각 stream FPS와 USB/camera.log 확인 안내를 표시한다.
- 정상화되었다고 판단하기 전 카메라 USB 연결을 점검하고 다시 촬영해야 한다.
  실제 재촬영·USB reset·모터 명령은 이 조사에서 수행하지 않았다.

검증: 관련 기록·전송·카메라 검사·세션 테스트 **33 passed (5.78초)**,
변경 Python pyflakes 및 `git diff --check` 통과.
