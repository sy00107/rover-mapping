# ================================================================
# controller.py
# 설명: 노트북 촬영 세션과 소유 subprocess의 정상 종료·저장·TSDF를 관리한다.
# 사용법: from experiments.handheld_mapping.controller import Session
# ================================================================
from datetime import datetime
from collections import deque
import io
import json
import math
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import sys
import tarfile
import threading
import time
import uuid

from experiments.handheld_mapping.storage import RunWriter, atomic_json
from experiments.handheld_mapping.transport import PacketReader

ROOT = Path(__file__).resolve().parents[2]
ACTIVE = {"starting", "recording", "stopping", "finalizing", "transferring", "processing", "reconstructing", "comparing"}


def remote_command(host: str, run_id: str, sensors_only: bool = False, queue_mib: float = 4., field_metadata: bool = False) -> list[str]:
    if not re.fullmatch(r"[A-Za-z0-9_.@:-]+", host) or host.startswith("-"):
        raise ValueError("잘못된 SSH 연결 대상")
    if not math.isfinite(queue_mib) or not .25 <= queue_mib <= 64:
        raise ValueError("invalid queue MiB limit")
    remote = f"/dev/shm/rover_mapping_{run_id}"
    code = (
        "set -e; source /opt/ros/jazzy/setup.bash >&2; "
        "export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=67 RMW_IMPLEMENTATION=rmw_fastrtps_cpp RCUTILS_LOGGING_USE_STDOUT=0; "
        f"export ROS_LOG_DIR={shlex.quote(remote+'/logs')}; "
        "export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/handheld_mapping/live/fastdds_stereo_local.xml; "
        "export LD_LIBRARY_PATH=/home/pi/handheld_mapping/runtime/install/lib:/home/pi/handheld_mapping/runtime/prefix/usr/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}; "
        f"cd {shlex.quote(remote+'/code')}; "
        f"exec /usr/bin/python3 -m experiments.handheld_mapping.pi_worker --run-dir {shlex.quote(remote+'/run')}"
        f" --queue-mib {queue_mib}"+(" --sensors-only" if sensors_only else "")+(" --field-metadata" if field_metadata else "")
    )
    return ["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", "-o", "ServerAliveInterval=5",
            "-o", "ServerAliveCountMax=3", host, shlex.join(["bash", "-c", code])]


def deploy(host: str, run_id: str) -> None:
    # 명령의 host 검증을 공유하며 사용자 셸 문자열을 실행하지 않는다.
    remote_command(host, run_id)
    archive = io.BytesIO()
    with tarfile.open(fileobj=archive, mode="w:gz") as tar:
        for path in (ROOT / "experiments/handheld_mapping").glob("*.py"):
            tar.add(path, arcname=f"experiments/handheld_mapping/{path.name}")
        for name in ("prepare_mapping_run.py",):
            tar.add(ROOT / "ros2_ws/src/orbslam3_ros2/scripts" / name, arcname=f"helpers/{name}")
    remote = f"/dev/shm/rover_mapping_{run_id}/code"
    command = f"mkdir -p {shlex.quote(remote)} && tar -xzf - -C {shlex.quote(remote)}"
    subprocess.run(["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", host, command],
                   input=archive.getvalue(), check=True, timeout=20, stdout=subprocess.DEVNULL,
                   stderr=subprocess.PIPE)


class Session:
    """촬영·파일 기록·최종 지도 생성의 상태를 한 worker thread에서 관리한다."""

    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.status = {"state": "idle", "message": "준비", "counts": {}, "run_dir": "", "dropped": 0}
        self.process: subprocess.Popen | None = None
        self.thread: threading.Thread | None = None
        self.stop_requested = threading.Event()
        self.preview = None
        self.started_at = 0.
        self.ended_at = 0.
        self.generation = 0
        self.left_receipts = deque(maxlen=30)
        self.live = None
        self.imu = None

    def live_snapshot(self) -> dict | None:
        with self.lock:
            mapper = self.live
            error = self.status.get('live_error')
        return mapper.snapshot() if mapper is not None else ({'error':error} if error else None)

    def snapshot(self) -> dict:
        with self.lock:
            result = {**self.status, "counts": dict(self.status.get("counts", {}))}
            result["elapsed_s"] = (self.ended_at or time.monotonic())-self.started_at if self.started_at else 0.
            receipts = self.left_receipts
            result["left_receipt_fps"] = ((len(receipts)-1)/(receipts[-1]-receipts[0])
                if len(receipts) >= 2 and result["state"] == "recording" and time.monotonic()-receipts[-1] < 1 else 0.)
            imu = self.imu
        if imu is not None:
            result["imu"] = imu.snapshot()
        return result

    def update(self, **values) -> None:
        with self.lock:
            if values.get("state") in {"complete", "partial", "failed", "paused"} and not self.ended_at:
                self.ended_at = time.monotonic()
            self.status.update(values)

    def start(self, config: dict) -> None:
        config = dict(config)
        if config.get('inertial_fusion') and not config.get('demo'):
            config['imu_enabled'] = True
        with self.lock:
            if self.status["state"] in ACTIVE or (self.thread is not None and self.thread.is_alive()):
                raise RuntimeError("이미 실행 중입니다")
            self.stop_requested.clear()
            self.generation += 1
            self.process = None
            self.preview = None
            self.live = None
            self.imu = None
            self.left_receipts.clear()
            self.started_at = time.monotonic()
            self.ended_at = 0.
            self.status = {"state": "starting", "message": "연결·카메라 준비 중", "counts": {}, "run_dir": "", "dropped": 0}
            self.thread = threading.Thread(target=self._run, args=(dict(config),), daemon=False)
            self.thread.start()

    def stop(self) -> None:
        with self.lock:
            if self.status["state"] not in {"starting", "recording", "stopping"}:
                return
            first = not self.stop_requested.is_set()
            self.stop_requested.set()
            self.status.update(state="stopping", message="촬영 종료·최종 경로 저장 중")
            process = self.process
            if process is not None and process.poll() is None:
                try:
                    process.stdin.write(b"stop\n")
                    process.stdin.flush()
                except (BrokenPipeError, OSError):
                    pass
            if first:
                threading.Thread(target=self._stop_timeout, args=(self.generation, time.monotonic()+75), daemon=True).start()

    def pause_transfer(self) -> None:
        with self.lock:
            if not self.status.get('local_recording') or self.status['state'] not in {'stopping','finalizing','transferring'}:
                return
            process = self.process
            if process is not None and process.poll() is None:
                try:
                    process.stdin.write(b'pause\n'); process.stdin.flush()
                    self.status['message'] = '연결 대기·전송 보류 중 · 원본과 이어받기 정보는 보존됩니다'
                except (BrokenPipeError, OSError):
                    pass

    def resume_transfer(self, run_dir: Path) -> None:
        run = Path(run_dir).expanduser().resolve()
        saved = json.loads((run/'session.json').read_text()) if (run/'session.json').exists() else {}
        state = json.loads((run/'transfer_checkpoint.json').read_text())
        if state.get('phase') == 'captured' and saved.get('state') in {'complete','partial'}:
            raise ValueError('원본 전송이 이미 완료됐습니다. 3D 지도 보기 또는 기록에서 다시 생성을 사용하세요.')
        from .local_client import validate_resume_target
        validate_resume_target(run, state['host'], state['run_id'])
        options = saved.get('processing_options', {})
        self.start(dict(output_root=str(run.parent), resume_run=str(run), host=state['host'],
            imu_enabled=saved.get('imu_requested', False),
            gravity_enabled=saved.get('gravity_requested', False), **options))

    def _stop_timeout(self, generation: int, deadline: float) -> None:
        while time.monotonic() < deadline:
            with self.lock:
                if generation != self.generation or not self.stop_requested.is_set():
                    return
                process = self.process
                state = self.status["state"]
            if state not in {"starting", "recording", "stopping"}:
                return
            if process is not None and process.poll() is not None:
                return
            time.sleep(.2)
        with self.lock:
            if generation != self.generation or not self.stop_requested.is_set():
                return
            process = self.process
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)

    def _run(self, config: dict) -> None:
        writer = None
        run = None
        process = None
        log = None
        live = None
        packet_reader = None
        preview_writer = None
        local_recording = bool(not config.get('demo') and
                               config.get('local_recording', not config.get('worker_command')))
        transfer_complete = not local_recording
        importing_originals = False
        imu_partial = False
        try:
            gravity_requested = bool(config.get('gravity_enabled'))
            if gravity_requested and (not config.get('imu_enabled') or config.get('demo')):
                raise ValueError('중력 기준 지도에는 실제 보정 IMU 기록이 필요합니다')
            root = Path(config["output_root"]).expanduser().resolve()
            root.mkdir(parents=True, exist_ok=True)
            if shutil.disk_usage(root).free < 2_000_000_000:
                raise OSError("노트북 여유 공간이 2GB 미만입니다")
            resuming = bool(config.get('resume_run'))
            if resuming:
                run = Path(config['resume_run']).resolve()
                state = json.loads((run/'transfer_checkpoint.json').read_text())
                run_id = state['run_id']
                from .local_client import validate_resume_target
                validate_resume_target(run, config['host'], run_id)
                self.stop_requested.set()
            else:
                run_id = datetime.now().strftime("%Y%m%d_%H%M%S_")+uuid.uuid4().hex[:6]
                run = root / run_id
            writer = RunWriter(run, resume=resuming)
            self.update(run_dir=str(run), source="synthetic_demo" if config.get("demo") else "D435_Pi",
                        gravity_requested=gravity_requested, imu_requested=bool(config.get('imu_enabled')),
                        processing_options={k: bool(config.get(k)) for k in ('inertial_fusion','compare_lines')})
            atomic_json(run / "session.json", self.snapshot())
            if local_recording:
                if not resuming and not config.get('worker_command') and not (ROOT/'outputs/handheld_mapping/orb3_comparison_build/rover_orb3_compare').is_file():
                    raise FileNotFoundError('노트북 ORB 실행기가 없습니다: tools/build_mapping_baseline.sh')
                preview_writer = RunWriter(run/'preview', resume=resuming and (run/'preview').exists())
                if config.get('imu_enabled') and not resuming:
                    shutil.copyfile(Path(config['imu_calibration']).expanduser(), preview_writer.path/'imu_calibration.json')
                    shutil.copyfile(ROOT/'ros2_ws/src/rover_description/urdf/rocker_bogie_4wd_rover.urdf',
                                    preview_writer.path/'robot.urdf')
                self.update(local_recording=True, remote_run_dir=f'/home/pi/handheld_mapping/recordings/{run_id}')
            if config.get("demo"):
                command = [sys.executable, "-u", "-m", "experiments.handheld_mapping.demo"]
            elif config.get("worker_command"):
                command = config["worker_command"]
            else:
                if local_recording:
                    from .local_client import provision
                    if not resuming:
                        provision(config.get('host', 'pi@raspberrypi.local'), run_id, config)
                    command = [sys.executable, '-u', '-m', 'experiments.handheld_mapping.local_client',
                               '--host', config.get('host', 'pi@raspberrypi.local'),
                               '--run-id', run_id, '--run', str(run)]
                    if resuming:
                        command += ['--recover', '--binary']
                else:
                    deploy(config.get("host", "pi@raspberrypi.local"), run_id)
                    command = remote_command(config.get("host", "pi@raspberrypi.local"), run_id,
                                             sensors_only=config.get("sensors_only", False),
                                             queue_mib=config.get("queue_mib", 4.), field_metadata=config.get("field_metadata", False))
            if config.get('imu_enabled') and local_recording:
                from .local_client import RemoteImu
                def send_control(data):
                    with self.lock:
                        if self.process is None or self.process.poll() is not None:
                            raise RuntimeError('Pi 연결 대기 중입니다')
                        self.process.stdin.write(data)
                        self.process.stdin.flush()
                self.imu = RemoteImu(run, send_control)
            if config.get("imu_enabled") and not config.get("demo") and not local_recording:
                from experiments.handheld_mapping.imu import ImuCapture
                calibration = config.get("imu_calibration")
                if not calibration:
                    raise ValueError("IMU 보정 파일 경로가 필요합니다")
                imu = ImuCapture(config.get("host", "pi@raspberrypi.local"), run,
                                 Path(calibration).expanduser(),
                                 ROOT / "ros2_ws/src/rover_description/urdf/rocker_bogie_4wd_rover.urdf")
                with self.lock:
                    self.imu = imu
                self.update(message="보정 IMU의 최신 유효 샘플 대기 중")
                imu.start()
                self.update(imu=imu.snapshot(), message="IMU 기록 준비 완료 · 카메라 준비 중")
            if self.stop_requested.is_set() and not resuming:
                raise RuntimeError("시작 중 종료 요청: 촬영하지 않았습니다")
            log = (run / "worker.log").open("ab" if resuming else "wb")
            process = subprocess.Popen(command, cwd=ROOT, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                       stderr=log, start_new_session=True, bufsize=0)
            with self.lock:
                self.process = process
                if self.stop_requested.is_set():
                    process.stdin.write(b"stop\n")
                    process.stdin.flush()
            finished = None
            received = 0
            laptop_capture = {"packets": 0, "read_wait_ms_total": 0., "read_wait_ms_max": 0.,
                              "writer_accept_ms_total": 0., "writer_accept_ms_max": 0.,
                              "packet_process_ms_max": 0., "packet_process_ms_max_kind": None,
                              "packet_process_ms_max_stamp_ns": None}
            packet_reader = PacketReader(process.stdout)
            while True:
                if self.imu is not None:
                    imu_status = self.imu.snapshot()
                    if imu_status.get("error"):
                        if not local_recording:
                            raise RuntimeError("IMU 기록 오류: "+str(imu_status["error"]))
                        imu_partial = True
                        self.update(imu_warning='IMU 오류 · 영상 원본 회수는 계속합니다: '+str(imu_status['error']))
                    if imu_status.get("stale"):
                        self.update(imu_warning="IMU 표시 갱신 대기 · 노트북 수신 지연")
                    else:
                        self.update(imu_warning=None)
                packet = packet_reader.get()
                if packet is None:
                    break
                header, payload = packet
                kind = header['kind']
                if local_recording and kind in {'local_status', 'transfer_begin', 'transfer_progress',
                                                'originals_begin', 'transfer_complete', 'connection_wait', 'capture_failure',
                                                'stop_pending','stop_ack','capture_stopped','capture_quality','transport_paused','retention_status'}:
                    if kind == 'local_status':
                        self.update(pi_recording=header, counts=header.get('counts', {}), connection_warning=None)
                        captured = header.get('capture', {})
                        self.update(dropped=captured.get('dropped', 0),
                                    capture_metrics=captured.get('capture_metrics', {}))
                        warning = captured.get('stream_warning')
                        if self.snapshot()['state'] == 'recording':
                            if warning:
                                self.update(capture_warning=warning, message=warning)
                            elif self.snapshot().get('capture_warning'):
                                self.update(capture_warning=None, message='Pi 원본 기록 중 · 노트북 실시간 미리보기')
                        if self.imu is not None and header.get('imu'):
                            self.imu.update(header['imu'], header.get('gravity_samples', []))
                    elif kind == 'connection_wait':
                        message = ('Pi 연결 복구 대기 중 · 원본 저장 위치: '+header['remote_path'])
                        self.update(message=message, connection_warning=message)
                        print(message, flush=True)
                    elif kind == 'stop_pending':
                        self.update(state='stopping', message='Pi에 촬영 종료 요청 중 · 아직 종료가 확인되지 않았습니다')
                    elif kind == 'stop_ack':
                        self.update(state='stopping', message='Pi가 종료 요청을 받았습니다 · 영상 마감 대기 중')
                    elif kind == 'capture_stopped':
                        self.update(state='finalizing', remote_stop_confirmed=True,
                                    message='Pi 촬영 종료 확인 · 원본 파일 마감 중입니다')
                    elif kind == 'capture_quality':
                        self.update(capture_quality=header, capture_partial=bool(
                            self.snapshot().get('capture_partial') or header.get('partial')))
                        imu_partial = imu_partial or bool(header.get('imu_degraded'))
                    elif kind == 'transport_paused':
                        self.update(state='paused', message=header['message'],
                                    remote_stop_confirmed=header.get('remote_stop_confirmed',False))
                        print(header['message'], flush=True)
                    elif kind == 'capture_failure':
                        message = '촬영 오류로 중단: '+header['message']+' · 저장된 영상·IMU를 가져옵니다.'
                        self.update(state='transferring', capture_error=header['message'], message=message)
                        print(message, flush=True)
                    elif kind == 'transfer_begin':
                        message = ('촬영 오류 · 남아 있는 원본을 가져오는 중입니다. 기다려 주세요.'
                                   if self.snapshot().get('capture_error') else
                                   '촬영 종료 · Pi의 영상·IMU 원본을 노트북으로 옮기는 중입니다. 완료될 때까지 기다려 주세요.')
                        if self.snapshot()['state'] != 'transferring':
                            print(message, flush=True)
                        self.update(state='transferring', message=message)
                    elif kind == 'transfer_progress':
                        total = header['total']
                        percent = 100*header['bytes']/total if total else 100
                        message = f"원본 전송 {percent:.0f}% ({header['bytes']/1e6:.1f}/{total/1e6:.1f} MB) · 기다려 주세요."
                        if header.get('reconnecting'):
                            message += ' 연결 복구 후 이어받습니다.'
                        self.update(message=message, transfer=header)
                        print(message, flush=True)
                    elif kind == 'originals_begin':
                        importing_originals = True
                        self.update(state='transferring', message='원본 전송 검증 완료 · 영상 파일 정리 중입니다. 기다려 주세요.')
                        print(self.snapshot()['message'], flush=True)
                    elif kind == 'retention_status':
                        self.update(message=header['message'])
                        if header.get('result'):
                            self.update(pi_retention=header['result'])
                        print(header['message'], flush=True)
                    elif kind == 'transfer_complete':
                        if dict(writer.counts) != header.get('counts'):
                            raise RuntimeError('Pi 원본과 노트북 영상 개수가 다릅니다')
                        transfer_complete = bool(header.get('verified'))
                        self.update(transfer_verified=transfer_complete, connection_warning=None)
                        print('영상·IMU 원본 전송 및 검증 완료 · 최종 지도 계산을 시작합니다.', flush=True)
                    continue
                receipt_ns = header['laptop_receipt_monotonic_ns']
                read_ms = header['laptop_read_wait_ms']
                # Pi와 노트북 monotonic은 서로 다른 clock이다. read 대기는 latency가 아니다.
                header = {**header, "laptop_receipt_monotonic_ns": receipt_ns,
                          "laptop_read_wait_ms": read_ms}
                accept_start = time.monotonic_ns()
                active_writer = preview_writer if local_recording and not importing_originals else writer
                image = active_writer.accept(header, payload)
                accept_ms = (time.monotonic_ns()-accept_start)/1e6
                if header['kind'] == 'calibration':
                    self.update(capture_mode=header.get('capture_mode', 'pi_orb'),
                                source=header.get('source', self.snapshot()['source']))
                    if config.get('live_mapping') and not importing_originals:
                        try:
                            from experiments.handheld_mapping.live import LiveMapper
                            live_options = {'demo':bool(config.get('demo'))}
                            if gravity_requested:
                                live_options['gravity_provider'] = self.imu.gravity_samples
                            live = LiveMapper(active_writer.path, **live_options)
                            with self.lock:
                                self.live = live
                        except Exception as error:
                            self.update(live_error=str(error))
                if header['kind'] == 'pose' and live is not None and not importing_originals:
                    try:
                        live.offer_pose(header)
                    except Exception as error:
                        self.update(live_error=str(error))
                received += 1
                if received % 45 == 0 and shutil.disk_usage(run).free < 256_000_000:
                    raise OSError("노트북 공간 부족으로 촬영을 종료합니다")
                if header["kind"] == "ready" and not self.stop_requested.is_set():
                    self.update(state="recording", message=header.get("message", "촬영 중"))
                elif header["kind"] == "status":
                    self.update(tracking_state=header.get("tracking_state", -1), dropped=header.get("dropped", 0),
                                source_frames=header.get("source_frames", 0), transport_queue=header.get('queue', {}))
                    if 'capture_metrics' in header:
                        self.update(capture_metrics=header['capture_metrics'])
                elif header["kind"] == "error":
                    raise RuntimeError(header["message"])
                elif header["kind"] == "finished":
                    finished = header
                    self.update(**{target: header[source] for source, target in (
                        ('queue', 'transport_queue'), ('dropped', 'dropped'), ('source_frames', 'source_frames'),
                        ('capture_metrics', 'capture_metrics')) if source in header})
                if image is not None:
                    if live is not None and not importing_originals:
                        live.offer(header)
                    with self.lock:
                        if not local_recording or importing_originals:
                            self.status["counts"] = dict(writer.counts)
                        if header["stream"] == "left" and not importing_originals:
                            self.preview = image
                            self.left_receipts.append(receipt_ns/1e9)
                laptop_capture['packets'] = received
                for key, value in (("read_wait_ms", read_ms), ("writer_accept_ms", accept_ms)):
                    laptop_capture[key+'_total'] += value
                    laptop_capture[key+'_max'] = max(laptop_capture[key+'_max'], value)
                processing_ms = (time.monotonic_ns()-accept_start)/1e6
                if processing_ms > laptop_capture['packet_process_ms_max']:
                    laptop_capture.update(packet_process_ms_max=processing_ms,
                        packet_process_ms_max_kind=header['kind'],
                        packet_process_ms_max_stamp_ns=header.get('stamp_ns'))
                self.update(laptop_capture={**laptop_capture,
                    'receive_buffer': packet_reader.snapshot(),
                    'images': {side: dict(stats) for side, stats in writer.image_timings.items()}})
            exitcode = process.wait(timeout=10)
            imu_status = self._finish_imu()
            if self.snapshot()['state'] == 'paused':
                return
            if self.snapshot().get("imu_cleanup_error"):
                if not local_recording:
                    raise RuntimeError("IMU 기록 저장 실패: "+self.snapshot()["imu_cleanup_error"])
                imu_partial = True
            if imu_status:
                if imu_status.get("error"):
                    if not local_recording:
                        raise RuntimeError("IMU 기록 오류: "+str(imu_status["error"]))
                    imu_partial = True
                imu_partial = imu_partial or bool(imu_status.get('stale') or any(imu_status.get(key,0) for key in
                                   ('source_gap_events','resets','clock_jumps','tee_drops')))
                if not imu_status.get('stale'):
                    self.update(imu_warning=None)
            writer.close()
            writer = None
            if live is not None:
                try:
                    live.finish()
                    details = live.snapshot()
                    self.update(live={key:details[key] for key in ('processed','dropped_pairs','no_depth',
                                'error','map_id','lag_ms','map_changes','synthetic')})
                except Exception as error:
                    self.update(live_error=str(error))
                live = None
            if not transfer_complete:
                raise RuntimeError('Pi 원본 전송·검증이 완료되지 않았습니다. Pi 원본은 보존됩니다.')
            if exitcode != 0 or finished is None or finished.get("status") != "saved":
                raise RuntimeError(f"정상 종료·저장 확인 실패 (exit={exitcode}, finished={finished}) — worker.log 확인")
            backend = 'orb'
            if finished.get('capture_mode') == 'sensors_only':
                self.update(state='processing', message='노트북에서 원본으로 ORB 계산 중')
                from experiments.handheld_mapping.offline import run_point
                existing = None
                folder = run/'orb_offline'
                if resuming and folder.exists():
                    summary = folder/'summary.json'
                    if summary.exists() and (folder/'runner_complete.json').exists() and any(folder.glob('map_*_keyframes.csv')):
                        existing = json.loads(summary.read_text())
                    if existing is None:
                        folder.rename(run/('orb_offline_before_resume_'+uuid.uuid4().hex[:8]))
                self.update(offline=existing or run_point(run))
                backend = 'orb_offline'
            gravity = None
            if gravity_requested:
                self.update(state='reconstructing',message='정지 IMU로 지도별 중력 기준 계산 중')
                from experiments.handheld_mapping.gravity import process_maps
                gravity = process_maps(run,backend)
                self.update(gravity=gravity)
            self.update(state="reconstructing", message="최종 keyframe으로 TSDF 지도 생성 중")
            from experiments.handheld_mapping.reconstruct import reconstruct
            dense_options = {'backend':backend,'progress':lambda message:self.update(message=message)}
            if gravity_requested:dense_options['gravity_alignments'] = gravity['maps']
            dense = reconstruct(run,**dense_options)
            terrain = None
            if gravity_requested:
                self.update(message='중력 기준 높이·경사·단차 지도 생성 중')
                from experiments.handheld_mapping.gravity import terrain_maps
                terrain = terrain_maps(run,backend,gravity)
                self.update(terrain=terrain)
            self.update(dense=dense)
            inertial_partial = (self._run_inertial_postprocessing(run)
                                if config.get('inertial_fusion') and not config.get('demo') else False)
            line_partial = False
            if config.get("compare_lines"):
                self.update(state="comparing", message="동일 원본으로 점+선 SLAM 비교 중")
                from experiments.handheld_mapping.compare import run_orbline
                comparison = run_orbline(run)
                self.update(comparison=comparison)
                line_partial = comparison.get("dense", {}).get("status", "complete") != "complete"
            gravity_partial = gravity_requested and (gravity['status'] != 'complete' or terrain['status'] != 'complete')
            partial = (dense["status"] != "complete" or finished.get("dropped", 0) > 0
                       or self.snapshot().get('capture_partial', False)
                       or line_partial or imu_partial or gravity_partial or inertial_partial)
            message = ('중력·지형 결과 일부 미완료 · 원본과 3D 지도 저장 완료' if gravity_partial else
                       '일부 누락 있음 · 지도 저장 완료' if partial else '지도 저장 완료')
            self.update(state="partial" if partial else "complete", message=message,
                        dense=dense, dropped=finished.get("dropped", 0))
        except Exception as error:
            self.update(state="failed", error=str(error), message=str(error))
            if local_recording:
                print('매핑 실패: '+str(error), flush=True)
        finally:
            if packet_reader is not None:
                packet_reader.close()
            self._finish_imu()
            if live is not None:
                try:
                    live.finish()
                except Exception as error:
                    self.update(live_error=str(error))
            if process is not None:
                if process.poll() is None:
                    try:
                        process.stdin.close()
                        process.wait(timeout=3)
                    except (OSError, subprocess.TimeoutExpired):
                        os.killpg(process.pid, signal.SIGTERM)
                        try:
                            process.wait(timeout=3)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGKILL)
                            process.wait()
                process.stdout.close()
                if not process.stdin.closed:
                    process.stdin.close()
            if packet_reader is not None:
                packet_reader.thread.join(timeout=3)
            if writer is not None:
                writer.close()
            if preview_writer is not None:
                preview_writer.close()
            if log is not None:
                log.close()
            with self.lock:
                if not self.ended_at:
                    self.ended_at = time.monotonic()
            if run is not None:
                try:
                    atomic_json(run / "session.json", self.snapshot())
                except OSError as error:
                    self.update(state="failed", error=str(error), message="상태 파일 저장 실패: "+str(error))

    def _run_inertial_postprocessing(self, run: Path) -> bool:
        """기본 3D·지형을 보존한 뒤 선택형 IMU 융합을 실행하고 미완료 여부를 반환한다."""
        fusion_result = None
        try:
            self.update(state='processing', message='저장한 영상과 IMU의 시간 정렬·융합 시험 중')
            from experiments.handheld_mapping.inertial import run_inertial_auto
            fusion_result = run_inertial_auto(run)
            self.update(inertial_fusion=fusion_result)
            if not fusion_result.get('orb_fused'):
                self.update(inertial_warning=fusion_result.get('error') or 'IMU 초기화 미완료 · 기본 지도는 저장됨')
                return True
            self.update(state='reconstructing', message='IMU 융합 경로로 별도 3D 지도 생성 중')
            from experiments.handheld_mapping.reconstruct import reconstruct
            dense = reconstruct(run, backend='orb_inertial', progress=lambda message: self.update(message=message))
            self.update(inertial_dense=dense)
            if dense.get('status') != 'complete':
                self.update(inertial_warning='IMU 융합 지도 일부 누락 · 기본 지도는 저장됨')
                return True
            return False
        except Exception as error:
            if fusion_result is None:
                self.update(inertial_fusion={'status': 'failed', 'orb_fused': False, 'error': str(error)})
            self.update(inertial_warning='IMU 후처리 실패 · 기본 지도는 저장됨: '+str(error))
            return True

    def _finish_imu(self) -> dict | None:
        with self.lock:
            imu = self.imu
        if imu is None:
            return None
        try:
            summary = imu.stop()
            self.update(imu=summary)
            return summary
        except Exception as error:
            self.update(imu_cleanup_error=str(error))
            try:
                self.update(imu=imu.snapshot())
            except Exception:
                pass
            return None
        finally:
            with self.lock:
                self.imu = None

    def zero_imu_display(self) -> dict:
        with self.lock:
            imu = self.imu
        if imu is None:
            raise ValueError('IMU를 함께 기록하는 촬영 중에 평지에서 정지한 뒤 누르세요')
        result = imu.zero_display()
        self.update(message='롤·피치 참고 표시 영점 설정 완료')
        return result

    def rebuild(self, run_dir: Path) -> None:
        with self.lock:
            if self.status["state"] in ACTIVE or (self.thread is not None and self.thread.is_alive()):
                raise RuntimeError("촬영·저장 중에는 다시 생성할 수 없습니다")
            self.started_at = time.monotonic()
            self.ended_at = 0.
            self.status.update(state="reconstructing", run_dir=str(run_dir), message="원본에서 TSDF 다시 생성 중")
            self.thread = threading.Thread(target=self._rebuild, args=(Path(run_dir),), daemon=False)
            self.thread.start()

    def _rebuild(self, run_dir: Path) -> None:
        try:
            from experiments.handheld_mapping.reconstruct import reconstruct
            backend = next((name for name in ('orb_inertial','orb','orb_offline')
                            if any((run_dir/name).glob('map_*_keyframes.csv'))),'orb_offline')
            session_file = run_dir/'session.json'
            saved = json.loads(session_file.read_text()) if session_file.is_file() else {}
            gravity_requested = bool(saved.get('gravity_requested') or (run_dir/'gravity'/backend).is_dir())
            gravity = None
            if gravity_requested:
                from experiments.handheld_mapping.gravity import process_maps
                gravity = process_maps(run_dir,backend)
                self.update(gravity=gravity,gravity_requested=True)
            options = {'backend':backend,'progress':lambda message:self.update(message=message)}
            if gravity_requested:options['gravity_alignments'] = gravity['maps']
            result = reconstruct(run_dir,**options)
            terrain = None
            if gravity_requested:
                from experiments.handheld_mapping.gravity import terrain_maps
                terrain = terrain_maps(run_dir,backend,gravity)
                self.update(terrain=terrain)
            complete = result['status'] == 'complete' and (not gravity_requested or
                        (gravity['status'] == 'complete' and terrain['status'] == 'complete'))
            self.update(state="complete" if complete else "partial", message="지도 다시 생성 완료", dense=result)
        except Exception as error:
            self.update(state="failed", message=str(error), error=str(error))
        finally:
            with self.lock:
                if not self.ended_at:
                    self.ended_at = time.monotonic()
