"""Laptop transport for persistent Pi recording and verified, resumable copying."""
import argparse
import io
import json
import os
from pathlib import Path
import queue
import select
import shlex
import signal
import shutil
import subprocess
import sys
import tarfile
import threading
import time

from .local_recording import digest, run_path
from .protocol import read_packet, write_packet
from .storage import atomic_json


class CapturePaused(RuntimeError):
    """Laptop stopped waiting; never implies that the Pi accepted a stop request."""


def checkpoint(run, host, run_id, **updates):
    path = Path(run)/'transfer_checkpoint.json'
    state = json.loads(path.read_text()) if path.exists() else dict(schema=1, host=host, run_id=run_id,
        remote_path=str(run_path(run_id)), phase='recording', remote_stop_confirmed=False)
    if state.get('host') != host or state.get('run_id') != run_id:
        raise ValueError('이어받기 대상 host/촬영 ID가 기존 기록과 다릅니다')
    state.update(updates, updated_ns=time.time_ns())
    atomic_json(path, state)
    return state


def validate_resume_target(run, host, run_id):
    run = Path(run)
    if not run.exists():
        return
    if (run/'transfer_checkpoint.json').exists():
        state = json.loads((run/'transfer_checkpoint.json').read_text())
        if state.get('host') != host or state.get('run_id') != run_id:
            raise ValueError('이어받기 대상 host/촬영 ID가 기존 기록과 다릅니다')
        return
    manifest = run/'pi_originals/manifest.json'
    if manifest.exists() and json.loads(manifest.read_text()).get('remote_path') == str(run_path(run_id)):
        checkpoint(run, host, run_id, phase='fetching')
        return
    raise ValueError('기존 폴더의 Pi 촬영 ID를 확인할 수 없습니다. 원래 촬영 폴더나 새 폴더를 지정하세요.')


class ControlState:
    def __init__(self, controls):
        self.stopped, self.paused, self.closed = threading.Event(), threading.Event(), threading.Event()
        self.zeros = queue.Queue()
        self.stop_at = None
        def pump():
            while not self.closed.is_set():
                try:
                    control = controls.get(timeout=.1)
                except queue.Empty:
                    continue
                if control == 'stop':
                    if self.stop_at is None:
                        self.stop_at = time.monotonic()
                    self.stopped.set()
                elif control == 'pause':
                    self.paused.set()
                elif control == 'zero':
                    self.zeros.put(control)
        self.thread = threading.Thread(target=pump, daemon=True)
        self.thread.start()

    def close(self):
        self.closed.set()
        self.thread.join(timeout=.5)


def ssh_command(host, run_id, action, *options):
    from .controller import remote_command
    command = remote_command(host, run_id, sensors_only=True)
    shell = shlex.split(command[-1])[2]
    prefix = shell.rsplit('exec /usr/bin/python3', 1)[0]
    remote = ['/usr/bin/python3', '-u', '-m', 'experiments.handheld_mapping.local_recording',
              action, run_id, *options]
    command[-1] = shlex.join(['bash', '-c', prefix+'exec '+shlex.join(remote)])
    return command


def provision(host, run_id, config):
    from .controller import deploy, ROOT
    deploy(host, run_id)
    payload = io.BytesIO()
    with tarfile.open(fileobj=payload, mode='w') as archive:
        data = json.dumps({k: config.get(k, False) for k in ('imu_enabled', 'field_metadata', 'strict_imu')}).encode()
        info = tarfile.TarInfo('config.json'); info.size = len(data)
        archive.addfile(info, io.BytesIO(data))
        if config.get('imu_enabled'):
            from .imu import load_calibration
            source = Path(config['imu_calibration']).expanduser()
            load_calibration(source)
            archive.add(source, arcname='input_calibration.json')
            archive.add(ROOT/'ros2_ws/src/rover_description/urdf/rocker_bogie_4wd_rover.urdf',
                        arcname='input_robot.urdf')
    remote = shlex.quote(str(run_path(run_id)))
    command = ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', host,
               f'mkdir -p {remote} && tar -xf - -C {remote}']
    subprocess.run(command, input=payload.getvalue(), check=True, timeout=30,
                   stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)


class RemoteImu:
    """UI-only IMU snapshot; authoritative data is copied from Pi after stopping."""
    def __init__(self, run, send_control):
        self.run = Path(run)
        self.send_control = send_control
        self.lock = threading.RLock()
        self.state = {'state': 'starting', 'recorded': 0, 'valid': 0, 'unique': 0, 'stale': True}
        self.samples = []
        self.received = 0.

    def update(self, state, samples):
        with self.lock:
            self.state = dict(state)
            self.samples = list(samples)
            self.received = time.monotonic()

    def snapshot(self):
        with self.lock:
            state = dict(self.state)
            state['stale'] = bool(state.get('stale') or time.monotonic()-self.received > 1.5)
            return state

    def gravity_samples(self):
        with self.lock:
            return list(self.samples)

    def zero_display(self):
        if self.snapshot().get('stale'):
            raise ValueError('최신 IMU 수신 후 평지에서 3초 정지하세요')
        self.send_control(b'zero\n')
        return {'requested': True}

    def stop(self):
        path = self.run/'imu_summary.json'
        if path.exists():
            try:
                return json.loads(path.read_text())
            except ValueError:
                return dict(state='incomplete', error='IMU 요약 파일이 미완성입니다. 영상 원본은 보존됩니다.', stale=True)
        return self.snapshot()


def receive_file(entry, target, fetch, progress, retry_wait=2., cancel_event=None):
    """Resume partial bytes; expose a final file only after size and SHA256 match."""
    target = Path(target)
    if target.exists():
        if target.stat().st_size != entry['size'] or digest(target) != entry['sha256']:
            raise ValueError('기존 파일이 Pi 원본과 다릅니다: '+target.name)
        return
    partial = target.with_name(target.name+'.part')
    if partial.exists() and partial.stat().st_size > entry['size']:
        raise ValueError('전송 임시 파일이 원본보다 큽니다')
    attempts_without_progress = 0
    while True:
        if cancel_event is not None and cancel_event.is_set():
            raise CapturePaused('원본 전송을 보류했습니다. 같은 촬영 폴더에서 이어받을 수 있습니다.')
        offset = partial.stat().st_size if partial.exists() else 0
        if offset == entry['size']:
            # Empty files still need a materialized temporary file.
            partial.touch(exist_ok=True)
            if digest(partial) != entry['sha256']:
                raise ValueError('원본 전송 체크섬 불일치: '+entry['name'])
            partial.replace(target)
            return
        if shutil.disk_usage(target.parent).free < entry['size']-offset+256_000_000:
            raise OSError('노트북 공간 부족 · Pi 원본은 보존됩니다')
        process = fetch(entry['name'], offset)
        try:
            with partial.open('ab') as stream:
                while True:
                    if cancel_event is not None and cancel_event.is_set():
                        raise CapturePaused('원본 전송을 보류했습니다. Pi 원본과 부분 다운로드는 보존됩니다.')
                    if not select.select([process.stdout], [], [], .2)[0]:
                        continue
                    block = os.read(process.stdout.fileno(), 256*1024)
                    if not block:
                        break
                    if stream.tell()+len(block) > entry['size']:
                        raise ValueError('전송 파일 크기가 manifest와 다릅니다')
                    stream.write(block)
                    progress(stream.tell())
                stream.flush(); os.fsync(stream.fileno())
            code = process.wait(timeout=20)
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill(); process.wait()
            process.stdout.close()
        new_offset = partial.stat().st_size
        if code == 0 and new_offset == entry['size']:
            continue
        attempts_without_progress = attempts_without_progress+1 if new_offset == offset else 0
        progress(new_offset, reconnecting=True)
        # Leave verified predecessors and the partial file for explicit recovery too.
        if attempts_without_progress >= 30:
            raise ConnectionError('원본 전송 재연결 실패 · Pi 원본과 노트북 임시 파일을 보존했습니다')
        time.sleep(retry_wait)


def copy_originals(host, run_id, run, manifest, emit, cancel_event=None):
    run = Path(run)
    cache = run/'pi_originals'; cache.mkdir(exist_ok=True)
    atomic_json(cache/'manifest.json', manifest)
    files = manifest['files']
    names = [entry['name'] for entry in files]
    allowed_imu = {'imu.jsonl', 'imu_calibration.json', 'robot.urdf', 'imu_manifest.json',
                   'imu_summary.json', 'imu_display_events.jsonl'}
    if len(names) != len(set(names)):
        raise ValueError('duplicate recording manifest entry')
    for name in names:
        if name not in ('capture.bin', 'recorder.log', 'camera.log', 'imu_health.jsonl') and not (
                Path(name).parent == Path('imu')/run_id and Path(name).name in allowed_imu):
            raise ValueError('invalid recording manifest path')
    if 'capture.bin' not in names:
        raise ValueError('Pi 원본 영상 파일이 없습니다: '+str(manifest.get('error')))
    if manifest.get('imu_required'):
        required = {f'imu/{run_id}/{name}' for name in allowed_imu-{'imu_display_events.jsonl'}}
        if not required.issubset(names):
            emit({'kind': 'capture_quality', 'partial': True, 'imu_degraded': True,
                  'issues': ['Pi IMU 파일 일부 누락 · 남아 있는 영상과 IMU부터 회수합니다']})
    total = sum(e['size'] for e in files)
    done = 0
    last_report = 0.
    def fetch(name, offset):
        return subprocess.Popen(ssh_command(host, run_id, 'fetch', '--file', name, '--offset', str(offset)),
                                stdout=subprocess.PIPE, stderr=sys.stderr, bufsize=0)
    for entry in files:
        def progress(current, reconnecting=False):
            nonlocal last_report
            if time.monotonic()-last_report >= 1 or reconnecting:
                emit({'kind': 'transfer_progress', 'bytes': done+current, 'total': total,
                      'reconnecting': reconnecting})
                last_report = time.monotonic()
        name = Path(entry['name']).name
        target = cache/name
        receive_file(entry, target, fetch, progress, cancel_event=cancel_event)
        if name in allowed_imu:
            # Keep the downloaded originals as well as the canonical processing files.
            destination = run/name
            if destination.exists():
                if digest(destination) != entry['sha256']:
                    raise ValueError('노트북 IMU 파일이 Pi 원본과 다릅니다')
            else:
                shutil.copyfile(target, destination)
        done += entry['size']
    emit({'kind': 'transfer_progress', 'bytes': total, 'total': total})
    return cache/'capture.bin'


def transport(host, run_id, run, emit, controls, recover=False, connection_timeout=75.):
    from .transfer_monitor import TransferReporter
    reporter = TransferReporter(run)
    original_emit = emit
    def emit(header, payload=b''):
        reporter.event(header)
        original_emit(header, payload)
    state = ControlState(controls)
    checkpoint(run, host, run_id)
    try:
        _transport(host, run_id, run, emit, state, recover, connection_timeout)
    except CapturePaused as error:
        previous = checkpoint(run, host, run_id)
        saved = checkpoint(run, host, run_id, phase='paused', previous_phase=previous['phase'],
                           stop_requested=state.stopped.is_set())
        message = str(error)
        if not saved.get('remote_stop_confirmed'):
            message += ' Pi 촬영 종료는 아직 확인되지 않았습니다.'
        emit({'kind': 'transport_paused', 'message': message, **saved})
    except Exception as error:
        reporter.event({'kind': 'error', 'message': str(error)})
        raise
    finally:
        state.close()


def _transport(host, run_id, run, emit, controls, recover, connection_timeout):
    """Reconnect preview independently of the persistent recording service."""
    stopped = controls.stopped
    sent_stop = False
    calibrated = False
    prepared = False
    last_stamps = {}
    manifest = None
    saved = checkpoint(run, host, run_id)
    cached_manifest = Path(run)/'pi_originals/manifest.json'
    if (cached_manifest.exists() and saved.get('manifest_sha256') == digest(cached_manifest)):
        manifest = json.loads(cached_manifest.read_text())
    last_activity = time.monotonic()
    while manifest is None:
        if controls.paused.is_set():
            raise CapturePaused('연결 대기를 보류했습니다. 촬영 폴더에서 원본 회수를 재개할 수 있습니다.')
        process = subprocess.Popen(ssh_command(host, run_id, 'attach' if recover else 'start'), stdin=subprocess.PIPE,
                                   stdout=subprocess.PIPE, stderr=sys.stderr, bufsize=0)
        packets = queue.Queue(maxsize=8)
        cancelled = threading.Event()
        def read(process, packets, cancelled):
            try:
                while (packet := read_packet(process.stdout)) is not None:
                    while not cancelled.is_set():
                        try:
                            packets.put(packet, timeout=.2)
                            break
                        except queue.Full:
                            continue
                    if cancelled.is_set():
                        return
            except Exception as error:
                # The outer loop reports disconnect and reconnects.
                print('미리보기 연결 종료: '+str(error), file=sys.stderr, flush=True)
            finally:
                while not cancelled.is_set():
                    try:
                        packets.put(None, timeout=.2)
                        break
                    except queue.Full:
                        continue
        reader = threading.Thread(target=read, args=(process, packets, cancelled), daemon=True)
        reader.start()
        try:
            sent_stop = False
            while True:
                if controls.paused.is_set():
                    raise CapturePaused('연결 대기를 보류했습니다. 촬영 폴더에서 회수를 재개할 수 있습니다.')
                if (time.monotonic()-last_activity > connection_timeout or
                        (controls.stop_at is not None and time.monotonic()-controls.stop_at > connection_timeout)):
                    raise CapturePaused('Pi 응답 대기 시간이 지났습니다. 연결 복구 후 같은 폴더에서 재개하세요.')
                try:
                    controls.zeros.get_nowait()
                    process.stdin.write(b'zero\n'); process.stdin.flush()
                except queue.Empty:
                    pass
                if stopped.is_set() and not sent_stop:
                    checkpoint(run, host, run_id, phase='stop_requested', stop_requested=True)
                    emit({'kind': 'stop_pending', 'remote_path': str(run_path(run_id))})
                    sent_stop = True
                    try:
                        process.stdin.write(b'stop\n'); process.stdin.flush()
                    except (BrokenPipeError, OSError):
                        # A completed recorder may close stdin after publishing done.
                        # Drain that completion packet before deciding to reconnect.
                        pass
                try:
                    packet = packets.get(timeout=.1)
                except queue.Empty:
                    if process.poll() is not None:
                        break
                    continue
                if packet is None:
                    break
                header, payload = packet
                last_activity = time.monotonic()
                kind = header['kind']
                if kind == 'local_done':
                    manifest = header
                    break
                if kind == 'capture_stopped':
                    checkpoint(run, host, run_id, phase='finalizing', remote_stop_confirmed=True)
                if kind == 'calibration':
                    if calibrated:
                        continue
                    calibrated = True
                elif kind == 'ready':
                    if prepared:
                        continue
                    prepared = True
                elif kind == 'image':
                    side, stamp = header['stream'], header['stamp_ns']
                    if stamp <= last_stamps.get(side, -1):
                        continue
                    last_stamps[side] = stamp
                emit(header, payload)
        except (BrokenPipeError, OSError):
            pass
        finally:
            cancelled.set()
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill(); process.wait()
            process.stdin.close(); process.stdout.close()
            reader.join(timeout=2)
        if manifest is None:
            emit({'kind': 'connection_wait', 'stopping': stopped.is_set(),
                  'remote_path': str(run_path(run_id))})
            time.sleep(2)
            if controls.paused.is_set() or time.monotonic()-last_activity > connection_timeout:
                raise CapturePaused('Pi 재연결 대기를 보류했습니다. 같은 촬영 폴더에서 재개하세요.')
    checkpoint(run, host, run_id, phase='fetching', remote_stop_confirmed=True)
    if manifest.get('error'):
        emit({'kind': 'capture_failure', 'message': manifest['error'],
              'counts': manifest.get('counts', {}), 'remote_path': str(run_path(run_id))})
    emit({'kind': 'transfer_begin', 'remote_path': str(run_path(run_id)),
          'bytes': 0, 'total': sum(entry['size'] for entry in manifest.get('files', []))})
    quality = manifest.get('quality', {})
    if quality:
        emit({'kind': 'capture_quality', **quality})
    cache = Path(run)/'pi_originals'
    cache.mkdir(exist_ok=True)
    atomic_json(cache/'manifest.json', manifest)
    checkpoint(run, host, run_id, manifest_sha256=digest(cache/'manifest.json'))
    capture = copy_originals(host, run_id, run, manifest, emit, cancel_event=controls.paused)
    checkpoint(run, host, run_id, phase='importing')
    emit({'kind': 'originals_begin', 'counts': manifest.get('counts', {}), 'verified': True})
    finished = None
    with capture.open('rb') as stream:
        while (packet := read_packet(stream)) is not None:
            header, payload = packet
            if controls.paused.is_set():
                raise CapturePaused('영상 파일 정리를 보류했습니다. 같은 폴더에서 중복 없이 재개합니다.')
            if header['kind'] in ('ready', 'status'):
                # Recording UI must not be re-enabled while original files are imported.
                continue
            header = {**header, 'delivery_mode': 'post_capture_import'}
            if header['kind'] == 'error':
                # Preserve the failure event, finish importing saved frames/IMU,
                # then report failure with the camera diagnostics from manifest.
                header['kind'] = 'capture_error'
            if header['kind'] == 'finished':
                finished = header
            emit(header, payload)
    if manifest.get('recovered') and (finished is None or finished.get('status') != 'saved'):
        emit({'kind': 'finished', 'status': 'saved', 'capture_mode': 'sensors_only',
              'recovered': True, 'dropped': 0, 'quality': quality})
    if manifest.get('error'):
        raise RuntimeError('Pi 기록 오류 · 가져온 원본은 보존됨: '+manifest['error'])
    from .retention_client import finalize_retention
    retention = finalize_retention(host, run_id, run, manifest, emit, controls.paused)
    checkpoint(run, host, run_id, phase='captured', transfer_verified=True, pi_retention=retention)
    emit({'kind': 'transfer_complete', 'verified': True, 'counts': manifest.get('counts', {}), 'quality': quality})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', required=True)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--recover', action='store_true', help='기존 Pi 세션을 정지하고 원본 이어받기')
    parser.add_argument('--binary', action='store_true', help='앱에서 복구 packet을 수신할 때 사용')
    args = parser.parse_args()
    controls = queue.Queue()
    for signum in (signal.SIGTERM, signal.SIGINT):
        signal.signal(signum, lambda *_: controls.put('pause'))
    if args.recover:
        controls.put('stop')
    def input_controls():
        for line in sys.stdin:
            controls.put(line.strip())
    threading.Thread(target=input_controls, daemon=True).start()
    def binary_emit(header, payload=b''):
        write_packet(sys.stdout.buffer, header, payload)
    emit = binary_emit
    writer = None
    original_mode = False
    if args.recover and not args.binary:
        from .storage import RunWriter
        validate_resume_target(args.run, args.host, args.run_id)
        writer = RunWriter(args.run, resume=args.run.exists())
        def recovery_emit(header, payload=b''):
            nonlocal original_mode
            kind = header['kind']
            if kind == 'originals_begin':
                original_mode = True
                print('원본 검증 완료 · 영상 정리 중입니다. 기다려 주세요.', flush=True)
            elif kind == 'transfer_complete':
                if writer.counts != header['counts']:
                    raise ValueError('복구 영상 개수가 Pi 원본과 다릅니다')
                atomic_json(args.run/'session.json', dict(state='captured', source='D435_Pi',
                    transfer_verified=True, counts=writer.counts, remote_run_id=args.run_id,
                    message='원본 복구 완료 · 최종 지도 재생성 가능'))
                print('영상·IMU 원본 복구 완료: '+str(args.run), flush=True)
            elif kind in ('transfer_begin', 'transfer_progress', 'connection_wait', 'stop_pending',
                          'stop_ack', 'capture_stopped', 'capture_quality', 'transport_paused', 'retention_status'):
                print('원본 전송 대기 중 · '+json.dumps(header, ensure_ascii=False), flush=True)
            elif kind == 'error':
                print(header['message'], file=sys.stderr, flush=True)
            elif kind == 'capture_failure':
                print('촬영 오류: '+header['message']+' · 남아 있는 원본을 가져옵니다.', flush=True)
            elif original_mode:
                writer.accept(header, payload)
        emit = recovery_emit
    try:
        transport(args.host, args.run_id, args.run, emit, controls, recover=args.recover)
    except Exception as error:
        emit({'kind': 'error', 'message': str(error)})
        raise
    finally:
        if writer is not None:
            writer.close()


if __name__ == '__main__':
    main()
