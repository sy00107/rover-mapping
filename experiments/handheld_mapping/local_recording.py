"""Pi persistent recorder, disposable preview and reconnectable SSH attachment.

The camera's stdout is drained to disk, never to a network socket. SSH only
reads replaceable preview snapshots. Verified retention keeps the latest recording and removes only proven older copies.
"""
import argparse
from collections import OrderedDict, deque
import copy
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import zlib

from .protocol import read_packet, write_packet
from .storage import atomic_json, decode_image
from .capture_integrity import CaptureIntegrity
from .recording_recovery import owner_alive, process_identity, recover_abandoned

RECORDINGS = Path('/home/pi/handheld_mapping/recordings')


def run_path(run_id):
    import re
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,79}', run_id):
        raise ValueError('invalid recording ID')
    return RECORDINGS/run_id


def digest(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            result.update(block)
    return result.hexdigest()


def capture_failure_reason(errors, camera_log=''):
    """Surface recorded device failures without claiming a specific physical cause."""
    reason = '; '.join(dict.fromkeys(errors)) or None
    if not reason:
        return None
    if 'The device has been disconnected!' in camera_log or 'No such device' in camera_log:
        reason = '카메라 USB 연결 끊김 · USB 케이블·포트·전원을 확인하세요. ('+reason+')'
    elif 'Depth stream start failure' in camera_log:
        reason = '카메라 깊이 영상 시작 실패 · USB 연결을 확인하세요. ('+reason+')'
    if 'Device USB type: 2.1' in camera_log:
        reason += ' [카메라 USB 2.1 연결로 감지됨]'
    return reason


def collect_camera_diagnostics(source, target, limit=256*1024):
    """USB error floods must not turn diagnostic transfer into a huge download."""
    source, target = Path(source), Path(target)
    with source.open('rb') as stream:
        if source.stat().st_size <= limit:
            data = stream.read()
        else:
            prefix = stream.read(16*1024)
            marker = b'\n[diagnostic log truncated: first and last bytes retained]\n'
            tail_size = limit-len(prefix)-len(marker)
            stream.seek(-tail_size, os.SEEK_END)
            data = prefix+marker+stream.read(tail_size)
    target.write_bytes(data)
    return data.decode(errors='replace')


def preview_calibration(header):
    result = copy.deepcopy(header)
    def scale(info, center_offset=.25):
        info = copy.deepcopy(info)
        info['width'] //= 2
        info['height'] //= 2
        # cv2.resize maps destination pixel centers back to source centers.
        for key, stride in [('k', 3), ('p', 4)]:
            if key not in info:
                continue
            values = info[key]
            for i in range(2*stride):
                values[i] *= .5
            values[2] -= center_offset
            values[stride+2] -= center_offset
        return info
    # Nearest-neighbour depth selects source pixels (2*u, 2*v).
    result['depth'] = scale(result['depth'], center_offset=0.)
    for side in ('left', 'right'):
        result['stereo'][side] = scale(result['stereo'][side])
        original = result['stereo'].get('source_camera_info', {}).get(side)
        if original:
            result['stereo']['source_camera_info'][side] = scale(original)
    result.update(preview=True, sensor_profile='320x240x15_preview', capture_mode='sensors_only')
    result['stereo']['camera_profile'] = '320x240x15:Y8'
    return result


class Preview:
    """Bounded latest stereo pair; depth is optional and never gates stereo."""
    def __init__(self, path):
        self.path = Path(path)
        self.lock = threading.Lock()
        self.pending = {s: OrderedDict() for s in ('left', 'right', 'depth')}
        self.latest = None
        self.closed = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self.work, daemon=True)
        self.thread.start()

    def offer(self, header, payload):
        side, stamp = header['stream'], header['stamp_ns']
        with self.lock:
            own = self.pending[side]
            own[stamp] = (dict(header), payload)
            while len(own) > 4:
                own.popitem(last=False)
            if side == 'depth':
                return
            other = self.pending['right' if side == 'left' else 'left']
            if stamp in other:
                pair = [self.pending[s].pop(stamp) for s in ('left', 'right')]
                self.latest = pair

    def work(self):
        sequence = 0
        last_depth = -1
        try:
            import cv2
            while not self.closed.wait(.02):
                with self.lock:
                    pair, self.latest = self.latest, None
                    depths = list(self.pending['depth'].values())
                if pair is None:
                    continue
                stamp = pair[0][0]['stamp_ns']
                # Depth is not required, but allow its usual callback to arrive.
                self.closed.wait(.008)
                with self.lock:
                    depths = list(self.pending['depth'].values())
                depth = min(depths, key=lambda p: abs(p[0]['stamp_ns']-stamp), default=None)
                items = []
                if depth and abs(depth[0]['stamp_ns']-stamp) <= 2_000_000 and stamp-last_depth >= 500_000_000:
                    items.append(depth)
                    last_depth = stamp
                items.extend(pair)
                buffer = io.BytesIO()
                for header, payload in items:
                    image = decode_image(header, payload)
                    width, height = header['width']//2, header['height']//2
                    image = cv2.resize(image, (width, height), interpolation=(
                        cv2.INTER_NEAREST if header['stream'] == 'depth' else cv2.INTER_AREA))
                    h = {**header, 'preview': True, 'width': width, 'height': height,
                         'step': width*(2 if header['stream'] == 'depth' else 1),
                         'is_bigendian': False}
                    if header['stream'] == 'depth':
                        # Depth keeps integer millimetres and is never JPEG encoded.
                        h['compression'] = 'zlib'
                        data = zlib.compress(image.astype('<u2').tobytes(), 1)
                    else:
                        h['compression'] = 'jpeg'
                        ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 65])
                        if not ok:
                            raise OSError('preview JPEG encoding failed')
                        data = encoded.tobytes()
                    write_packet(buffer, h, data)
                sequence += 1
                temporary = self.path/'preview.tmp'
                temporary.write_bytes(buffer.getvalue())
                temporary.replace(self.path/'preview.bin')
                atomic_json(self.path/'preview_seq.json', {'seq': sequence})
        except Exception as error:
            self.error = str(error)

    def close(self):
        self.closed.set()
        self.thread.join(timeout=5)


def record(path, worker_command=None, imu_command=None, min_free=512_000_000):
    from .recording_retention import run_lock
    path = Path(path)
    path.mkdir(parents=True, exist_ok=True)
    with run_lock(path.parent, path.name):
        return _record(path, worker_command, imu_command, min_free)


def _record(path, worker_command=None, imu_command=None, min_free=512_000_000):
    """Detached supervisor. A disk/IMU failure stops capture, a preview failure does not."""
    import shutil
    import fcntl
    from .imu import ImuCapture
    path = Path(path)
    path.mkdir(parents=True, exist_ok=True)
    # A second disconnected session must not acquire the same camera/IMU source.
    owner = (path.parent/'.capture.lock').open('a')
    try:
        fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        owner.close()
        atomic_json(path/'done.json', {'error': '다른 Pi 로컬 촬영이 실행 중입니다', 'files': []})
        return
    preview = Preview(path)
    process = imu = None
    errors = []
    stop_monitor = threading.Event()
    counts = {s: 0 for s in ('left', 'right', 'depth')}
    integrity = CaptureIntegrity()
    imu_degraded = False
    previous_imu_health = None
    source_status = {}
    final = None
    last_sync = time.monotonic()
    log = (path/'recorder.log').open('ab')
    config = json.loads((path/'config.json').read_text())
    health_log = (path/'imu_health.jsonl').open('a', buffering=1)
    code = Path(__file__).resolve().parents[2]
    def monitor():
        nonlocal source_status, imu_degraded, previous_imu_health
        history = deque(maxlen=9)
        while not stop_monitor.wait(.25):
            try:
                history.append((time.monotonic(), dict(counts)))
                elapsed = history[-1][0]-history[0][0]
                fps = {side: (counts[side]-history[0][1][side])/elapsed if elapsed else 0.
                       for side in counts}
                state = imu.snapshot() if imu else None
                health = ('error' if state.get('error') else 'stale' if state.get('stale') else 'fresh') if state else 'off'
                if health != previous_imu_health:
                    health_log.write(json.dumps(dict(pi_monotonic_ns=time.monotonic_ns(), state=health,
                        error=state.get('error') if state else None))+'\n')
                    previous_imu_health = health
                if health in ('error', 'stale'):
                    imu_degraded = True
                    if config.get('strict_imu'):
                        errors.append('Pi IMU 기록 중단: '+str(state.get('error', 'fresh sample timeout')))
                        (path/'stop').touch()
                if shutil.disk_usage(path).free < min_free:
                    errors.append('Pi 저장장치 여유 공간 부족')
                    (path/'stop').touch()
                if (path/'zero_imu').exists() and imu:
                    (path/'zero_imu').unlink()
                    try:
                        imu.zero_display()
                    except ValueError as error:
                        source_status = {**source_status, 'imu_zero_error': str(error)}
                atomic_json(path/'status.json', dict(kind='local_status', preview=True,
                    counts=dict(counts), disk_free_bytes=shutil.disk_usage(path).free,
                    recording_fps=fps,
                    imu=state, gravity_samples=imu.gravity_samples()[-240:] if imu else [],
                    capture=source_status, preview_error=preview.error, recording=True,
                    imu_degraded=imu_degraded))
                if (path/'stop').exists() and process and process.poll() is None:
                    try:
                        process.stdin.write(b'stop\n')
                        process.stdin.flush()
                    except (BrokenPipeError, OSError):
                        pass
                    return
            except Exception as error:
                errors.append(str(error))
                if process and process.poll() is None:
                    process.terminate()
                return
    monitor_thread = None
    try:
        if shutil.disk_usage(path).free < min_free:
            raise OSError('Pi 저장장치 여유 공간 부족')
        if config.get('imu_enabled'):
            imu_dir = path/'imu'/path.name
            imu = ImuCapture('localhost', imu_dir, path/'input_calibration.json', path/'input_robot.urdf',
                command=imu_command or [sys.executable, '-u', '-m',
                    'experiments.handheld_mapping.imu_subscriber'], receipt_device='pi')
            try:
                imu.start()
            except Exception:
                imu_degraded = True
                if config.get('strict_imu'):
                    raise
        command = worker_command or [sys.executable, '-u', '-m', 'experiments.handheld_mapping.pi_worker',
            '--run-dir', str(code.parent/'run'), '--sensors-only', '--independent-streams', '--queue-mib', '32']
        if config.get('field_metadata') and worker_command is None:
            command += ['--field-metadata']
        process = subprocess.Popen(command, cwd=code, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                   stderr=log, start_new_session=True, bufsize=0)
        atomic_json(path/'children.json', [identity for child in (process, imu.process if imu else None)
                    if child and (identity := process_identity(child.pid)) is not None])
        monitor_thread = threading.Thread(target=monitor, daemon=True)
        monitor_thread.start()
        with (path/'capture.bin').open('xb') as archive:
            while True:
                packet = read_packet(process.stdout)
                if packet is None:
                    break
                header, payload = packet
                write_packet(archive, header, payload)
                kind = header['kind']
                if kind == 'calibration':
                    atomic_json(path/'preview_calibration.json', preview_calibration(header))
                elif kind == 'image':
                    integrity.offer(header)
                    counts[header['stream']] += 1
                    preview.offer(header, payload)
                elif kind == 'status':
                    source_status = header
                elif kind == 'ready':
                    (path/'ready').touch()
                elif kind == 'finished':
                    final = header
                elif kind == 'error':
                    errors.append(header['message'])
                if time.monotonic()-last_sync > 2:
                    archive.flush()
                    os.fsync(archive.fileno())
                    last_sync = time.monotonic()
            archive.flush()
            os.fsync(archive.fileno())
        if process.wait(timeout=20) or not final or final.get('status') != 'saved':
            errors.append('Pi 원본 촬영이 정상 종료되지 않았습니다')
    except Exception as error:
        errors.append(str(error))
    finally:
        stop_monitor.set()
        if monitor_thread:
            monitor_thread.join(timeout=3)
        if process:
            if process.poll() is None:
                from .pi_worker import stop_child
                try:
                    stop_child(process, 15)
                except Exception as error:
                    errors.append(str(error))
            process.stdin.close()
            process.stdout.close()
        atomic_json(path/'capture_stopped.json', dict(counts=counts, pi_monotonic_ns=time.monotonic_ns()))
        if imu:
            try:
                summary = imu.stop()
                if summary.get('error'):
                    imu_degraded = True
                    if config.get('strict_imu'):
                        errors.append('Pi IMU: '+str(summary['error']))
            except Exception as error:
                imu_degraded = True
                if config.get('strict_imu'):
                    errors.append(str(error))
        preview.close()
        log.close()
        health_log.close()
        files = []
        candidates = [path/'capture.bin', path/'recorder.log', path/'imu_health.jsonl']
        camera_log = code.parent/'run/camera.log'
        diagnostics = ''
        if camera_log.is_file():
            diagnostics = collect_camera_diagnostics(camera_log, path/'camera.log')
            candidates.append(path/'camera.log')
        if imu:
            candidates.extend(sorted((path/'imu'/path.name).glob('*')))
        for f in candidates:
            if f.is_file():
                with f.open('rb') as stream:
                    os.fsync(stream.fileno())
                files.append(dict(name=str(f.relative_to(path)), size=f.stat().st_size, sha256=digest(f)))
        quality = integrity.result()
        quality.update(imu_degraded=imu_degraded, partial=quality['partial'] or imu_degraded)
        atomic_json(path/'done.json', dict(files=files, counts=counts, finished=final, quality=quality,
                    imu_required=bool(config.get('imu_enabled')),
                    error=capture_failure_reason(errors, diagnostics), remote_path=str(path)))
        owner.close()


def attach(path):
    """Network client is expendable. EOF/disconnect never stops the recorder."""
    path = Path(path)
    def controls():
        for line in sys.stdin:
            if line.strip() == 'stop':
                (path/'stop').touch()
            elif line.strip() == 'zero':
                (path/'zero_imu').touch()
    threading.Thread(target=controls, daemon=True).start()
    calibrated = False
    ready = False
    previous = None
    heartbeat = 0.
    stop_ack = camera_stopped = False
    while True:
        if (path/'done.json').exists():
            write_packet(sys.stdout.buffer, {'kind': 'local_done', **json.loads((path/'done.json').read_text())})
            return
        started = path/'started'
        marker = json.loads(started.read_text()) if started.exists() else {}
        if marker.get('pid') and not owner_alive(marker):
            manifest = recover_abandoned(path)
            if manifest is not None:
                write_packet(sys.stdout.buffer, {'kind': 'local_done', **manifest})
                return
        if not stop_ack and (path/'stop').exists():
            write_packet(sys.stdout.buffer, {'kind': 'stop_ack'})
            stop_ack = True
        if not camera_stopped and (path/'capture_stopped.json').exists():
            write_packet(sys.stdout.buffer, {'kind': 'capture_stopped'})
            camera_stopped = True
        calibration = path/'preview_calibration.json'
        if not calibrated and calibration.exists():
            write_packet(sys.stdout.buffer, json.loads(calibration.read_text()))
            calibrated = True
        if calibrated and not ready and (path/'ready').exists():
            write_packet(sys.stdout.buffer, {'kind': 'ready', 'preview': True,
                'message': 'Pi 원본 기록 중 · 노트북 실시간 미리보기'})
            ready = True
        sequence = path/'preview_seq.json'
        if calibrated and sequence.exists():
            seq = json.loads(sequence.read_text())['seq']
            if seq != previous:
                # Atomic replacement makes an already opened snapshot immutable.
                with (path/'preview.bin').open('rb') as stream:
                    while (packet := read_packet(stream)) is not None:
                        write_packet(sys.stdout.buffer, *packet)
                previous = seq
        if time.monotonic()-heartbeat > .5:
            status = path/'status.json'
            if status.exists():
                write_packet(sys.stdout.buffer, json.loads(status.read_text()))
            heartbeat = time.monotonic()
        time.sleep(.02)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['start', 'record', 'attach', 'fetch', 'catalog', 'prune'])
    parser.add_argument('run_id')
    parser.add_argument('--offset', type=int, default=0)
    parser.add_argument('--file')
    args = parser.parse_args()
    path = run_path(args.run_id)
    if args.action in {'catalog', 'prune'}:
        from .recording_retention import catalog, prune
        if args.action == 'catalog':
            result = catalog(RECORDINGS)
        else:
            request = sys.stdin.buffer.read(1024*1024+1)
            if len(request) > 1024*1024:
                raise ValueError('oversized cleanup request')
            result = prune(RECORDINGS, json.loads(request))
        print(json.dumps(result), flush=True)
    elif args.action == 'record':
        record(path)
    elif args.action == 'start':
        if not (path/'started').exists():
            # The per-recording directory is provisioned by laptop deploy first.
            with (path/'started').open('x') as marker:
                json.dump({'created_ns': time.time_ns()}, marker)
            with (path/'supervisor.log').open('ab') as log:
                process = subprocess.Popen([sys.executable, '-u', '-m',
                    'experiments.handheld_mapping.local_recording', 'record', args.run_id],
                    stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)
            atomic_json(path/'started', {**(process_identity(process.pid) or {'pid': process.pid}),
                                        'created_ns': time.time_ns()})
        attach(path)
    elif args.action == 'attach':
        attach(path)
    else:
        manifest = (json.loads((path/'done.json').read_text()) if (path/'done.json').exists()
                    else recover_abandoned(path))
        if manifest is None:
            raise RuntimeError('Pi 원본 촬영 마감 전에는 fetch할 수 없습니다')
        entry = next((f for f in manifest['files'] if f['name'] == args.file), None)
        if entry is None or not 0 <= args.offset <= entry['size']:
            raise ValueError('file or offset outside finalized recording')
        source = (path/args.file).resolve()
        if not source.is_relative_to(path.resolve()):
            raise ValueError('invalid recording file')
        with source.open('rb') as stream:
            stream.seek(args.offset)
            remaining = entry['size']-args.offset
            while remaining and (block := stream.read(min(1024*1024, remaining))):
                sys.stdout.buffer.write(block)
                remaining -= len(block)
            sys.stdout.buffer.flush()


if __name__ == '__main__':
    main()
