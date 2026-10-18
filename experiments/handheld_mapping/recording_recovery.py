"""Recover complete packet prefixes after recorder death without changing Pi originals."""
import hashlib
import json
import os
from pathlib import Path
import signal
import time
import zlib

from .capture_integrity import CaptureIntegrity
from .protocol import read_packet
from .storage import atomic_json
from .storage import decode_image


def process_identity(pid):
    try:
        # comm can contain spaces and parentheses; fields after the final ')' are stable.
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        if fields[0] == 'Z':
            return None
        return {'pid': int(pid), 'start_ticks': fields[19]}
    except (OSError, IndexError, ValueError):
        return None


def owner_alive(marker):
    if not marker.get('pid'):
        return False
    actual = process_identity(marker['pid'])
    return actual is not None and (not marker.get('start_ticks') or
                                   actual['start_ticks'] == marker['start_ticks'])


def prefix_digest(path, size):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        remaining = size
        while remaining:
            block = stream.read(min(1024*1024, remaining))
            if not block:
                raise EOFError('recording became shorter during recovery')
            result.update(block)
            remaining -= len(block)
    return result.hexdigest()


def scan_capture(path, cancel_check=lambda: None):
    integrity = CaptureIntegrity()
    valid_bytes = 0
    calibration = finished = None
    errors = []
    with Path(path).open('rb') as stream:
        while True:
            cancel_check()
            try:
                packet = read_packet(stream)
                if packet is None:
                    break
                header, payload = packet
                if header['kind'] == 'image':
                    decode_image(header, payload)
                    integrity.offer(header)
                elif header['kind'] == 'calibration':
                    calibration = header
                elif header['kind'] == 'finished':
                    finished = header
                elif header['kind'] == 'error':
                    errors.append(header.get('message', 'camera error'))
                valid_bytes = stream.tell()
            except (EOFError, ValueError, KeyError, TypeError, zlib.error) as error:
                errors.append('미완성/손상 packet 이전까지 복구: '+str(error))
                break
    return dict(valid_bytes=valid_bytes, calibration=calibration, finished=finished,
                quality=integrity.result(), errors=errors)


def jsonl_prefix_size(path):
    """An interrupted IMU line is retained on Pi but excluded from the recovered copy."""
    valid = 0
    with Path(path).open('rb') as stream:
        while line := stream.readline(65537):
            try:
                if not line.endswith(b'\n'):
                    break
                json.loads(line)
            except (ValueError, UnicodeDecodeError):
                break
            valid = stream.tell()
    return valid


def recover_abandoned(path):
    """Finalize only a dead owner's stable files; fetch will limit capture to its valid prefix."""
    import fcntl
    path = Path(path)
    if (path/'done.json').exists():
        return json.loads((path/'done.json').read_text())
    marker = json.loads((path/'started').read_text()) if (path/'started').exists() else {}
    if owner_alive(marker):
        return None
    if not (path/'capture.bin').is_file():
        return dict(files=[], error='Pi 원본 영상 파일이 없습니다', remote_path=str(path))
    with (path.parent/'.capture.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return None
        if (path/'done.json').exists():
            return json.loads((path/'done.json').read_text())
        children = json.loads((path/'children.json').read_text()) if (path/'children.json').exists() else []
        for child in children:
            if owner_alive(child):
                try:
                    os.killpg(child['pid'], signal.SIGINT)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic()+2
        while time.monotonic() < deadline and any(owner_alive(c) for c in children):
            time.sleep(.05)
        for child in children:
            if owner_alive(child):
                try:
                    os.killpg(child['pid'], signal.SIGTERM)
                except ProcessLookupError:
                    pass
        result = scan_capture(path/'capture.bin')
        config = json.loads((path/'config.json').read_text()) if (path/'config.json').exists() else {}
        quality = result['quality']
        quality.update(partial=True, interrupted=True, imu_degraded=bool(config.get('imu_enabled')))
        quality['issues'].append('촬영 프로세스 비정상 종료 · 완전한 packet까지만 회수')
        files = []
        candidates = [path/'capture.bin', path/'recorder.log', path/'camera.log', path/'imu_health.jsonl']
        candidates += sorted((path/'imu'/path.name).glob('*'))
        for source in candidates:
            if source.is_file():
                size = result['valid_bytes'] if source.name == 'capture.bin' else source.stat().st_size
                if source.suffix == '.jsonl':
                    size = jsonl_prefix_size(source)
                files.append(dict(name=str(source.relative_to(path)), size=size,
                                  sha256=prefix_digest(source, size)))
        error = None if result['calibration'] else '복구한 원본에 카메라 보정값이 없습니다'
        manifest = dict(files=files, counts=quality['counts'], error=error, quality=quality,
            recovered=True, imu_required=bool(config.get('imu_enabled')),
            finished=result['finished'], warnings=result['errors'], remote_path=str(path),
            original_capture_size=(path/'capture.bin').stat().st_size,
            recovered_capture_size=result['valid_bytes'])
        atomic_json(path/'done.json', manifest)
        return manifest
