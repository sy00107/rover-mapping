"""Fail-closed Pi retention: fresh, durable laptop copies authorize old-run removal."""
from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import time
import uuid

ID = re.compile(r'[A-Za-z0-9][A-Za-z0-9_-]{0,79}')
OPERATIONAL_FILES = {'done.json', 'started', 'config.json', 'input_calibration.json',
    'input_robot.urdf', 'children.json', 'capture_stopped.json', 'status.json', 'stop',
    'zero_imu', 'ready', 'preview.bin', 'preview.tmp', 'preview_seq.json',
    'preview_calibration.json', 'supervisor.log'}
IMU_FILES = {'imu.jsonl', 'imu_calibration.json', 'robot.urdf', 'imu_manifest.json',
             'imu_summary.json', 'imu_display_events.jsonl'}


def fsync_dir(path):
    fd = os.open(str(path), os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def durable_json(path, data):
    """Persist both JSON bytes and its directory entry before returning."""
    path = Path(path)
    temporary = path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    try:
        with temporary.open('x') as stream:
            json.dump(data, stream, sort_keys=True, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        fsync_dir(path.parent)
    finally:
        temporary.unlink(missing_ok=True)


def manifest_digest(manifest):
    # Wire envelopes add kind=local_done; it isn't part of done.json on the Pi.
    value = {k: v for k, v in manifest.items() if k != 'kind'}
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def entries(manifest, run_id):
    if not ID.fullmatch(run_id) or manifest.get('error') or manifest.get('recovered'):
        raise ValueError('비정상 종료·복구 촬영은 Pi에 보존합니다')
    if manifest.get('quality', {}).get('imu_degraded'):
        raise ValueError('IMU 기록 이상 촬영은 Pi에 보존합니다')
    files = manifest['files']
    names = set()
    for entry in files:
        name = entry['name']
        allowed = name in {'capture.bin', 'recorder.log', 'camera.log', 'imu_health.jsonl'} or (
            Path(name).parent == Path('imu')/run_id and Path(name).name in IMU_FILES)
        if (not allowed or name in names or type(entry['size']) is not int or entry['size'] < 0
                or not re.fullmatch(r'[0-9a-f]{64}', entry['sha256'])):
            raise ValueError('삭제 검증용 원본 목록이 유효하지 않습니다')
        names.add(name)
    if 'capture.bin' not in names:
        raise ValueError('원본 영상이 없습니다')
    if manifest.get('imu_required') and not {
            f'imu/{run_id}/{name}' for name in IMU_FILES-{'imu_display_events.jsonl'}}.issubset(names):
        raise ValueError('IMU 원본 목록이 불완전합니다')
    return files


def checked_hash(path, entry, cancel_check=lambda: None, sync=False):
    """Reject symlinks, changed sizes and files modified while being checked."""
    fd = os.open(str(path), os.O_RDONLY | os.O_NOFOLLOW)
    try:
        before = os.fstat(fd)
        import stat
        if not stat.S_ISREG(before.st_mode) or before.st_size != entry['size']:
            raise ValueError('원본 크기/종류 불일치: '+entry['name'])
        sha = hashlib.sha256()
        while block := os.read(fd, 1024*1024):
            cancel_check()
            sha.update(block)
        if sha.hexdigest() != entry['sha256']:
            raise ValueError('원본 SHA-256 불일치: '+entry['name'])
        if sync:
            os.fsync(fd)
        after = os.fstat(fd)
        if (before.st_size, before.st_mtime_ns, before.st_ctime_ns) != (
                after.st_size, after.st_mtime_ns, after.st_ctime_ns):
            raise ValueError('검증 중 원본 변경: '+entry['name'])
    finally:
        os.close(fd)


@contextmanager
def run_lock(root, run_id, blocking=False):
    if not ID.fullmatch(run_id):
        raise ValueError('invalid recording ID')
    with (Path(root)/('.run_lock_'+run_id)).open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | (0 if blocking else fcntl.LOCK_NB))
        yield


def load_remote(root, run_id):
    from .recording_recovery import owner_alive
    if not isinstance(run_id, str) or not ID.fullmatch(run_id):
        raise ValueError('invalid recording ID')
    path = Path(root)/run_id
    if path.is_symlink() or not path.is_dir():
        raise ValueError('invalid recording directory')
    # No deletion while supervisor/owned children might still be writing.
    marker = json.loads((path/'started').read_text())
    if not marker.get('created_ns') or owner_alive(marker):
        raise ValueError('촬영 종료 확인 불가')
    if not (path/'capture_stopped.json').is_file():
        raise ValueError('카메라 종료 확인 불가')
    children = json.loads((path/'children.json').read_text()) if (path/'children.json').exists() else []
    if any(owner_alive(child) for child in children):
        raise ValueError('촬영 자식 프로세스가 실행 중입니다')
    manifest = json.loads((path/'done.json').read_text())
    if manifest.get('remote_path') != str(path):
        raise ValueError('manifest 촬영 경로 불일치')
    files = entries(manifest, run_id)
    return path, marker['created_ns'], manifest, files


def catalog(root):
    root = Path(root)
    rows, retained = [], []
    for path in sorted(root.iterdir()):
        if not ID.fullmatch(path.name) or not path.is_dir() or path.is_symlink():
            continue
        try:
            _, created, manifest, _ = load_remote(root, path.name)
            rows.append(dict(run_id=path.name, created_ns=created, manifest=manifest,
                             manifest_sha256=manifest_digest(manifest)))
        except (OSError, ValueError, KeyError, TypeError) as error:
            retained.append(dict(run_id=path.name, reason=str(error)))
    return dict(schema=1, recordings=rows, retained=retained)


def newest_run(root):
    """Protect the latest started run, including incomplete/untransferred recordings."""
    times = []
    for path in Path(root).iterdir():
        if ID.fullmatch(path.name) and path.is_dir() and not path.is_symlink():
            # If any start time is unreadable, refuse pruning rather than guess recency.
            marker = json.loads((path/'started').read_text())
            created = marker['created_ns']
            if type(created) is not int or created <= 0:
                raise ValueError('촬영 순서 확인 불가')
            times.append((created, path.name))
    return max(times)[1] if times else None


def prune(root, request):
    # Serialize pruning clients, without occupying the camera's global lock.
    with (Path(root)/'.retention.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return _prune(root, request)


def _prune(root, request):
    """Never infer receipt from a previous run; only this request's fresh proofs count."""
    root = Path(root)
    if root.is_symlink() or not root.is_dir():
        raise ValueError('invalid recordings root')
    proofs = request.get('proofs', [])
    if request.get('schema') != 1 or not proofs or len(proofs) > 1000:
        raise ValueError('invalid cleanup request')
    valid, skipped, deleted = {}, [], []
    for proof in proofs:
        try:
            run_id = proof['run_id']
            path, created, manifest, files = load_remote(root, run_id)
            if manifest_digest(manifest) != proof['manifest_sha256']:
                raise ValueError('Pi manifest 변경')
            valid[run_id] = (created, proof)
        except (OSError, ValueError, KeyError, TypeError) as error:
            skipped.append(dict(run_id=proof.get('run_id'), reason=str(error)))
    if not valid:
        return dict(deleted=[], skipped=skipped, kept=[])
    keep_verified = max((created, run_id) for run_id, (created, _) in valid.items())[1]
    # Require that the retained verified backup itself still matches, before deleting older runs.
    with run_lock(root, keep_verified):
        path, _, _, files = load_remote(root, keep_verified)
        for entry in files:
            checked_hash(path/entry['name'], entry)
        keep = {newest_run(root), keep_verified}
        audit = root/'.retention_audit'; audit.mkdir(exist_ok=True)
        trash = root/'.verified_trash'; trash.mkdir(exist_ok=True)
        fsync_dir(root)
        for run_id, (_, proof) in sorted(valid.items()):
            if run_id in keep:
                continue
            try:
                with run_lock(root, run_id):
                    path, _, manifest, files = load_remote(root, run_id)
                    if manifest_digest(manifest) != proof['manifest_sha256']:
                        raise ValueError('Pi manifest 변경')
                    allowed = OPERATIONAL_FILES | {e['name'] for e in files}
                    for child in path.rglob('*'):
                        if child.is_symlink() or (child.is_file() and str(child.relative_to(path)) not in allowed):
                            raise ValueError('원본 목록 외 파일/링크 발견 · 보존')
                    for entry in files:
                        checked_hash(path/entry['name'], entry)
                    # Recheck recency and finalization after potentially long hashing.
                    if run_id == newest_run(root):
                        keep.add(run_id)
                        continue
                    _, _, latest, _ = load_remote(root, run_id)
                    if manifest_digest(latest) != proof['manifest_sha256']:
                        raise ValueError('검증 중 Pi manifest 변경')
                    token = run_id+'_'+uuid.uuid4().hex
                    journal = audit/(token+'.json')
                    target = trash/token
                    row = dict(schema=1, run_id=run_id, manifest=manifest, proof=proof,
                               kept=sorted(keep), trash_path=str(target), state='authorized', time_ns=time.time_ns())
                    durable_json(journal, row)
                    os.rename(path, target)
                    fsync_dir(root); fsync_dir(trash)
                    shutil.rmtree(target)
                    fsync_dir(trash)
                    durable_json(journal, {**row, 'state': 'deleted'})
                    deleted.append(run_id)
            except (OSError, ValueError, KeyError, TypeError) as error:
                skipped.append(dict(run_id=run_id, reason=str(error)))
        return dict(deleted=deleted, skipped=skipped, kept=sorted(keep))
