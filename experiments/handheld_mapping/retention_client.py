"""Fresh laptop proof + Pi revalidation before retention cleanup. Never deletes locally."""
import hashlib
import json
from pathlib import Path
import subprocess
import threading
import time

from .recording_retention import (checked_hash, durable_json, entries, fsync_dir,
                                  manifest_digest)
from .recording_recovery import scan_capture


def receipt_path(root, host, run_id):
    key = hashlib.sha256(json.dumps([host, run_id]).encode()).hexdigest()
    return Path(root)/'.pi_verified_copies'/(key+'.json')


def verify_local(run, manifest, cancel_check=lambda: None):
    """Durably retain exact raw archives; PNG/map success alone is insufficient."""
    run = Path(run)
    cache = run/'pi_originals'
    if run.is_symlink() or cache.is_symlink():
        raise ValueError('노트북 원본 폴더 링크는 자동 정리 대상에서 제외합니다')
    run_id = Path(manifest['remote_path']).name
    files = entries(manifest, run_id)
    if (cache/'manifest.json').is_symlink():
        raise ValueError('노트북 manifest 링크는 자동 정리 대상에서 제외합니다')
    on_disk = json.loads((cache/'manifest.json').read_text())
    if manifest_digest(on_disk) != manifest_digest(manifest):
        raise ValueError('노트북 manifest 변경')
    for entry in files:
        source = cache/Path(entry['name']).name
        checked_hash(source, entry, cancel_check, sync=True)
        if source.suffix == '.jsonl':
            with source.open('rb') as stream:
                for line in stream:
                    cancel_check()
                    if not line.endswith(b'\n'):
                        raise ValueError('미완성 IMU/상태 JSONL')
                    json.loads(line)
        elif source.suffix == '.json':
            json.loads(source.read_text())
    scan = scan_capture(cache/'capture.bin', cancel_check)
    if (scan['errors'] or scan['valid_bytes'] != (cache/'capture.bin').stat().st_size
            or not scan['calibration'] or scan['calibration'].get('preview')
            or not scan['finished'] or scan['finished'].get('status') != 'saved'
            or scan['quality']['counts'] != manifest.get('counts')
            or any(count <= 0 for count in scan['quality']['counts'].values())):
        raise ValueError('원본 영상 전체 디코딩/종료/프레임 수 검증 실패')
    # Persist manifest bytes, then directory entries for every level created by transfer.
    with (cache/'manifest.json').open('rb') as stream:
        import os
        os.fsync(stream.fileno())
    # Also persist a newly created output root and its parent directory entries.
    resolved_cache = cache.resolve()
    for directory in (resolved_cache, *resolved_cache.parents):
        fsync_dir(directory)
    return dict(run_id=run_id, manifest_sha256=manifest_digest(manifest),
                laptop_path=str(run.resolve()), verified_ns=time.time_ns())


def remote_json(host, run_id, action, request, cancel_check, timeout=300.):
    from .local_client import ssh_command
    process = subprocess.Popen(ssh_command(host, run_id, action), stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    result, errors = [], []
    data = json.dumps(request).encode() if request is not None else b''
    def exchange():
        try:
            result.append(process.communicate(data))
        except Exception as error:
            errors.append(error)
    thread = threading.Thread(target=exchange, daemon=True)
    thread.start()
    deadline = time.monotonic()+timeout
    try:
        while thread.is_alive():
            cancel_check()
            if time.monotonic() > deadline:
                raise TimeoutError('Pi 정리 응답 시간 초과 · 확인되지 않은 정리는 보류합니다')
            thread.join(.2)
        if errors:
            raise errors[0]
        stdout, stderr = result[0]
        if process.returncode:
            raise OSError('Pi 정리 요청 실패: '+stderr.decode(errors='replace')[-500:])
        return json.loads(stdout)
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
        thread.join(3)


def finalize_retention(host, run_id, run, manifest, emit, cancel_event):
    """Failures only defer cleanup; they never invalidate the copied originals."""
    from .local_client import CapturePaused
    from .local_recording import run_path
    def cancel_check():
        if cancel_event.is_set():
            raise CapturePaused('원본 검증·Pi 정리를 보류했습니다. 같은 촬영 폴더에서 재개하세요.')
    last_report = [0.]
    def checking():
        cancel_check()
        if time.monotonic()-last_report[0] > 1:
            emit(dict(kind='retention_status', message='원본 보존 검증 중 · 크기·SHA-256·영상 디코딩 확인'))
            last_report[0] = time.monotonic()
    try:
        # Legacy/inconsistent manifests never authorize remote removal.
        if manifest.get('remote_path') != str(run_path(run_id)):
            raise ValueError('Pi 촬영 경로 확인 불가 · 자동 정리 보류')
        current = verify_local(run, manifest, checking)
        root = Path(run).parent
        registry = root/'.pi_verified_copies'
        if registry.is_symlink():
            raise ValueError('전송 증명 폴더 링크 발견')
        registry.mkdir(exist_ok=True)
        durable_json(receipt_path(root, host, run_id), {**current, 'host': host})
        fsync_dir(root)
        listing = remote_json(host, run_id, 'catalog', None, cancel_check, timeout=30.)
        proofs, skipped = [], []
        for row in listing['recordings']:
            candidate = row['run_id']
            try:
                if row['manifest'].get('remote_path') != str(run_path(candidate)):
                    raise ValueError('촬영 경로 불일치')
                receipt_file = receipt_path(root, host, candidate)
                if receipt_file.is_symlink():
                    raise ValueError('전송 증명 링크 발견')
                if receipt_file.exists():
                    saved = json.loads(receipt_file.read_text())
                    local = Path(saved['laptop_path'])
                else:
                    # Upgrade pre-retention recordings only when the existing laptop
                    # checkpoint independently binds this exact host and recording ID.
                    local = root/candidate
                    binding = json.loads((local/'transfer_checkpoint.json').read_text())
                    saved = dict(host=binding.get('host'), run_id=binding.get('run_id'),
                                 manifest_sha256=manifest_digest(json.loads(
                                     (local/'pi_originals/manifest.json').read_text())))
                if (saved.get('host') != host or saved.get('run_id') != candidate
                        or local.parent.resolve() != root.resolve()
                        or saved['manifest_sha256'] != row['manifest_sha256']
                        or manifest_digest(row['manifest']) != row['manifest_sha256']):
                    raise ValueError('촬영별 전송 증명 불일치')
                # Re-read old originals now; a historical receipt is never enough.
                proof = verify_local(local, row['manifest'], checking)
                durable_json(receipt_file, {**proof, 'host': host})
                proofs.append(proof)
            except (OSError, ValueError, KeyError, TypeError) as error:
                skipped.append(dict(run_id=candidate, reason=str(error)))
        if not any(proof['run_id'] == run_id for proof in proofs):
            raise ValueError('현재 촬영에 대한 Pi/노트북 검증 일치 실패')
        emit(dict(kind='retention_status', message='원본 저장 검증 완료 · Pi에서 최신 촬영을 남기고 과거 검증본 정리 중'))
        result = remote_json(host, run_id, 'prune', dict(schema=1, proofs=proofs), cancel_check)
        result['local_skipped'] = skipped
        result['unverified_retained'] = listing.get('retained', [])
        result['status'] = 'complete'
        durable_json(Path(run)/'pi_retention.json', result)
        emit(dict(kind='retention_status', result=result, message=f"Pi 과거 원본 {len(result['deleted'])}개 정리 · 최신 촬영 및 검증되지 않은 원본 보존"))
        return result
    except CapturePaused:
        raise
    except Exception as error:
        result = dict(status='deferred', reason=str(error))
        # Even a local disk error must not turn cleanup into a recording failure.
        try:
            durable_json(Path(run)/'pi_retention.json', result)
        except OSError:
            pass
        emit(dict(kind='retention_status', result=result, message='Pi 원본 정리 보류 · '+str(error)))
        return result
