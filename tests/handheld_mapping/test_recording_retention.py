"""Real files/protocol/subprocess CLI, with no Pi, camera or motor access."""
import json
import os
from pathlib import Path
import shutil
import sys
import threading

import pytest

from experiments.handheld_mapping import local_client, local_recording
from experiments.handheld_mapping import recording_retention as retention
from experiments.handheld_mapping import retention_client as client
from experiments.handheld_mapping.protocol import write_packet
from test_local_recording import calibration, image


@pytest.fixture
def env(tmp_path, monkeypatch):
    remote = tmp_path/'pi'/'recordings'; remote.mkdir(parents=True)
    laptop = tmp_path/'laptop'; laptop.mkdir()
    monkeypatch.setattr(local_recording, 'RECORDINGS', remote)
    def ssh(host, run_id, action, *options):
        assert host == 'pi@test'
        script = ('import sys;from pathlib import Path;'
                  'from experiments.handheld_mapping import local_recording as r;'
                  f'r.RECORDINGS=Path({str(remote)!r});'
                  f'sys.argv=["recorder",{action!r},{run_id!r},*{list(options)!r}];r.main()')
        return [sys.executable, '-u', '-c', script]
    monkeypatch.setattr(local_client, 'ssh_command', ssh)
    return remote, laptop


def recording(root, run_id, created, imu=True):
    path = root/run_id; path.mkdir()
    (path/'started').write_text(json.dumps(dict(pid=99999999, created_ns=created)))
    (path/'capture_stopped.json').write_text('{}')
    (path/'children.json').write_text('[]')
    (path/'config.json').write_text(json.dumps(dict(imu_enabled=imu)))
    with (path/'capture.bin').open('wb') as stream:
        write_packet(stream, calibration())
        for stamp in (1_000_000_000, 1_066_666_667):
            for side in ('left', 'right', 'depth'):
                write_packet(stream, *image(side, stamp))
        write_packet(stream, dict(kind='finished', status='saved', capture_mode='sensors_only'))
    files = [path/'capture.bin']
    if imu:
        folder = path/'imu'/run_id; folder.mkdir(parents=True)
        for name in retention.IMU_FILES-{'imu_display_events.jsonl'}:
            value = '<robot/>' if name.endswith('.urdf') else '{}\n'
            source = folder/name; source.write_text(value); files.append(source)
    manifest = dict(remote_path=str(path), files=[dict(name=str(f.relative_to(path)),
        size=f.stat().st_size, sha256=local_recording.digest(f)) for f in files],
        counts=dict(left=2, right=2, depth=2), error=None, imu_required=imu,
        quality=dict(partial=False, imu_degraded=False))
    (path/'done.json').write_text(json.dumps(manifest))
    return manifest


def copy_to_laptop(root, manifest):
    run_id = Path(manifest['remote_path']).name
    run = root/run_id; cache = run/'pi_originals'; cache.mkdir(parents=True)
    (cache/'manifest.json').write_text(json.dumps(manifest))
    for entry in manifest['files']:
        shutil.copyfile(Path(manifest['remote_path'])/entry['name'], cache/Path(entry['name']).name)
    return run


def finish(run, manifest):
    events = []
    result = client.finalize_retention('pi@test', run.name, run, manifest, events.append, threading.Event())
    return result, events


def register(root, run, manifest):
    proof = client.verify_local(run, manifest)
    registry = root/'.pi_verified_copies'; registry.mkdir(exist_ok=True)
    retention.durable_json(client.receipt_path(root, 'pi@test', run.name), {**proof, 'host':'pi@test'})
    return proof


def test_subprocess_cleanup_keeps_latest_and_never_clears_untransferred_data(env):
    remote, laptop = env
    a = recording(remote, 'a', 1); b = recording(remote, 'b', 2)
    c = recording(remote, 'c', 4); recording(remote, 'untransferred', 3)
    ra = copy_to_laptop(laptop, a); rb = copy_to_laptop(laptop, b); rc = copy_to_laptop(laptop, c)
    result, _ = finish(ra, a)
    assert result['status'] == 'complete' and not result['deleted']
    result, _ = finish(rb, b)
    assert result['deleted'] == ['a'] and (remote/'c/capture.bin').exists()
    result, _ = finish(rc, c)
    assert result['deleted'] == ['b']
    assert (remote/'c/capture.bin').exists() and (remote/'untransferred/capture.bin').exists()
    for run, manifest in ((ra,a),(rb,b),(rc,c)):
        assert client.verify_local(run,manifest)['manifest_sha256'] == retention.manifest_digest(manifest)
    audits = [json.loads(p.read_text()) for p in (remote/'.retention_audit').glob('*.json')]
    assert len(audits) == 2 and all(row['state'] == 'deleted' for row in audits)
    assert not list((remote/'.verified_trash').iterdir())


@pytest.mark.parametrize('damage', ['corrupt', 'missing', 'part', 'manifest', 'symlink'])
def test_old_receipt_never_substitutes_for_fresh_old_copy_verification(env, damage):
    remote, laptop = env
    a = recording(remote, 'a', 1); b = recording(remote, 'b', 2)
    ra = copy_to_laptop(laptop, a); rb = copy_to_laptop(laptop, b)
    register(laptop, ra, a)
    source = ra/'pi_originals/capture.bin'
    if damage == 'corrupt':
        raw = source.read_bytes(); source.write_bytes(raw[:-1]+b'x')
    elif damage == 'missing': source.unlink()
    elif damage == 'part': source.rename(source.with_suffix('.bin.part'))
    elif damage == 'manifest': (ra/'pi_originals/manifest.json').write_text('{}')
    else:
        saved = ra/'elsewhere.bin'; source.rename(saved); source.symlink_to(saved)
    result, _ = finish(rb, b)
    assert result['status'] == 'complete' and not result['deleted']
    assert result['local_skipped'] and (remote/'a/capture.bin').exists()


@pytest.mark.parametrize('damage', ['hash', 'torn_video', 'bad_compression', 'imu', 'counts', 'recovered', 'missing_imu'])
def test_bad_current_copy_never_authorizes_cleanup(env, damage, monkeypatch):
    remote, laptop = env
    a = recording(remote, 'a', 1); b = recording(remote, 'b', 2)
    ra = copy_to_laptop(laptop, a); register(laptop, ra, a)
    if damage in {'torn_video', 'bad_compression'}:
        source = remote/'b/capture.bin'
        if damage == 'torn_video':
            with source.open('ab') as stream: stream.write(b'\x00\x00')
        else:
            with source.open('wb') as stream:
                write_packet(stream, calibration())
                header, _ = image('left',1)
                write_packet(stream, header, b'not zlib')
        b['files'][0].update(size=source.stat().st_size, sha256=local_recording.digest(source))
    elif damage == 'counts': b['counts']['left'] = 3
    elif damage == 'recovered': b['recovered'] = True
    elif damage == 'missing_imu': b['files'] = b['files'][:-1]
    elif damage == 'imu':
        source = remote/'b/imu/b/imu.jsonl'; source.write_text('{"broken":')
        entry = next(e for e in b['files'] if e['name'].endswith('/imu.jsonl'))
        entry.update(size=source.stat().st_size, sha256=local_recording.digest(source))
    (remote/'b/done.json').write_text(json.dumps(b))
    rb = copy_to_laptop(laptop,b)
    if damage == 'hash': (rb/'pi_originals/capture.bin').write_bytes(b'x')
    calls=[]
    monkeypatch.setattr(client, 'remote_json', lambda *a, **k: calls.append(a))
    result, _ = finish(rb,b)
    assert result['status'] == 'deferred' and calls == []
    assert (remote/'a/capture.bin').exists() and (remote/'b/capture.bin').exists()


@pytest.mark.parametrize('damage', ['changed', 'extra_tail', 'extra_file', 'symlink', 'running'])
def test_pi_rechecks_old_source_and_refuses_unexpected_data(env, damage):
    remote, laptop = env
    a = recording(remote,'a',1); b = recording(remote,'b',2)
    ra=copy_to_laptop(laptop,a); register(laptop,ra,a)
    rb=copy_to_laptop(laptop,b)
    if damage == 'changed':
        source=remote/'a/capture.bin'; raw=source.read_bytes();source.write_bytes(raw[:-1]+b'x')
    elif damage == 'extra_tail':
        with (remote/'a/capture.bin').open('ab') as f: f.write(b'unlisted tail')
    elif damage == 'extra_file': (remote/'a/unlisted-video.bin').write_bytes(b'unlisted original')
    elif damage == 'symlink': (remote/'a/link').symlink_to(remote/'b')
    else:
        (remote/'a/started').write_text(json.dumps(dict(pid=os.getpid(),created_ns=1)))
    result,_=finish(rb,b)
    assert not result.get('deleted')
    assert (remote/'a/capture.bin').exists() and (remote/'b/capture.bin').exists()


def test_fsync_failure_blocks_receipt_and_prune(env,monkeypatch):
    remote,laptop=env
    a=recording(remote,'a',1); b=recording(remote,'b',2)
    ra=copy_to_laptop(laptop,a); register(laptop,ra,a); rb=copy_to_laptop(laptop,b)
    def fail(*args): raise OSError('simulated fsync failure')
    monkeypatch.setattr(client,'fsync_dir',fail)
    result,_=finish(rb,b)
    assert result['status']=='deferred' and 'fsync' in result['reason']
    assert not client.receipt_path(laptop,'pi@test','b').exists()
    assert (remote/'a/capture.bin').exists()


def test_lost_cleanup_connection_can_be_retried_without_recapture(env,monkeypatch):
    remote,laptop=env
    a=recording(remote,'a',1); b=recording(remote,'b',2)
    ra=copy_to_laptop(laptop,a); register(laptop,ra,a); rb=copy_to_laptop(laptop,b)
    original=client.remote_json
    def offline(host,run_id,action,*args,**kwargs):
        if action=='prune': raise ConnectionError('dropped before cleanup')
        return original(host,run_id,action,*args,**kwargs)
    monkeypatch.setattr(client,'remote_json',offline)
    result,_=finish(rb,b)
    assert result['status']=='deferred' and (remote/'a/capture.bin').exists()
    monkeypatch.setattr(client,'remote_json',original)
    result,_=finish(rb,b)
    assert result['deleted']==['a'] and (remote/'b/capture.bin').exists()


def test_interrupted_delete_leaves_audit_and_recoverable_trash(env,monkeypatch):
    remote,laptop=env
    a=recording(remote,'a',1); b=recording(remote,'b',2)
    ra=copy_to_laptop(laptop,a); rb=copy_to_laptop(laptop,b)
    proofs=[register(laptop,ra,a),register(laptop,rb,b)]
    def fail(path): raise OSError('simulated delete interruption')
    monkeypatch.setattr(retention.shutil,'rmtree',fail)
    result=retention.prune(remote,dict(schema=1,proofs=proofs))
    assert result['skipped'] and result['deleted']==[]
    trash=list((remote/'.verified_trash').iterdir())
    assert len(trash)==1 and (trash[0]/'capture.bin').read_bytes()==(ra/'pi_originals/capture.bin').read_bytes()
    audit=json.loads(next((remote/'.retention_audit').glob('*.json')).read_text())
    assert audit['state']=='authorized' and (remote/'b/capture.bin').exists()


def test_per_run_lock_prevents_cleanup_of_writer_owned_run(env):
    remote,laptop=env
    a=recording(remote,'a',1); b=recording(remote,'b',2)
    ra=copy_to_laptop(laptop,a); rb=copy_to_laptop(laptop,b)
    proofs=[register(laptop,ra,a),register(laptop,rb,b)]
    with retention.run_lock(remote,'a'):
        result=retention.prune(remote,dict(schema=1,proofs=proofs))
    assert result['skipped'] and not result['deleted'] and (remote/'a/capture.bin').exists()


def test_cancel_during_verification_never_sends_prune(env):
    remote,laptop=env
    a=recording(remote,'a',1); b=recording(remote,'b',2)
    ra=copy_to_laptop(laptop,a); register(laptop,ra,a); rb=copy_to_laptop(laptop,b)
    cancel=threading.Event();cancel.set()
    with pytest.raises(local_client.CapturePaused):
        client.finalize_retention('pi@test','b',rb,b,lambda event:None,cancel)
    assert (remote/'a/capture.bin').exists()


def test_latest_incomplete_capture_is_kept_as_well_as_latest_verified(env):
    remote,laptop=env
    a=recording(remote,'a',1); b=recording(remote,'b',2)
    c=remote/'c';c.mkdir();(c/'started').write_text(json.dumps(dict(created_ns=3,pid=99999999)))
    (c/'capture.bin').write_bytes(b'in progress')
    ra=copy_to_laptop(laptop,a);register(laptop,ra,a); rb=copy_to_laptop(laptop,b)
    result,_=finish(rb,b)
    assert result['deleted']==['a'] and set(result['kept'])=={'b','c'}
    assert (c/'capture.bin').read_bytes()==b'in progress'


@pytest.mark.parametrize('host', ['pi@test', 'pi@other'])
def test_older_app_copies_require_bound_checkpoint_and_fresh_full_verification(env,host):
    remote,laptop=env
    a=recording(remote,'a',1); b=recording(remote,'b',2)
    ra=copy_to_laptop(laptop,a); rb=copy_to_laptop(laptop,b)
    (ra/'transfer_checkpoint.json').write_text(json.dumps(dict(host=host,run_id='a')))
    result,_=finish(rb,b)
    assert result['status']=='complete'
    assert result['deleted']==(['a'] if host=='pi@test' else [])
    assert (remote/'a').exists() == (host!='pi@test')


def test_changed_retained_pi_backup_blocks_all_deletion(env):
    remote,laptop=env
    a=recording(remote,'a',1); b=recording(remote,'b',2)
    ra=copy_to_laptop(laptop,a); rb=copy_to_laptop(laptop,b)
    proofs=[register(laptop,ra,a),register(laptop,rb,b)]
    (remote/'b/capture.bin').write_bytes(b'bad')
    with pytest.raises(ValueError): retention.prune(remote,dict(schema=1,proofs=proofs))
    assert (remote/'a/capture.bin').exists()


def test_invalid_start_time_blocks_guessing_latest_recording(env):
    remote,laptop=env
    a=recording(remote,'a',1); b=recording(remote,'b',2)
    ra=copy_to_laptop(laptop,a); register(laptop,ra,a); rb=copy_to_laptop(laptop,b)
    unstarted=remote/'unstarted'; unstarted.mkdir();(unstarted/'capture.bin').write_bytes(b'unknown')
    result,_=finish(rb,b)
    assert result['status']=='deferred' and (remote/'a/capture.bin').exists()


def test_real_transport_download_import_then_cleanup_uses_verified_archive(env):
    import queue
    remote,laptop=env
    a=recording(remote,'a',1); b=recording(remote,'b',2)
    ra=copy_to_laptop(laptop,a); register(laptop,ra,a)
    rb=laptop/'b'; rb.mkdir()
    controls=queue.Queue();controls.put('stop')
    events=[]
    local_client.transport('pi@test','b',rb,lambda h,p=b'':events.append(h),controls)
    assert events[-1]['kind']=='transfer_complete'
    assert events[-1]['verified'] is True
    assert any(row.get('result',{}).get('deleted')==['a'] for row in events)
    assert not (remote/'a').exists() and (remote/'b/capture.bin').exists()
    assert client.verify_local(rb,b)['manifest_sha256']==retention.manifest_digest(b)
    state=json.loads((rb/'transfer_checkpoint.json').read_text())
    assert state['phase']=='captured' and state['pi_retention']['deleted']==['a']


@pytest.mark.parametrize('result, expected', [
    (dict(status='deferred',reason='checksum mismatch'),'정리 보류'),
    (dict(status='complete',deleted=['a'],kept=['b'],skipped=[],local_skipped=[],unverified_retained=[]),'1개 정리'),
    (dict(status='complete',deleted=[],kept=['b'],skipped=[],local_skipped=[dict(run_id='a')],unverified_retained=[]),'1개 추가 보존'),
])
def test_ui_keeps_cleanup_outcome_visible_after_mapping(tmp_path,result,expected):
    from PyQt5.QtWidgets import QApplication
    from experiments.handheld_mapping.app import MappingWindow
    from experiments.handheld_mapping.controller import Session
    app=QApplication.instance() or QApplication([])
    session=Session();window=MappingWindow(session,tmp_path)
    window.show();session.update(state='complete',pi_retention=result)
    window.refresh();app.processEvents()
    assert window.retention_status.isVisible()
    assert expected in window.retention_status.text()
    window.close()


def test_durable_copy_syncs_new_output_root_and_all_ancestor_entries(env,monkeypatch):
    remote,laptop=env
    a=recording(remote,'a',1);ra=copy_to_laptop(laptop,a)
    synced=[]
    monkeypatch.setattr(client,'fsync_dir',synced.append)
    client.verify_local(ra,a)
    cache=(ra/'pi_originals').resolve()
    assert synced==[cache,*cache.parents]


def test_manifest_symlink_cannot_authorize_delete(env):
    remote,laptop=env
    a=recording(remote,'a',1);b=recording(remote,'b',2)
    ra=copy_to_laptop(laptop,a);register(laptop,ra,a);rb=copy_to_laptop(laptop,b)
    source=rb/'pi_originals/manifest.json'
    elsewhere=rb/'elsewhere.json';source.rename(elsewhere);source.symlink_to(elsewhere)
    result,_=finish(rb,b)
    assert result['status']=='deferred' and (remote/'a/capture.bin').exists()
