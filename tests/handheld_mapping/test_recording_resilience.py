"""Real subprocess failure/restart tests; never access SSH, camera, SPI or motors."""
import io
import json
import os
from pathlib import Path
import queue
import subprocess
import sys
import time

import numpy as np
import pytest

from test_local_recording import calibration, image, sensor_script, until
from test_imu_capture import CALIBRATION, AXES
from experiments.handheld_mapping.capture_integrity import CaptureIntegrity, stale_streams
from experiments.handheld_mapping.local_recording import digest
from experiments.handheld_mapping.protocol import write_packet
from experiments.handheld_mapping.storage import RunWriter


def recorder(folder, imu_script=None):
    folder.mkdir()
    (folder/'config.json').write_text(json.dumps(dict(imu_enabled=imu_script is not None)))
    if imu_script:
        (folder/'input_calibration.json').write_text(json.dumps(CALIBRATION))
        (folder/'input_robot.urdf').write_text('<robot name="test"/>')
    command = [sys.executable, '-u', '-c', sensor_script()]
    imu = [sys.executable, '-u', '-c', imu_script] if imu_script else None
    code = ('from experiments.handheld_mapping.local_recording import record; '
            f'record({str(folder)!r},worker_command={command!r},imu_command={imu!r},min_free=1)')
    return subprocess.Popen([sys.executable, '-u', '-c', code], stderr=subprocess.PIPE)


def cleanup(process, path):
    (path/'stop').touch()
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.terminate(); process.wait(timeout=5)
    process.stderr.close()


def test_imu_pause_keeps_camera_recording_and_marks_quality(tmp_path):
    imu_script = '''import sys,time,json,select
print(json.dumps(dict(kind='ready',chip_to_body=AXES)),flush=True)
start=time.monotonic();n=10000
while not select.select([sys.stdin],[],[],0)[0]:
 n+=20
 if 1.5 < time.monotonic()-start < 2.5:
  time.sleep(.02);continue
 print(json.dumps(dict(kind='telemetry',schema=1,host_monotonic_ns=time.monotonic_ns(),host_receipt_ns=time.time_ns(),tee_drops=0,telemetry=dict(flags=16,t_ms=n,tick=n%256,gyro_dps=[.5,1.4,1.6],acc_g=[.01,.02,1.03]))),flush=True)
 time.sleep(.02)
'''.replace('AXES', repr(AXES))
    path = tmp_path/'run'; p = recorder(path, imu_script)
    try:
        until(lambda:(path/'status.json').exists() and
              json.loads((path/'status.json').read_text()).get('imu_degraded'))
        before=json.loads((path/'status.json').read_text())['counts']['left']
        until(lambda:json.loads((path/'status.json').read_text())['counts']['left']>before+10)
        until(lambda:not json.loads((path/'status.json').read_text())['imu']['stale'])
        assert not (path/'done.json').exists()
        (path/'stop').touch();p.wait(timeout=10)
        done=json.loads((path/'done.json').read_text())
        assert done['error'] is None
        assert done['quality']['imu_degraded'] and done['quality']['partial']
        health=[json.loads(s)['state'] for s in (path/'imu_health.jsonl').read_text().splitlines()]
        assert 'stale' in health and health[-1]=='fresh'
    finally:
        cleanup(p,path)


def test_failed_imu_start_does_not_prevent_video_capture(tmp_path):
    path=tmp_path/'run'; p=recorder(path,'raise RuntimeError("injected receiver exit")')
    try:
        until(lambda:(path/'status.json').exists() and
              json.loads((path/'status.json').read_text())['counts']['left']>5)
        (path/'stop').touch();p.wait(timeout=10)
        done=json.loads((path/'done.json').read_text())
        assert done['error'] is None and done['counts']['left']>5
        assert done['quality']['imu_degraded']
    finally:
        cleanup(p,path)


def test_remote_imu_error_does_not_abort_original_import(tmp_path, monkeypatch):
    from experiments.handheld_mapping.controller import Session
    from experiments.handheld_mapping import offline, reconstruct
    packets=io.BytesIO()
    write_packet(packets,dict(kind='local_status',counts=dict(left=0,right=0,depth=0),
        imu=dict(error='injected IMU failure',stale=True),gravity_samples=[]))
    write_packet(packets,dict(kind='transfer_begin',remote_path='/fixture'))
    write_packet(packets,dict(kind='originals_begin',verified=True))
    write_packet(packets,calibration())
    for side in ('left','right','depth'):
        write_packet(packets,*image(side,1))
    write_packet(packets,dict(kind='finished',status='saved',capture_mode='sensors_only',dropped=0))
    write_packet(packets,dict(kind='transfer_complete',verified=True,counts=dict(left=1,right=1,depth=1)))
    source=tmp_path/'input.bin';source.write_bytes(packets.getvalue())
    cal=tmp_path/'cal.json';cal.write_text(json.dumps(CALIBRATION))
    monkeypatch.setattr(offline,'run_point',lambda *a,**kw:dict(status='complete'))
    monkeypatch.setattr(reconstruct,'reconstruct',lambda *a,**kw:dict(status='complete'))
    session=Session()
    session.start(dict(output_root=str(tmp_path/'sessions'),imu_enabled=True,imu_calibration=str(cal),
        local_recording=True,worker_command=[sys.executable,'-c',f'import sys;sys.stdout.buffer.write(open({str(source)!r},"rb").read())']))
    session.thread.join(timeout=10)
    assert not session.thread.is_alive()
    result=session.snapshot()
    assert result['state']=='partial' and result['counts']==dict(left=1,right=1,depth=1),result
    assert result['transfer_verified']


def test_integrity_detects_missing_stream_and_timestamp_gaps_with_bounded_memory():
    tracker=CaptureIntegrity()
    for i in range(1000):
        stamp=1_000_000_000+i*66_666_667
        for side in ('left','depth'):
            tracker.offer(dict(stream=side,stamp_ns=stamp,stream_seq=i+1))
        if i<10:
            tracker.offer(dict(stream='right',stamp_ns=stamp,stream_seq=i+1))
    result=tracker.result()
    assert result['partial'] and result['stereo_pairs']==10
    assert result['unmatched_stereo_images']==990
    assert all(len(p)<=128 for p in tracker.pending.values())
    other=CaptureIntegrity()
    for stamp in [1_000_000_000,1_200_000_000]:
        for side in ('left','right','depth'):
            other.offer(dict(stream=side,stamp_ns=stamp))
    assert other.result()['partial']
    assert other.result()['stereo_pair_fraction']==1.
    assert other.result()['streams']['left']['gap_events']==1
    assert stale_streams(dict(left=[9.9],right=[1.],depth=[2.]),10.)==['right','depth']


def test_crashed_recording_recovers_valid_prefix_without_changing_original(tmp_path):
    from experiments.handheld_mapping.recording_recovery import recover_abandoned,process_identity,prefix_digest
    path=tmp_path/'run'; p=recorder(path)
    try:
        until(lambda:(path/'status.json').exists() and
              json.loads((path/'status.json').read_text())['counts']['left']>8)
        (path/'started').write_text(json.dumps(process_identity(p.pid)))
        p.kill();p.wait(timeout=5)
        with (path/'capture.bin').open('ab') as stream:
            stream.write(b'\0\0\0')
        original=(path/'capture.bin').read_bytes()
        manifest=recover_abandoned(path)
        assert manifest['recovered'] and manifest['quality']['partial']
        assert manifest['counts']['left']>8
        assert (path/'capture.bin').read_bytes()==original
        entry=next(f for f in manifest['files'] if f['name']=='capture.bin')
        assert entry['size']<len(original)
        assert entry['sha256']==prefix_digest(path/'capture.bin',entry['size'])
        # The real remote fetch entry point must stop at the valid prefix, not send its damaged tail.
        code=('import sys;from pathlib import Path;from experiments.handheld_mapping import local_recording as lr;'
              f'lr.RECORDINGS=Path({str(tmp_path)!r});sys.argv=["x","fetch","run","--file","capture.bin"];lr.main()')
        fetched=subprocess.run([sys.executable,'-c',code],capture_output=True,check=True,timeout=5).stdout
        assert fetched==original[:entry['size']]
        assert recover_abandoned(path)==manifest
    finally:
        cleanup(p,path)


def make_archive(tmp_path):
    archive=tmp_path/'remote_capture.bin'
    with archive.open('wb') as stream:
        write_packet(stream,calibration())
        for i in range(8):
            for side in ('depth','left','right'):
                write_packet(stream,*image(side,1_000_000_000+i*66_666_667))
        write_packet(stream,dict(kind='finished',status='saved',capture_mode='sensors_only',dropped=0))
    return archive,dict(kind='local_done',remote_path='/home/pi/handheld_mapping/recordings/run',error=None,
        files=[dict(name='capture.bin',size=archive.stat().st_size,sha256=digest(archive))],
        counts=dict(left=8,right=8,depth=8))


def test_resume_download_and_import_after_two_separate_interruptions(tmp_path,monkeypatch):
    from experiments.handheld_mapping import local_client as client
    archive,manifest=make_archive(tmp_path)
    packet=io.BytesIO();write_packet(packet,manifest)
    remote_done=tmp_path/'remote_done';remote_done.write_bytes(packet.getvalue())
    offsets=[]
    def ssh(host,run_id,action,*options):
        if action=='start':
            return [sys.executable,'-c',f'import sys;sys.stdout.buffer.write(open({str(remote_done)!r},"rb").read())']
        assert action=='fetch'
        offset=int(options[options.index('--offset')+1]);offsets.append(offset)
        if len(offsets)==1:
            return [sys.executable,'-u','-c',f'import sys,time;sys.stdout.buffer.write(open({str(archive)!r},"rb").read(200));sys.stdout.buffer.flush();time.sleep(30)']
        return [sys.executable,'-c',f'import sys;f=open({str(archive)!r},"rb");f.seek({offset});sys.stdout.buffer.write(f.read())']
    monkeypatch.setattr(client,'ssh_command',ssh)
    run=tmp_path/'laptop';writer=RunWriter(run);writer.close()
    controls=queue.Queue(); events=[]
    def first_emit(h,p=b''):
        events.append(h)
        if h['kind']=='transfer_progress' and 0<h['bytes']<h['total']:
            controls.put('pause')
    client.transport('fixture','run',run,first_emit,controls)
    assert events[-1]['kind']=='transport_paused'
    partial=(run/'pi_originals/capture.bin.part').stat().st_size
    assert 0<partial<archive.stat().st_size
    writer=RunWriter(run,resume=True);controls=queue.Queue();importing=False;requested=False
    def second_emit(h,p=b''):
        nonlocal importing,requested
        if h['kind']=='originals_begin':
            importing=True
        elif importing and h['kind'] in ('calibration','image','finished'):
            writer.accept(h,p)
            if h['kind']=='image' and not requested:
                requested=True;controls.put('pause');time.sleep(.2)
    client.transport('fixture','run',run,second_emit,controls,recover=True)
    writer.close()
    assert offsets==[0,partial]
    writer=RunWriter(run,resume=True);importing=False;completed=[]
    def third_emit(h,p=b''):
        nonlocal importing
        if h['kind']=='originals_begin':
            importing=True
        elif h['kind']=='transfer_complete':
            completed.append(h)
        elif importing:
            writer.accept(h,p)
    client.transport('fixture','run',run,third_emit,queue.Queue(),recover=True)
    assert writer.counts==dict(left=8,right=8,depth=8)
    writer.close()
    assert completed and len((run/'frames.jsonl').read_text().splitlines())==24
    assert offsets==[0,partial]  # Already-verified capture is reused after import interruption.
    with pytest.raises(ValueError,match='ID'):
        client.validate_resume_target(run,'fixture','another_run')


def test_stop_is_not_reported_as_transfer_until_remote_confirms_and_timeout_can_pause(tmp_path,monkeypatch):
    from experiments.handheld_mapping import local_client as client
    monkeypatch.setattr(client,'ssh_command',lambda *a:[sys.executable,'-c','import time;time.sleep(30)'])
    run=tmp_path/'laptop';run.mkdir();controls=queue.Queue();controls.put('stop');events=[]
    client.transport('fixture','run',run,lambda h,p=b'':events.append(h),controls,connection_timeout=.2)
    kinds=[h['kind'] for h in events]
    assert 'stop_pending' in kinds and 'transfer_begin' not in kinds
    assert kinds[-1]=='transport_paused' and not events[-1]['remote_stop_confirmed']
    assert json.loads((run/'transfer_checkpoint.json').read_text())['phase']=='paused'


def test_bridge_and_single_owner_survive_log_viewer_exit_then_stop_cleanly(tmp_path):
    from experiments.handheld_mapping.imu_bridge_service import start_service,stop_service,alive,identity
    root=tmp_path/'bridge'
    code='import time\ntry:\n while True: print("TLM",flush=True);time.sleep(.03)\nfinally: print("ZERO_STOP",flush=True)'
    state=start_service(root,[sys.executable,'-u','-c',code])
    follower=None
    try:
        again=start_service(root,[sys.executable,'-c','raise RuntimeError("must not launch twice")'])
        assert again['pid']==state['pid'] and again['child']==state['child']
        script=Path('experiments/handheld_mapping/imu_bridge_service.py').resolve()
        follower=subprocess.Popen([sys.executable,str(script),'follow','--root',str(root)],stdout=subprocess.DEVNULL)
        time.sleep(.2);follower.terminate();follower.wait(timeout=5)
        assert alive(state) and identity(state['child']['pid'])==state['child']
    finally:
        stop_service(root)
        if follower and follower.poll() is None:
            follower.terminate();follower.wait(timeout=5)
    assert not alive(state) and identity(state['child']['pid']) is None
    assert 'ZERO_STOP' in (root/'bridge.log').read_text()


def test_resume_writer_rejects_changed_image_but_rebuilds_missing_png(tmp_path):
    path=tmp_path/'run';writer=RunWriter(path)
    writer.accept(calibration(),b'');writer.accept(*image('left',1));writer.close()
    target=path/'raw/left/1.png';target.unlink()
    writer=RunWriter(path,resume=True)
    writer.accept(*image('left',1));assert target.exists() and writer.counts['left']==1
    import cv2
    cv2.imwrite(str(target),np.zeros((60,80),np.uint8))
    with pytest.raises(ValueError,match='다릅니다'):
        writer.accept(*image('left',1))
    writer.close()


def test_app_session_resumes_cached_originals_with_binary_client_and_no_new_capture(tmp_path,monkeypatch):
    from experiments.handheld_mapping import local_client,offline,reconstruct
    from experiments.handheld_mapping.controller import Session
    import shutil
    archive,manifest=make_archive(tmp_path)
    run=tmp_path/'existing_session';writer=RunWriter(run)
    writer.accept(calibration(),b'');writer.accept(*image('depth',1_000_000_000));writer.close()
    (run/'session.json').write_text(json.dumps(dict(state='paused',imu_requested=False,gravity_requested=False)))
    cache=run/'pi_originals';cache.mkdir()
    shutil.copyfile(archive,cache/'capture.bin')
    (cache/'manifest.json').write_text(json.dumps(manifest))
    local_client.checkpoint(run,'fixture','run',phase='importing',manifest_sha256=digest(cache/'manifest.json'))
    monkeypatch.setattr(local_client,'provision',lambda *a:pytest.fail('resume must not deploy or start capture'))
    monkeypatch.setattr(offline,'run_point',lambda *a,**kw:dict(status='complete'))
    monkeypatch.setattr(reconstruct,'reconstruct',lambda *a,**kw:dict(status='complete'))
    session=Session();session.resume_transfer(run);session.thread.join(timeout=15)
    assert not session.thread.is_alive()
    result=session.snapshot()
    assert result['state']=='complete' and result['transfer_verified'],result
    assert result['counts']==dict(left=8,right=8,depth=8)
    assert len((run/'frames.jsonl').read_text().splitlines())==24


def test_paused_session_can_close_and_resume_button_is_available(tmp_path,monkeypatch):
    os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
    from PyQt5.QtWidgets import QApplication
    from experiments.handheld_mapping.app import MappingWindow
    from experiments.handheld_mapping.controller import Session
    app=QApplication.instance() or QApplication([])
    session=Session()
    session.update(state='transferring',local_recording=True)
    calls=[]
    def pause():
        calls.append(True);session.update(state='paused')
    monkeypatch.setattr(session,'pause_transfer',pause)
    window=MappingWindow(session,tmp_path,False)
    try:
        window.refresh()
        assert window.pause_button.isEnabled()
        window.close();app.processEvents()
        assert calls and session.snapshot()['state']=='paused'
        window.refresh()
        assert window.resume_button.isEnabled()
    finally:
        window.timer.stop();window.close()


def test_crash_recovery_excludes_torn_imu_line_from_copy_only(tmp_path):
    from experiments.handheld_mapping.recording_recovery import recover_abandoned,jsonl_prefix_size
    archive,_=make_archive(tmp_path)
    path=tmp_path/'run';path.mkdir();(path/'config.json').write_text('{"imu_enabled":true}')
    (path/'capture.bin').write_bytes(archive.read_bytes())
    imu=path/'imu/run';imu.mkdir(parents=True)
    valid=b'{"raw":{"sample":1}}\n'
    (imu/'imu.jsonl').write_bytes(valid+b'{"raw":')
    manifest=recover_abandoned(path)
    entry=next(f for f in manifest['files'] if f['name']=='imu/run/imu.jsonl')
    assert entry['size']==len(valid)==jsonl_prefix_size(imu/'imu.jsonl')
    assert (imu/'imu.jsonl').read_bytes()==valid+b'{"raw":'


def test_capture_continuity_quality_forces_partial_even_when_remaining_map_succeeds(tmp_path,monkeypatch):
    from experiments.handheld_mapping.controller import Session
    from experiments.handheld_mapping import offline,reconstruct
    packets=io.BytesIO();tracker=CaptureIntegrity()
    write_packet(packets,dict(kind='originals_begin',verified=True))
    write_packet(packets,calibration())
    for i in range(2):
        for side in ('depth','left','right'):
            if i and side=='right':
                continue
            header,payload=image(side,1_000_000_000+i*66_666_667)
            tracker.offer(header);write_packet(packets,header,payload)
    quality=tracker.result()
    write_packet(packets,dict(kind='capture_quality',**quality))
    write_packet(packets,dict(kind='finished',status='saved',capture_mode='sensors_only',dropped=0))
    write_packet(packets,dict(kind='transfer_complete',verified=True,counts=quality['counts']))
    source=tmp_path/'input.bin';source.write_bytes(packets.getvalue())
    monkeypatch.setattr(offline,'run_point',lambda *a,**kw:dict(status='complete'))
    monkeypatch.setattr(reconstruct,'reconstruct',lambda *a,**kw:dict(status='complete'))
    session=Session();session.start(dict(output_root=str(tmp_path/'sessions'),local_recording=True,
        worker_command=[sys.executable,'-c',f'import sys;sys.stdout.buffer.write(open({str(source)!r},"rb").read())']))
    session.thread.join(timeout=10)
    assert session.snapshot()['state']=='partial'
    assert session.snapshot()['capture_quality']['unmatched_stereo_images']==1
