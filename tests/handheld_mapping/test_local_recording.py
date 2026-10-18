"""Persistent capture survives a preview disconnect; copying verifies raw data."""
import hashlib
import io
import json
import subprocess
import sys
import time
import zlib

import numpy as np
import pytest

from experiments.handheld_mapping.local_recording import Preview, digest, preview_calibration
from experiments.handheld_mapping.local_client import receive_file
from experiments.handheld_mapping.protocol import read_packet, write_packet
from experiments.handheld_mapping.storage import decode_image


def calibration():
    info = dict(width=80, height=60, k=[70.,0,39.5,0,70.,29.5,0,0,1], d=[0.]*5,
                depth_scale_m=.001, T_left_depth=np.eye(4).tolist(), depth_frame='depth')
    return dict(kind='calibration', source='D435_Pi', capture_mode='sensors_only',
                depth=info, stereo=dict(left=dict(info), right=dict(info), T_c1_c2=np.eye(4).tolist()))


def image(side, stamp):
    a = np.full((60,80),1000 if side=='depth' else 100,
                dtype=np.uint16 if side=='depth' else np.uint8)
    return dict(kind='image',stream=side,stamp_ns=stamp,width=80,height=60,
                step=160 if side=='depth' else 80,encoding='16UC1' if side=='depth' else 'mono8',
                compression='zlib',frame_id=side,pi_receipt_monotonic_ns=time.monotonic_ns()), zlib.compress(a.tobytes(),1)


def until(predicate, timeout=10):
    deadline = time.monotonic()+timeout
    while not predicate():
        assert time.monotonic()<deadline
        time.sleep(.02)


def test_preview_stereo_does_not_wait_for_depth_and_calibration_matches_resize(tmp_path):
    preview = Preview(tmp_path)
    try:
        for side in ['left','right']:
            preview.offer(*image(side,1_000_000_000))
        until(lambda:(tmp_path/'preview.bin').exists())
        with (tmp_path/'preview.bin').open('rb') as f:
            a,b = read_packet(f),read_packet(f)
            assert read_packet(f) is None
        assert [a[0]['stream'],b[0]['stream']]==['left','right']
        for h,p in [a,b]:
            assert h['compression']=='jpeg' and h['preview'] is True
            assert decode_image(h,p).shape==(30,40)
        cal=preview_calibration(calibration())
        assert cal['depth']['width']==40
        assert cal['stereo']['left']['k'][0]==35
        assert cal['stereo']['left']['k'][2]==19.5
    finally:
        preview.close()


def test_jpeg_cannot_be_accepted_as_authoritative_raw():
    import cv2
    h,p=image('left',1)
    data=cv2.imencode('.jpg',decode_image(h,p))[1].tobytes()
    with pytest.raises(ValueError):decode_image({**h,'compression':'jpeg'},data)


def test_copy_resumes_after_disconnect_and_checks_hash(tmp_path):
    data=b'original raw\x00'*100
    entry=dict(name='capture.bin',size=len(data),sha256=hashlib.sha256(data).hexdigest())
    offsets=[]
    def fetch(name,offset):
        offsets.append(offset)
        block=data[offset:200] if len(offsets)==1 else data[offset:]
        return subprocess.Popen([sys.executable,'-c',
            'import sys;sys.stdout.buffer.write('+repr(block)+');sys.exit('+('1' if len(offsets)==1 else '0')+')'],
            stdout=subprocess.PIPE)
    target=tmp_path/'capture.bin'
    receive_file(entry,target,fetch,lambda *a,**kw:None,retry_wait=.001)
    assert offsets==[0,200] and target.read_bytes()==data
    assert not (tmp_path/'capture.bin.part').exists()
    target.unlink()
    (tmp_path/'capture.bin.part').write_bytes(b'x'*len(data))
    with pytest.raises(ValueError,match='체크섬'):
        receive_file(entry,target,fetch,lambda *a,**kw:None)
    assert not target.exists()


def sensor_script():
    # Real packet writer and stop control, no ROS or camera mocked in the recorder.
    return '''import sys,time,select,zlib,json,numpy as np
from experiments.handheld_mapping.protocol import write_packet
out=sys.stdout.buffer
cal=json.loads(CAL)
write_packet(out,cal)
write_packet(out,dict(kind='ready'))
n=0
while not select.select([sys.stdin],[],[],0)[0]:
 n+=1
 for side in ['left','right','depth']:
  if n==1 and side=='depth':continue
  a=np.full((60,80),1000 if side=='depth' else 100,dtype=np.uint16 if side=='depth' else np.uint8)
  write_packet(out,dict(kind='image',stream=side,stamp_ns=n*66666667,width=80,height=60,step=160 if side=='depth' else 80,encoding='16UC1' if side=='depth' else 'mono8',compression='zlib',frame_id=side,pi_receipt_monotonic_ns=time.monotonic_ns()),zlib.compress(a.tobytes(),1))
 time.sleep(.03)
write_packet(out,dict(kind='finished',status='saved',capture_mode='sensors_only',dropped=0))
'''.replace('CAL',repr(json.dumps(calibration())))


def test_detached_recorder_keeps_originals_and_imu_after_preview_disconnect(tmp_path):
    from test_imu_capture import CALIBRATION, AXES
    path=tmp_path/'run';path.mkdir()
    (path/'config.json').write_text(json.dumps(dict(imu_enabled=True)))
    (path/'input_calibration.json').write_text(json.dumps(CALIBRATION))
    (path/'input_robot.urdf').write_text('<robot name="test"/>')
    imu_script='''import sys,time,json,select
print(json.dumps(dict(kind='ready',chip_to_body=AXES)),flush=True)
n=10000
while not select.select([sys.stdin],[],[],0)[0]:
 n+=20
 print(json.dumps(dict(kind='telemetry',schema=1,host_monotonic_ns=time.monotonic_ns(),host_receipt_ns=time.time_ns(),tee_drops=0,telemetry=dict(flags=16,t_ms=n,tick=n%256,gyro_dps=[.5,1.4,1.6],acc_g=[.01,.02,1.03]))),flush=True)
 time.sleep(.02)
'''.replace('AXES',repr(AXES))
    command=[sys.executable,'-u','-c',sensor_script()]
    imu_command=[sys.executable,'-u','-c',imu_script]
    code='from experiments.handheld_mapping.local_recording import record; record('+repr(str(path))+',worker_command='+repr(command)+',imu_command='+repr(imu_command)+',min_free=1)'
    process=subprocess.Popen([sys.executable,'-u','-c',code],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    attachment=None
    try:
        until(lambda:(path/'ready').exists())
        attachment=subprocess.Popen([sys.executable,'-u','-c',
            'from experiments.handheld_mapping.local_recording import attach;attach('+repr(str(path))+')'],
            stdin=subprocess.PIPE,stdout=subprocess.DEVNULL)
        until(lambda:(path/'status.json').exists())
        before=json.loads((path/'status.json').read_text())['counts']['left']
        attachment.terminate();attachment.wait(timeout=5)
        until(lambda:json.loads((path/'status.json').read_text())['counts']['left']>before+5)
        (path/'stop').touch()
        assert process.wait(timeout=15)==0,process.stderr.read().decode()
        done=json.loads((path/'done.json').read_text())
        assert done['error'] is None,done
        assert done['counts']['left']>before+5
        assert done['counts']['left']==done['counts']['depth']+1
        assert all(digest(path/f['name'])==f['sha256'] for f in done['files'])
        rows=[json.loads(s) for s in (path/'imu/run/imu.jsonl').read_text().splitlines()]
        assert any(r['acceptance']=='accepted' for r in rows)
        assert 'pi_record_monotonic_ns' in rows[-1]['clock']
        assert 'laptop_receipt_monotonic_ns' not in rows[-1]['clock']
        with (path/'capture.bin').open('rb') as f:
            raw=[]
            while (packet:=read_packet(f)) is not None:
                if packet[0]['kind']=='image':raw.append(packet[0])
        assert len(raw)==sum(done['counts'].values())
        assert all(not h.get('preview') and h['width']==80 for h in raw)
    finally:
        (path/'stop').touch()
        if attachment and attachment.poll() is None:attachment.terminate();attachment.wait(timeout=5)
        if process.poll() is None:process.terminate();process.wait(timeout=5)
        process.stdout.close();process.stderr.close()


@pytest.mark.parametrize('capture_error', [None, '카메라 USB 연결 끊김'])
def test_session_imports_verified_originals_before_final_mapping(tmp_path,monkeypatch,capsys,capture_error):
    from experiments.handheld_mapping.controller import Session
    from experiments.handheld_mapping import offline,reconstruct
    from test_session import wait_state
    source=tmp_path/'source.bin'
    with source.open('wb') as f:
        write_packet(f,preview_calibration(calibration()))
        write_packet(f,dict(kind='ready',preview=True))
        if capture_error:
            write_packet(f,dict(kind='capture_failure',message=capture_error))
        # Controller must retain a separate preview calibration and raw calibration.
        write_packet(f,dict(kind='transfer_begin',remote_path='/pi/recordings/test'))
        write_packet(f,dict(kind='transfer_progress',bytes=5,total=10))
        write_packet(f,dict(kind='originals_begin',verified=True))
        write_packet(f,calibration())
        for side in ['depth','left','right']:write_packet(f,*image(side,1))
        write_packet(f,dict(kind='finished',status='saved',capture_mode='sensors_only',dropped=0))
        if capture_error:
            write_packet(f,dict(kind='capture_error',message=capture_error))
            write_packet(f,dict(kind='error',message=capture_error))
        else:
            write_packet(f,dict(kind='transfer_complete',verified=True,counts=dict(left=1,right=1,depth=1)))
    seen=[]
    def final_mapping(run):
        d=json.loads((run/'capture_calibration.json').read_text())
        assert d['depth']['width']==80 and not d.get('preview')
        assert json.loads((run/'preview/capture_calibration.json').read_text())['depth']['width']==40
        seen.append(run)
        return dict(status='complete')
    monkeypatch.setattr(offline,'run_point',final_mapping)
    monkeypatch.setattr(reconstruct,'reconstruct',lambda *a,**kw:dict(status='complete'))
    session=Session()
    session.start(dict(output_root=str(tmp_path/'sessions'),local_recording=True,
        worker_command=[sys.executable,'-c', 'import sys;sys.stdout.buffer.write(open('+repr(str(source))+',"rb").read())']))
    result=wait_state(session,{'complete','partial','failed'})
    session.thread.join(timeout=5)
    text=capsys.readouterr().out
    if capture_error:
        assert result['state']=='failed' and seen==[],result
        assert result['counts']==dict(left=1,right=1,depth=1)
        assert text.index('촬영 오류로 중단')<text.index('원본 전송 50%')
        assert '매핑 실패: '+capture_error in text
    else:
        assert result['state']=='complete',result
        assert result['transfer_verified'] and len(seen)==1
        assert '기다려 주세요' in text and '전송 및 검증 완료' in text


@pytest.mark.parametrize('capture_error', [None, 'camera disconnected'])
def test_transport_reconnects_preview_then_copies_and_imports_raw(tmp_path,monkeypatch,capture_error):
    import queue
    from experiments.handheld_mapping import local_client
    remote=tmp_path/'remote';remote.mkdir()
    archive=remote/'capture.bin'
    with archive.open('wb') as stream:
        write_packet(stream,calibration())
        for side in ['depth','left','right']:write_packet(stream,*image(side,1))
        write_packet(stream,dict(kind='finished',status='saved',capture_mode='sensors_only',dropped=0))
        if capture_error:
            write_packet(stream,dict(kind='error',message=capture_error))
    manifest=dict(kind='local_done',files=[dict(name='capture.bin',size=archive.stat().st_size,sha256=digest(archive))],
                  counts=dict(left=1,right=1,depth=1),error=capture_error)
    packets=io.BytesIO()
    write_packet(packets,preview_calibration(calibration()))
    for side in ['left','right']:
        h,p=image(side,1);write_packet(packets,{**h,'preview':True},p)
    partial=remote/'first.bin';partial.write_bytes(packets.getvalue())
    write_packet(packets,manifest)
    completed=remote/'second.bin';completed.write_bytes(packets.getvalue())
    calls=[]
    def ssh(host,run_id,action,*options):
        calls.append(action)
        if action=='start':
            source=partial if calls.count('start')==1 else completed
            script='import sys;sys.stdout.buffer.write(open('+repr(str(source))+',"rb").read())'
        else:
            assert action=='fetch'
            offset=int(options[options.index('--offset')+1])
            script='import sys;f=open('+repr(str(archive))+',"rb");f.seek('+str(offset)+');sys.stdout.buffer.write(f.read())'
        return [sys.executable,'-u','-c',script]
    monkeypatch.setattr(local_client,'ssh_command',ssh)
    original_sleep=time.sleep
    monkeypatch.setattr(local_client.time,'sleep',lambda value:original_sleep(min(value,.001)))
    run=tmp_path/'laptop';run.mkdir()
    controls=queue.Queue();controls.put('stop')
    emitted=[]
    if capture_error:
        with pytest.raises(RuntimeError,match='camera disconnected'):
            local_client.transport('pi@fixture','run',run,lambda h,p=b'':emitted.append((h,p)),controls)
    else:
        local_client.transport('pi@fixture','run',run,lambda h,p=b'':emitted.append((h,p)),controls)
    kinds=[h['kind'] for h,p in emitted]
    assert calls.count('start')==2 and 'connection_wait' in kinds
    assert sum(h['kind']=='calibration' and h.get('preview',False) for h,p in emitted)==1
    assert sum(h['kind']=='image' and h.get('preview',False) for h,p in emitted)==2
    assert kinds.index('originals_begin')<max(i for i,k in enumerate(kinds) if k=='calibration')
    if capture_error:
        assert kinds.index('capture_failure')<kinds.index('originals_begin')
        assert 'capture_error' in kinds and 'error' not in kinds
        assert 'transfer_complete' not in kinds
    else:
        assert kinds[-1]=='transfer_complete'
    assert digest(run/'pi_originals/capture.bin')==digest(archive)


def test_start_detaches_real_module_and_does_not_restart_existing_recording(tmp_path,monkeypatch):
    import os
    from types import SimpleNamespace
    from experiments.handheld_mapping import local_recording
    path=tmp_path/'run';path.mkdir()
    monkeypatch.setattr(local_recording,'RECORDINGS',tmp_path)
    monkeypatch.setattr(local_recording,'__name__','__main__')
    monkeypatch.setattr(sys,'argv',['local_recording','start','run'])
    spawned=[];attached=[]
    def spawn(command,**options):
        spawned.append((command,options))
        return SimpleNamespace(pid=os.getpid())
    monkeypatch.setattr(local_recording.subprocess,'Popen',spawn)
    monkeypatch.setattr(local_recording,'attach',attached.append)
    local_recording.main();local_recording.main()
    assert len(spawned)==1 and attached==[path,path]
    command,options=spawned[0]
    assert command[command.index('-m')+1]=='experiments.handheld_mapping.local_recording'
    assert options['start_new_session'] and options['stdin']==subprocess.DEVNULL


def test_capture_failure_explains_usb_disconnect_without_blaming_wireless():
    from experiments.handheld_mapping.local_recording import capture_failure_reason
    log='Device USB type: 2.1\nThe device has been disconnected!\nNo such device'
    reason=capture_failure_reason(['input stopped'],log)
    assert 'USB 연결 끊김' in reason and 'USB 2.1' in reason
    assert 'input stopped' in reason
    assert capture_failure_reason([],log) is None


def test_camera_diagnostic_transfer_is_bounded_and_explicitly_truncated(tmp_path):
    from experiments.handheld_mapping.local_recording import collect_camera_diagnostics
    source=tmp_path/'source.log';target=tmp_path/'camera.log'
    raw=b'Device USB type: 2.1\n'+b'x'*400000+b'\nThe device has been disconnected!'
    source.write_bytes(raw)
    text=collect_camera_diagnostics(source,target)
    assert target.stat().st_size<=256*1024
    assert 'truncated' in text and 'USB type: 2.1' in text and 'disconnected!' in text
    assert source.read_bytes()==raw
