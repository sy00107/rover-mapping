import csv
import json
from pathlib import Path
import subprocess
import sys
import tarfile
import time

import pytest
from experiments.handheld_mapping import field_collection as f


def test_init_keeps_existing_folder_and_records_unknown_metadata(tmp_path):
    root=f.init(tmp_path/'field')
    assert (root/'sessions').is_dir()
    assert f.read_json(root/'dataset.json')['stl_from_mapping'] is None
    (root/'keep.txt').write_text('original')
    with pytest.raises(FileExistsError):f.init(root)
    assert (root/'keep.txt').read_text()=='original'


def test_trial_gt_and_full_telemetry_fields(tmp_path):
    root=f.init(tmp_path/'field');run=root/'sessions/run1';run.mkdir()
    f.write_json(root/'active_run.json',{'run_id':'run1'})
    f.mark(root,'begin','sand_01','sand')
    event=f.mark(root,'checkpoint',checkpoint_id='cp1',x_m=1.,y_m=2.,heading_deg=90.)
    assert event['run_id']=='run1' and event['trial_id']=='sand_01'
    assert event['laptop_monotonic_ns']>0
    with (root/'ground_truth/measurements.csv').open() as stream:
        row=list(csv.DictReader(stream))[0]
    assert row['x_m']=='1.0' and row['surface']=='sand'
    with pytest.raises(ValueError):f.mark(root,'distance',actual_distance_m=float('nan'))
    (run/'imu.jsonl').write_text(json.dumps({'raw':{'selected_cmd':[.1,0,0],
        'telemetry':{'t_ms':123,'cmd':[.09,0,0],'enc_ticks':[1,2,3,4],'wheel_mps':[.1]*4}},
        'clock':{'laptop_receipt_monotonic_ns':55,'pi_receipt_monotonic_ns':44},'acceptance':'accepted'})+'\n')
    out=tmp_path/'derived';out.mkdir();report=f.extract_telemetry(run,out)
    with (out/'telemetry.csv').open() as stream:row=list(csv.DictReader(stream))[0]
    assert row['enc_ticks_3']=='4' and row['selected_cmd_0']=='0.1' and row['mcu_t_ms']=='123'
    assert report['rows_with_encoder']==1


def test_archive_relocated_dataset_verifies_and_detects_corruption(tmp_path):
    root=f.init(tmp_path/'field');(root/'photos/color/test.txt').write_text('photo fixture')
    archive=f.pack(root,tmp_path/'share/field.tar.gz')
    dest=tmp_path/'received';dest.mkdir()
    with tarfile.open(archive) as tar:tar.extractall(dest)
    relocated=dest/'field';f.verify(relocated)
    result=subprocess.run([sys.executable,str(relocated/'verify_dataset.py')],capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    (relocated/'photos/color/test.txt').write_text('corrupted')
    with pytest.raises(ValueError,match='불일치'):f.verify(relocated)
    with f.lock(root,'.capture.lock',False):
        with pytest.raises(ValueError):f.pack(root,tmp_path/'busy.tar.gz')


def test_demo_run_auto_creates_portable_map_and_reports_missing_real_metadata(tmp_path):
    root=f.init(tmp_path/'field');session=f.make_field_session(root)
    session.start({'demo':True,'output_root':str(tmp_path/'ignored'),'live_mapping':False})
    deadline=time.monotonic()+20
    while session.snapshot()['state'] not in {'recording','failed'} and time.monotonic()<deadline:time.sleep(.02)
    assert session.snapshot()['state']=='recording',session.snapshot()
    time.sleep(.25);session.stop();session.thread.join(timeout=40)
    assert not session.thread.is_alive()
    assert session.snapshot()['state']=='complete',session.snapshot()
    run=Path(session.snapshot()['run_dir']);assert run.parent==root/'sessions'
    pointer=f.read_json(root/'derived'/run.name/'latest.json');folder=root/pointer['folder']
    report=f.read_json(folder/'report.json')
    assert report['maps'] and report['camera_metadata_records']==0 and report['missing']
    assert (folder/report['maps'][0]['mesh']).is_file()
    assert f.current_run(root)==run.name


def test_device_metadata_preserves_raw_payload_and_receipt_clock():
    from types import SimpleNamespace as S
    from experiments.handheld_mapping.pi_worker import camera_metadata_record,field_parameter_snapshot
    record=camera_metadata_record('left',S(json_data='{"frame_timestamp":12345}',header=S(stamp=S(sec=2,nanosec=3),frame_id='left')))
    assert record['json_data']=='{"frame_timestamp":12345}' and record['stamp_ns']==2000000003
    assert record['pi_receipt_monotonic_ns']>0
    settings=field_parameter_snapshot(lambda names:[S(type=2,integer_value=42)])
    assert settings['depth_module.exposure']==42
    assert all(v is None for v in field_parameter_snapshot(lambda names:[]).values())
