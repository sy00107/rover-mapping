"""Separate field dataset, synchronized manual labels, portable map previews and share bundles."""
import argparse
from contextlib import contextmanager
import csv
from datetime import datetime,timezone
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import sys
import tarfile
import time
import uuid

ROOT=Path(__file__).resolve().parents[2]


def clock():
    return dict(utc=datetime.now(timezone.utc).isoformat(),laptop_realtime_ns=time.time_ns(),
                laptop_monotonic_ns=time.monotonic_ns())


def read_json(path,default=None):
    return json.loads(Path(path).read_text()) if Path(path).is_file() else default


def write_json(path,data):
    from .storage import atomic_json
    atomic_json(Path(path),data)


def dataset(path):
    path=Path(path).expanduser().resolve()
    if not (path/'dataset.json').is_file():raise ValueError('수집 폴더의 dataset.json이 필요합니다.')
    return path


@contextmanager
def lock(root,name='.dataset.lock',blocking=True):
    with (Path(root)/name).open('a') as stream:
        try:fcntl.flock(stream,fcntl.LOCK_EX|(0 if blocking else fcntl.LOCK_NB))
        except BlockingIOError:raise ValueError('기록 앱이 실행 중입니다. 종료 후 다시 실행하세요.')
        try:yield
        finally:fcntl.flock(stream,fcntl.LOCK_UN)


def safe_id(value):
    import re
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,79}',value):raise ValueError('ID는 영문·숫자·밑줄·하이픈으로 입력하세요.')
    return value


def init(root,checklist=None):
    root=Path(root).expanduser().resolve();root.mkdir(parents=True,exist_ok=False)
    for name in ['sessions','stationary','ground_truth','photos/color','photos/checkpoint_ir','photos/ruts',
                 'photos/environment','reference_stl','derived','logs']:(root/name).mkdir(parents=True,exist_ok=True)
    commit=subprocess.run(['git','rev-parse','HEAD'],cwd=ROOT,capture_output=True,text=True).stdout.strip()
    dirty=bool(subprocess.run(['git','status','--porcelain'],cwd=ROOT,capture_output=True,text=True).stdout)
    write_json(root/'dataset.json',dict(schema=1,dataset_id=root.name,created=clock(),site='Yeouido',code_commit=commit,
        code_dirty=dirty,notes='센서 원본과 GT를 구분합니다. null은 미측정입니다.',
        units=dict(position='metre',heading='degree'),camera_pitch_down_deg=20,
        weather=None,ground_surface=None,battery_voltage_v=None,
        reference_stl_file=None,stl_units=None,
        stl_from_mapping=None,stl_transform_note='STL 기준 좌표와 매핑 시작 좌표 관계를 실측 후 입력; 임의 identity 사용 금지',
        rgb_source='별도 카메라/휴대폰 사진 수동 추가; 현재 D435 스트림은 IR+depth',
        time_alignment='device/MCU/Pi/laptop clocks are preserved; offsets are not assumed verified'))
    with (root/'ground_truth/measurements.csv').open('x',newline='') as f:
        csv.writer(f).writerow(['event_id','utc','run_id','trial_id','kind','surface','checkpoint_id','x_m','y_m','z_m',
                               'heading_deg','command_distance_m','actual_distance_m','battery_voltage_v','note'])
    templates=Path(__file__).parent/'field_templates'
    for name in ['view_maps.py','verify_dataset.py']:
        shutil.copy2(templates/name,root/name)
    snapshot=root/'code_snapshot/experiments/handheld_mapping';snapshot.mkdir(parents=True)
    for source in Path(__file__).parent.glob('*.py'):shutil.copy2(source,snapshot/source.name)
    (snapshot/'field_templates').mkdir()
    for source in templates.glob('*.py'):shutil.copy2(source,snapshot/'field_templates'/source.name)
    if checklist:shutil.copy2(Path(checklist).expanduser(),root/'collection_checklist.md')
    (root/'README.md').write_text('''# 여의도 데이터 묶음

`sessions/`: IR 좌우·z16 depth PNG·MCU/Pi/노트북 시계·IMU raw/보정·명령·encoder·최종 지도.
`stationary/`: 주행 전후 카메라 없이 1분 IMU 기록. 매핑과 동시 실행하지 않습니다.
`ground_truth/measurements.csv`: 줄자 실측. 좌표 m, heading 도, STL 기준 좌표임을 확인하세요.
`field_events.jsonl`: 수동 구간/체크포인트 표식과 노트북 시각. 센서 시각과 같은 것으로 간주하지 않습니다.
`photos/`: 컬러/바퀴자국/환경 사진을 직접 넣습니다. checkpoint_ir은 종료 후 자동 추출됩니다.
`reference_stl/`: 팀원 실측 STL과 기준점 설명을 넣습니다. dataset.json에 단위와 좌표변환을 적으세요.
`derived/RUN/REV/`: 명령·엔코더 CSV, 카메라 metadata, 공유용 지도 PLY+경로, 품질 요약.
`dataset.json`: 날씨·배터리·장착각·STL 좌표관계. null은 미측정입니다.

원본 폴더를 삭제하지 않습니다. RAW IMU는 MCU 제어주기에서 전달된 데이터이며 센서 내부 208Hz 원시 샘플 전체가 아닐 수 있습니다.
서로 다른 clock 간 -47ms 같은 보정값을 자동으로 적용하지 않습니다.
팀원은 코드 main_v2와 별도로 이 폴더 전체를 전달받아 field_collection view로 PLY를 엽니다.
''')
    return root


def current_run(root):
    current=read_json(root/'active_run.json',{})
    name=current.get('run_id')
    return name if name and (root/'sessions'/safe_id(name)).is_dir() else None


def mark(root,kind,trial_id=None,surface=None,**values):
    root=dataset(root)
    numeric={'x_m','y_m','z_m','heading_deg','command_distance_m','actual_distance_m','battery_voltage_v'}
    for key in numeric&values.keys():
        if values[key] is not None and not math.isfinite(float(values[key])):raise ValueError('측정값은 유한한 수치여야 합니다.')
    if trial_id:safe_id(trial_id)
    with lock(root):
        active=read_json(root/'active_trial.json',{})
        if kind=='begin' and not trial_id:raise ValueError('begin에는 실험 ID가 필요합니다.')
        row=dict(event_id=uuid.uuid4().hex,**clock(),run_id=current_run(root),kind=kind,
                 trial_id=trial_id or active.get('trial_id'),surface=surface or active.get('surface'),**values)
        with (root/'field_events.jsonl').open('a') as f:f.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
        if kind=='begin':write_json(root/'active_trial.json',dict(trial_id=row['trial_id'],surface=row['surface']))
        if kind=='end':write_json(root/'active_trial.json',{})
        if kind in ('checkpoint','distance','battery'):
            path=root/'ground_truth/measurements.csv'
            with path.open() as f:fields=next(csv.reader(f))
            with path.open('a',newline='') as f:csv.DictWriter(f,fields,extrasaction='ignore').writerow(row)
    print(f"{kind}: run={row['run_id'] or '연결 없음'} trial={row['trial_id']} {values}",flush=True)
    return row


def iter_jsonl(path):
    if not Path(path).is_file():return
    with Path(path).open() as f:
        for line in f:
            if line.strip():yield json.loads(line)


def extract_telemetry(run,out):
    scalar=['laptop_monotonic_ns','pi_monotonic_ns','mcu_t_ms','segment','acceptance','stale']
    vector={'selected_cmd':3,'udp_nominal_cmd':3,'stm_cmd':3,'enc_ticks':4,'wheel_mps':4,
            'gyro_dps':3,'acc_g':3,'steer_volt':4}
    fields=scalar+[f'{name}_{i}' for name,n in vector.items() for i in range(n)]
    count=0;valid_enc=0
    with (out/'telemetry.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fields);writer.writeheader()
        for row in iter_jsonl(run/'imu.jsonl'):
            raw=row.get('raw',{});t=raw.get('telemetry') or {};c=row.get('clock',{})
            item=dict(laptop_monotonic_ns=c.get('laptop_receipt_monotonic_ns'),pi_monotonic_ns=c.get('pi_receipt_monotonic_ns'),
                mcu_t_ms=t.get('t_ms'),segment=c.get('segment'),acceptance=row.get('acceptance'),stale=raw.get('stale'))
            values=dict(selected_cmd=raw.get('selected_cmd'),udp_nominal_cmd=raw.get('udp_nominal_cmd'),stm_cmd=t.get('cmd'),
                        **{name:t.get(name) for name in ['enc_ticks','wheel_mps','gyro_dps','acc_g','steer_volt']})
            for name,n in vector.items():
                v=values[name]
                for i in range(n):item[f'{name}_{i}']=v[i] if isinstance(v,(list,tuple)) and len(v)>i else None
            writer.writerow(item);count+=1
            valid_enc+=int(isinstance(t.get('enc_ticks'),(list,tuple)) and len(t['enc_ticks'])==4)
    return dict(rows=count,rows_with_encoder=valid_enc,steer_volt_note='steering servo voltage, NOT battery pack voltage')


def portable_maps(run,out):
    import numpy as np
    import open3d as o3d
    from .viewer import select_maps,load_trajectory,map_frame
    from .surface_colors import apply_surface_colors
    result=[]
    try:selected=select_maps(run)
    except (ValueError,OSError):return result
    for label,source in selected:
        identity=label.split()[-1]
        mesh=o3d.io.read_triangle_mesh(str(source));mesh.compute_vertex_normals()
        warning=None
        try:apply_surface_colors(mesh,source,fill_ground_gaps=True,smooth_obstacles=True)
        except (ValueError,OSError) as error:warning=str(error)
        name=f'map_{identity}.ply';o3d.io.write_triangle_mesh(str(out/name),mesh)
        trajectory=load_trajectory(source)
        np.savetxt(out/f'map_{identity}_trajectory.csv',np.asarray(trajectory).reshape(-1,3),delimiter=',',header='x_m,y_m,z_m',comments='')
        result.append(dict(map_id=int(identity),mesh=name,trajectory=f'map_{identity}_trajectory.csv',
            coordinate_system='gravity_z_up' if map_frame(source)[1] else 'initial_camera_optical',
            source=str(source.relative_to(run)),warning=warning))
    return result


def checkpoint_images(root,run,out):
    events=[e for e in iter_jsonl(root/'field_events.jsonl') if e.get('kind')=='checkpoint' and e.get('run_id')==run.name]
    if not events:return []
    # Post-capture import receipts are transfer times. Manual annotations must
    # match live preview receipts, whose source stamps also identify full raw frames.
    frame_index=run/'preview/frames.jsonl' if (run/'preview/frames.jsonl').exists() else run/'frames.jsonl'
    frames=[f for f in iter_jsonl(frame_index) if f.get('stream')=='left' and isinstance(f.get('laptop_receipt_monotonic_ns'),int)]
    results=[]
    for event in events:
        if not frames:break
        frame=min(frames,key=lambda f:abs(f['laptop_receipt_monotonic_ns']-event['laptop_monotonic_ns']))
        delta=(frame['laptop_receipt_monotonic_ns']-event['laptop_monotonic_ns'])/1e9
        # The annotation is laptop receipt time, not hardware exposure time.
        info=dict(event_id=event['event_id'],checkpoint_id=event.get('checkpoint_id'),receipt_time_difference_s=delta,
                  source_frame=frame['path'],matched=abs(delta)<=3.)
        if info['matched']:
            target=root/'photos/checkpoint_ir'/(event['event_id']+'.png')
            if not target.exists():shutil.copy2(run/frame['path'],target)
            info['image']=str(target.relative_to(root))
        results.append(info)
    return results


def finalize_run(root,run):
    root=dataset(root);run=Path(run).resolve()
    if not run.is_relative_to(root/'sessions') and not run.is_relative_to(root/'stationary'):
        raise ValueError('수집 폴더 내부 기록만 정리합니다.')
    revision=datetime.now().strftime('%Y%m%d_%H%M%S_')+uuid.uuid4().hex[:4]
    out=root/'derived'/run.name/revision;out.mkdir(parents=True,exist_ok=False)
    telemetry=extract_telemetry(run,out)
    camera_count=0;metadata_fields=set()
    with (out/'camera_metadata.jsonl').open('w') as f:
        for row in iter_jsonl(run/'events.jsonl'):
            if row.get('kind')!='camera_metadata':continue
            f.write(json.dumps(row,allow_nan=False)+'\n');camera_count+=1
            try:metadata_fields.update(json.loads(row['json_data']).keys())
            except (ValueError,TypeError):pass
    maps=portable_maps(run,out) if run.parent.name=='sessions' else []
    missing=[]
    normalized={str(name).lower() for name in metadata_fields}
    if run.parent.name=='sessions':
        if not camera_count:missing.append('camera raw metadata not received')
        if not any('timestamp' in name for name in normalized):missing.append('camera device-clock metadata not verified')
        if not any('exposure' in name for name in normalized):missing.append('per-frame exposure metadata not received')
        if not telemetry['rows_with_encoder']:missing.append('encoder telemetry not received')
    report=dict(schema=1,run_id=run.name,created=clock(),session=read_json(run/'session.json',{}),
        imu_summary=read_json(run/'imu_summary.json',{}),telemetry=telemetry,maps=maps,
        camera_settings=read_json(run/'capture_calibration.json',{}).get('field_camera_settings'),
        camera_metadata_records=camera_count,camera_metadata_fields=sorted(metadata_fields),
        checkpoints=checkpoint_images(root,run,out),
        missing=missing,
        manual_items=['color photos with fixed exposure','tape-measured GT','weather','battery pack voltage','reference STL frame transform'])
    write_json(out/'report.json',report)
    write_json(root/'derived'/run.name/'latest.json',dict(folder=str(out.relative_to(root))))
    print(f'공유용 정리 완료: {out}',flush=True)
    return report


def newest_run(root):
    paths=[p for p in (root/'sessions').iterdir() if p.is_dir() and (p/'session.json').is_file()]
    if not paths:raise ValueError('수집 폴더에 촬영 기록이 없습니다.')
    return max(paths,key=lambda p:p.name)


def make_field_session(root):
    from .controller import Session
    root=dataset(root)
    class FieldSession(Session):
        def start(self,config):
            config=dict(config,output_root=str(root/'sessions'),field_metadata=True)
            if not config.get('demo') and not config.get('imu_enabled'):
                raise ValueError('현장 수집은 IMU 함께 기록을 켜주세요.')
            return super().start(config)
        def update(self,**values):
            super().update(**values)
            if values.get('run_dir'):
                run=Path(values['run_dir'])
                if run.parent==root/'sessions':write_json(root/'active_run.json',dict(run_id=run.name,**clock()))
        def _run(self,config):
            super()._run(config)
            path=self.snapshot().get('run_dir')
            if path:
                try:
                    finalize_run(root,Path(path))
                    self.update(message=self.snapshot()['message']+' · 공유용 지도/CSV 정리 완료')
                except Exception as error:
                    self.update(message=self.snapshot()['message']+' · 공유 정리 오류: '+str(error))
    return FieldSession()


def launch_app(root,demo=False):
    from .app import MappingWindow
    from PyQt5.QtWidgets import QApplication
    root=dataset(root)
    with lock(root,'.capture.lock',False):
        app=QApplication.instance() or QApplication(sys.argv[:1]);session=make_field_session(root)
        window=MappingWindow(session,root/'sessions',demo)
        window.setWindowTitle('여의도 현장 수집 · '+root.name)
        window.host.setText(os.environ.get('FIELD_PI_HOST','pi@raspberrypi.local'))
        signal.signal(signal.SIGINT,lambda *_:window.close());signal.signal(signal.SIGTERM,lambda *_:window.close())
        window.show();app.exec_()


def stationary(root,phase,seconds,host,calibration):
    from .controller import deploy
    from .imu import ImuCapture
    root=dataset(root)
    if not math.isfinite(seconds) or seconds<=0:raise ValueError('seconds must be positive')
    with lock(root,'.capture.lock',False):
        run=root/'stationary'/(phase+'_'+datetime.now().strftime('%Y%m%d_%H%M%S_')+uuid.uuid4().hex[:4]);run.mkdir()
        capture=None;status='failed';started=time.monotonic()
        try:
            deploy(host,run.name)
            capture=ImuCapture(host,run,Path(calibration).expanduser(),ROOT/'ros2_ws/src/rover_description/urdf/rocker_bogie_4wd_rover.urdf')
            capture.start();started=time.monotonic()
            print(f'정지 IMU {phase}: {seconds:g}초 기록 중. 움직이지 마세요. Ctrl+C는 조기 마감.',flush=True)
            while time.monotonic()-started<seconds:
                if capture.snapshot().get('state')!='recording':raise RuntimeError('IMU 기록 중단')
                time.sleep(.2)
            status='complete'
        except KeyboardInterrupt:status='partial'
        finally:
            if capture is not None:capture.stop()
            write_json(run/'session.json',dict(state=status,phase=phase,requested_duration_s=seconds,
                elapsed_duration_s=time.monotonic()-started,camera_recorded=False))
            finalize_run(root,run)
    return run


def console(root):
    print('begin ID sand|hard / cp 이름 x_m y_m heading_deg / distance 명령_m 실측_m / battery 전압 / note 메모 / end / quit',flush=True)
    while True:
        try:line=input('수집> ').strip()
        except (EOFError,KeyboardInterrupt):break
        if not line:continue
        parts=shlex.split(line)
        try:
            if parts[0]=='quit':break
            if parts[0]=='begin' and len(parts)==3:mark(root,'begin',parts[1],parts[2])
            elif parts[0]=='cp' and len(parts)==5:
                mark(root,'checkpoint',checkpoint_id=parts[1],x_m=float(parts[2]),y_m=float(parts[3]),heading_deg=float(parts[4]))
            elif parts[0]=='distance' and len(parts)==3:
                mark(root,'distance',command_distance_m=float(parts[1]),actual_distance_m=float(parts[2]))
            elif parts[0]=='battery' and len(parts)==2:mark(root,'battery',battery_voltage_v=float(parts[1]))
            elif parts[0]=='note':mark(root,'note',note=' '.join(parts[1:]))
            elif parts[0]=='end':mark(root,'end')
            else:print('형식을 확인하세요. cp 위치는 STL 기준 m, 방향은 도입니다.')
        except (ValueError,OSError) as error:print(error)


def view(root,run_id=None):
    import numpy as np,open3d as o3d
    from .viewer import trajectory_geometries
    root=dataset(root);run_id=safe_id(run_id) if run_id else newest_run(root).name
    pointer=read_json(root/'derived'/run_id/'latest.json')
    if not pointer:raise ValueError('먼저 finalize를 실행하세요.')
    folder=(root/pointer['folder']).resolve()
    if not folder.is_relative_to(root):raise ValueError('Invalid preview folder')
    report=read_json(folder/'report.json');maps=report['maps']
    if not maps:raise ValueError('생성된 지도가 없습니다. 보고서의 촬영/매핑 상태를 확인하세요.')
    # Open one map at a time: independent SLAM map frames must never be silently combined.
    for item in maps:
        mesh=o3d.io.read_triangle_mesh(str(folder/item['mesh']));mesh.compute_vertex_normals()
        trajectory=np.loadtxt(folder/item['trajectory'],delimiter=',',skiprows=1).reshape(-1,3)
        geometries=[mesh]+trajectory_geometries(trajectory.tolist())
        v=o3d.visualization.Visualizer();v.create_window(window_name=f"Field map | {run_id} | map {item['map_id']}",width=1200,height=850)
        for g in geometries:v.add_geometry(g)
        opt=v.get_render_option();opt.background_color=[.07,.09,.12];opt.mesh_show_back_face=True
        c=v.get_view_control();c.set_up([0,0,1] if item['coordinate_system']=='gravity_z_up' else [0,-1,0]);c.set_zoom(.7)
        v.run();v.destroy_window()


def sha256(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda:f.read(4*1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def pack(root,output):
    root=dataset(root);output=Path(output).expanduser().resolve()
    if output==root or output.is_relative_to(root):raise ValueError('압축 파일은 수집 폴더 밖에 저장하세요.')
    with lock(root,'.capture.lock',False),lock(root):
        files=[p for p in sorted(root.rglob('*')) if p.is_file() and p.name not in ['manifest.json','.capture.lock','.dataset.lock']]
        if any(p.is_symlink() for p in root.rglob('*')):raise ValueError('심볼릭 링크는 해제 후 실제 파일로 넣어주세요.')
        records=[]
        for path in files:
            if path.suffix in ('.pem','.key') or path.name.startswith('.env'):raise ValueError('인증 파일은 공유할 수 없습니다.')
            records.append(dict(path=path.relative_to(root).as_posix(),size=path.stat().st_size,sha256=sha256(path)))
        write_json(root/'manifest.json',dict(schema=1,dataset_id=root.name,files=records))
        output.parent.mkdir(parents=True,exist_ok=True)
        with output.open('xb') as stream:
            with tarfile.open(fileobj=stream,mode='w:gz',compresslevel=1) as tar:
                for record,path in zip(records,files):
                    before=path.stat()
                    if before.st_size!=record['size'] or sha256(path)!=record['sha256']:raise ValueError('기록이 변경됐습니다. 다시 묶어주세요.')
                    tar.add(path,arcname=root.name+'/'+record['path'],recursive=False)
                    if path.stat().st_mtime_ns!=before.st_mtime_ns:raise ValueError('압축 중 파일이 변경됐습니다.')
                tar.add(root/'manifest.json',arcname=root.name+'/manifest.json',recursive=False)
        digest=sha256(output)
        Path(str(output)+'.sha256').write_text(digest+'  '+output.name+'\n')
    print(f'공유 파일: {output}\nSHA256: {digest}')
    return output


def verify(root):
    root=dataset(root);manifest=read_json(root/'manifest.json')
    if not manifest:raise ValueError('manifest.json이 없습니다.')
    for row in manifest['files']:
        path=(root/row['path']).resolve()
        if not path.is_relative_to(root) or not path.is_file() or path.stat().st_size!=row['size'] or sha256(path)!=row['sha256']:
            raise ValueError('불일치: '+row['path'])
    print(f"검증 완료: {len(manifest['files'])} files")


def main():
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='command',required=True)
    for name in ['init','app','console','finalize','view','pack','verify','imu']:
        command=sub.add_parser(name);command.add_argument('dataset',type=Path)
        if name=='init':command.add_argument('--checklist',type=Path)
        if name=='app':command.add_argument('--demo',action='store_true')
        if name in ['view','finalize']:command.add_argument('--run')
        if name=='pack':command.add_argument('--output',type=Path,required=True)
        if name=='imu':
            command.add_argument('--phase',choices=['pre','post'],required=True);command.add_argument('--seconds',type=float,default=60)
            command.add_argument('--host',default='pi@raspberrypi.local')
            command.add_argument('--calibration',type=Path,default=ROOT/'outputs/handheld_mapping/calibration/imu_calibration.json')
    args=parser.parse_args()
    try:
        root=args.dataset
        if args.command=='init':print(init(root,args.checklist))
        elif args.command=='app':launch_app(root,args.demo)
        elif args.command=='console':console(root)
        elif args.command=='view':view(root,args.run)
        elif args.command=='verify':verify(root)
        elif args.command=='pack':pack(root,args.output)
        elif args.command=='imu':stationary(root,args.phase,args.seconds,args.host,args.calibration)
        elif args.command=='finalize':
            root=dataset(root);run=root/'sessions'/safe_id(args.run) if args.run else newest_run(root)
            with lock(root,'.capture.lock',False):finalize_run(root,run)
    except (ValueError,OSError) as error:parser.exit(1,str(error)+'\n')


if __name__=='__main__':main()
