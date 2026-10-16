"""검증된 개별 IMU 표본만 ORB stereo-inertial 입력으로 허용한다.

현재 50Hz control telemetry의 평균값과 Pi 수신 시각을 센서 취득 표본이나
카메라 동기 시각으로 둔갑시키지 않는다. 별도의 실측 보정 profile이 필요하다.
"""
import csv
import json
import math
from pathlib import Path

from experiments.handheld_mapping.gravity import fingerprint
from experiments.handheld_mapping.reconstruct import transform


def prepare_input(run: Path, folder: Path, profile_path: Path) -> dict:
    run,folder = Path(run).resolve(),Path(folder).resolve()
    profile = json.loads(Path(profile_path).read_text())
    if profile.get('schema') != 1 or profile.get('run_id') != run.name:
        raise ValueError('inertial profile schema/RUN mismatch')
    for key in ('clock_verified','extrinsic_verified','sample_timestamps_verified','noise_verified'):
        if profile.get(key) is not True:
            raise ValueError(f'IMU 융합에는 실측 검증이 필요합니다: {key}')
    for key,value in (('sample_format','per_sample_SI_in_camera_clock'),
                      ('frame','rover_body_aligned_imu'),('acc_unit','m/s^2'),('gyro_unit','rad/s')):
        if profile.get(key) != value:
            raise ValueError(f'inertial {key} must be {value}; control packet averages are not raw samples')
    sample_name = profile.get('samples')
    if not isinstance(sample_name,str):
        raise ValueError('inertial samples path required')
    samples = (run/sample_name).resolve()
    if not samples.is_relative_to(run):
        raise ValueError('inertial samples must reside in RUN')
    for source in (samples,run/'capture_calibration.json',run/'imu_calibration.json',run/'robot.urdf'):
        if profile.get('source_sha256',{}).get(str(source.relative_to(run))) != fingerprint(source):
            raise ValueError(f'inertial calibration source mismatch: {source}')
    matrix = transform(profile['T_imu_left'])
    frequency = profile.get('frequency_hz')
    if type(frequency) not in (int,float) or not math.isfinite(frequency) or not 10 <= frequency <= 2000:
        raise ValueError('measured IMU frequency in 10..2000 Hz required')
    noise = profile.get('noise',{})
    keys = ('NoiseGyro','NoiseAcc','GyroWalk','AccWalk')
    for key in keys:
        value = noise.get(key)
        if type(value) not in (int,float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'measured IMU noise density/random walk required: {key}')
    first,last,count = None,None,0
    with samples.open(newline='') as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != ['stamp_ns','ax','ay','az','gx','gy','gz']:
            raise ValueError('inertial samples CSV header/units mismatch')
        for row in reader:
            stamp = int(row['stamp_ns'])
            values = [float(row[key]) for key in ('ax','ay','az','gx','gy','gz')]
            if not 0 <= stamp < 2**63 or not all(math.isfinite(v) for v in values):
                raise ValueError('invalid IMU sample values')
            if last is not None and (stamp <= last or stamp-last > 3e9/frequency):
                raise ValueError('nonmonotonic IMU clock or missing IMU samples')
            if first is None:first=stamp
            last=stamp; count+=1
    with (folder/'pairs.tsv').open() as stream:
        camera_stamps = (int(line.split('\t',1)[0]) for line in stream)
        start,end = next(camera_stamps),None
        for end in camera_stamps:pass
        end = start if end is None else end
    if count < 2 or first > start or last < end:
        raise ValueError('IMU samples must cover the entire stereo recording')
    settings = folder/'settings.yaml'
    text = settings.read_text()
    text = '\n'.join(line for line in text.splitlines()
                     if not line.startswith('Tracking.StereoRelocalizationWaitSeconds:'))+'\n'
    numbers = ', '.join(format(float(v),'.12g') for v in matrix.flat)
    text += f'IMU.T_b_c1: !!opencv-matrix\n  rows: 4\n  cols: 4\n  dt: f\n  data: [ {numbers} ]\n'
    text += f'IMU.Frequency: {frequency}\nIMU.InsertKFsWhenLost: 0\n'
    text += ''.join(f'IMU.{key}: {noise[key]}\n' for key in keys)
    settings.write_text(text)
    # 입력 profile 원문도 결과 옆에 보존한다. 측정값을 추정해서 채우지 않는다.
    (folder/'inertial_profile.json').write_bytes(Path(profile_path).read_bytes())
    return {'samples':str(samples),'sample_count':count,'profile_sha256':fingerprint(Path(profile_path)),
            'clock_verified':True,'extrinsic_verified':True,'sample_timestamps_verified':True,
            'sample_format':profile['sample_format'],'frame':profile['frame']}
