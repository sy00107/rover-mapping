"""Replay recorded left IR + measured depth through RTAB-Map, preserving source files."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

import numpy as np

from .reconstruct import nearest

ROOT=Path(__file__).resolve().parents[2]


def validate_calibration(calibration):
    depth=calibration['depth'];left=calibration['stereo']['left']
    if calibration['stereo'].get('input_geometry')!='rectified_confirmed':
        raise ValueError('Rectified left IR required')
    if not np.allclose(depth['T_left_depth'],np.eye(4),atol=1e-7):
        raise ValueError('Depth must be registered to left IR; nonidentity TF requires reprojection')
    if not np.allclose(depth['k'],left['k'],atol=1e-6) or any(depth[k]!=left[k] for k in ('width','height')):
        raise ValueError('Depth and left IR intrinsics/dimensions differ; registration required')
    if (depth['width'],depth['height'])!=(640,480):
        raise ValueError('Runner currently supports 640x480')
    if any(abs(float(v))>1e-8 for v in left.get('d',[])+depth.get('d',[])):
        raise ValueError('Undistorted input required')
    scale=float(depth['depth_scale_m'])
    if not np.isfinite(scale) or not 0<scale<.1:
        raise ValueError('Invalid depth scale')
    return depth


def prepare(run,out):
    calibration_path=run/'capture_calibration.json'
    calibration=json.loads(calibration_path.read_text());info=validate_calibration(calibration)
    depths=sorted(int(p.stem) for p in (run/'raw/depth').glob('*.png'))
    lefts=sorted((run/'raw/left').glob('*.png'),key=lambda p:int(p.stem))
    pairs=[];skipped=[];gaps=[]
    for left in lefts:
        stamp=int(left.stem); depth=nearest(depths,stamp)
        if depth is None:skipped.append(stamp);continue
        pairs.append(f'{stamp}\t{left}\t{run/"raw/depth"/f"{depth}.png"}')
        gaps.append(abs(depth-stamp))
    if not pairs:raise ValueError('No depth within 2ms of left IR')
    out.mkdir(parents=True,exist_ok=False)
    (out/'rgbd_pairs.tsv').write_text('\n'.join(pairs)+'\n')
    (out/'calibration.json').write_bytes(calibration_path.read_bytes())
    manifest=dict(source=str(run),input='left IR + measured depth (RGB-D API, grayscale texture)',
                  pairs=len(pairs),unpaired_left=len(skipped),unpaired_stamps_ns=skipped,
                  max_depth_timestamp_gap_ns=max(gaps),depth_scale_m=info['depth_scale_m'],
                  calibration_sha256=hashlib.sha256(calibration_path.read_bytes()).hexdigest(),
                  runner_source_sha256=hashlib.sha256((ROOT/'experiments/handheld_mapping/rtabmap/main.cpp').read_bytes()).hexdigest(),
                  depth_min_m=.25,depth_max_m=2.,raw_preserved=True,covariance='odometry registration covariance',
                  reset_policy='no automatic reset; failures logged')
    (out/'input_summary.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return info


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('run',type=Path);ap.add_argument('output',type=Path)
    ap.add_argument('--max-frames',type=int,default=0)
    args=ap.parse_args()
    if args.max_frames<0:ap.error('max-frames must be nonnegative')
    run=args.run.resolve();out=args.output.resolve();info=prepare(run,out)
    k=info['k']
    env=os.environ.copy();env.update(OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='1',
        LD_LIBRARY_PATH='/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:'+env.get('LD_LIBRARY_PATH',''))
    command=[str(ROOT/'outputs/handheld_mapping/rtabmap_build/rover_rtab_compare'),str(out/'rgbd_pairs.tsv'),str(out),
             *map(str,[k[0],k[4],k[2],k[5],0,args.max_frames]),'rgbd',str(info['depth_scale_m'])]
    (out/'command.json').write_text(json.dumps(command,indent=2)+'\n')
    print(f'Prepared RGB-D replay: {out}',flush=True)
    with (out/'rtab_runner.log').open('w') as log:subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    print((out/'runner_summary.json').read_text(),flush=True)

if __name__=='__main__':main()
