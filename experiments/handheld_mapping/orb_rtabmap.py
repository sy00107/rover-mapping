"""Import final ORB camera keyframes and measured depth into a separate RTAB database."""
import argparse
import json
import os
from pathlib import Path
import sqlite3
import subprocess

import numpy as np
from scipy.spatial.transform import Rotation

from .compare_rtabmap import orb_poses,tum_poses
from .gravity import fingerprint
from .reconstruct import nearest
from .rtabmap_rgbd import ROOT,validate_calibration


def prepare(run,out):
    info=validate_calibration(json.loads((run/'capture_calibration.json').read_text()))
    stamps,poses=orb_poses(run)
    lefts=sorted(int(p.stem) for p in (run/'raw/left').glob('*.png'))
    depths=sorted(int(p.stem) for p in (run/'raw/depth').glob('*.png'))
    quats=Rotation.from_matrix(poses[:,:3,:3]).as_quat();lines=[]
    for stamp,pose,quat in zip(stamps,poses,quats):
        left=nearest(lefts,round(stamp*1e9));depth=nearest(depths,round(stamp*1e9))
        if left is None or depth is None:raise ValueError('Every ORB keyframe must have original IR and depth within 2ms')
        values='\t'.join(format(v,'.17g') for v in [*pose[:3,3],*quat])
        lines.append(f'{left}\t{run/"raw/left"/f"{left}.png"}\t{run/"raw/depth"/f"{depth}.png"}\t{values}')
    out.mkdir(parents=True,exist_ok=False)
    (out/'orb_rgbd_frames.tsv').write_text('\n'.join(lines)+'\n')
    files=[run/'capture_calibration.json',*sorted((run/'orb_offline').glob('map_*_keyframes.csv'))]
    (out/'input_summary.json').write_text(json.dumps(dict(source=str(run),input='ORB final camera keyframe poses + original left IR and measured depth',
        keyframes=len(lines),display_title='ORB pose + RTAB RGB-D',extra_loop_correction=False,source_sha256={str(p):fingerprint(p) for p in files},
        runner_source_sha256=fingerprint(ROOT/'experiments/handheld_mapping/rtabmap/orb_fusion.cpp'),
        calibration_sha256=fingerprint(run/'capture_calibration.json'),raw_preserved=True),indent=2)+'\n')
    (out/'calibration.json').write_bytes((run/'capture_calibration.json').read_bytes())
    return info,len(lines)


def verify(out):
    it,ip=tum_poses(out/'orb_imported.tum');rt,rp=tum_poses(out/'rtab_optimized.tum')
    ids=np.searchsorted(it,rt);ids=np.clip(ids,0,len(it)-1)
    before=np.maximum(ids-1,0);ids=np.where(abs(it[before]-rt)<abs(it[ids]-rt),before,ids)
    if np.max(abs(it[ids]-rt))>.001:raise ValueError('RTAB graph timestamps do not match imported ORB poses')
    position=np.linalg.norm(ip[ids,:3,3]-rp[:,:3,3],axis=1)
    angles=np.degrees(Rotation.from_matrix(ip[ids,:3,:3].transpose(0,2,1)@rp[:,:3,:3]).magnitude())
    c=sqlite3.connect('file:'+str(out/'rtabmap.db')+'?mode=ro',uri=True)
    counts=dict(c.execute('select type,count(*) from Link group by type'));c.close()
    if any(kind!=0 for kind in counts):raise ValueError('Unexpected non-neighbor graph constraints')
    if position.max()>1e-5 or angles.max()>.001:raise ValueError('RTAB altered imported ORB poses')
    report=dict(imported_keyframes=len(it),retained_graph_nodes=len(rt),max_position_change_m=float(position.max()),
                max_rotation_change_deg=float(angles.max()),link_counts=counts,
                extra_loop_correction=False,pose_source='ORB final camera keyframes',
                interpretation='RTAB stores ORB poses and IR/depth; surface fused with existing TSDF pipeline. Not jointly optimized SLAM.')
    (out/'fusion_verification.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('run',type=Path);ap.add_argument('output',type=Path)
    args=ap.parse_args();run=args.run.resolve();out=args.output.resolve();info,count=prepare(run,out);k=info['k']
    command=[str(ROOT/'outputs/handheld_mapping/rtabmap_build/rover_orb_rtab_fusion'),str(out/'orb_rgbd_frames.tsv'),str(out),
             *map(str,[k[0],k[4],k[2],k[5],info['depth_scale_m'],count])]
    (out/'command.json').write_text(json.dumps(command,indent=2)+'\n')
    env=os.environ.copy();env.update(OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='1',
        LD_LIBRARY_PATH='/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:'+env.get('LD_LIBRARY_PATH',''))
    with (out/'rtab_runner.log').open('w') as log:subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    print(json.dumps(verify(out),indent=2),flush=True)

if __name__=='__main__':main()
