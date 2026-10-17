"""Quantify RTAB RGB-D replay against preserved ORB reference and stereo baseline."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .compare_rtabmap import orb_poses,tum_poses
from .compare_rtabmap_maps import interpolate


def evaluate(run,output,baseline):
    ot,op=orb_poses(run)
    series=[];result={}
    for name,path in [('stereo baseline',baseline/'rtab_odometry_raw.tum'),('measured depth RGB-D',output/'rtab_odometry_raw.tum')]:
        times,poses=tum_poses(path)
        series.append((name,times,poses))
    if (output/'rtab_optimized.tum').exists():
        times,poses=tum_poses(output/'rtab_optimized.tum')
        series.append(('RGB-D optimized graph',times,poses))
    end=min(t[-1] for _,t,_ in series)
    for name,times,poses in series:
        values=[]
        for stamp,pose in zip(ot,op):
            if stamp>end:continue
            other=interpolate(stamp,times,poses)
            if other is None:continue
            values.append([stamp-ot[0],np.linalg.norm(other[:3,3]-pose[:3,3]),
                           np.degrees(Rotation.from_matrix(pose[:3,:3].T@other[:3,:3]).magnitude())])
        if not values:raise ValueError('No common comparison timestamps')
        values=np.array(values)
        result[name]=dict(samples=len(values),last_time_s=float(values[-1,0]),median_position_m=float(np.median(values[:,1])),
                          p95_position_m=float(np.percentile(values[:,1],95)),median_rotation_deg=float(np.median(values[:,2])),
                          last_position_m=float(values[-1,1]),last_rotation_deg=float(values[-1,2]))
    result['reference']='ORB offline; disagreement is not ground truth error'
    result['comparison_end_s']=float(end-ot[0])
    (output/'rgbd_evaluation.json').write_text(json.dumps(result,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(12,5))
    for name,times,poses in [('ORB reference',ot,op),*series]:
        take=times<=end;positions=poses[take,:3,3]
        axes[0].plot(positions[:,0],positions[:,2],label=name)
        axes[1].plot(times[take]-ot[0],positions[:,1],label=name)
    axes[0].set(xlabel='X (m)',ylabel='Z (m)',aspect='equal')
    axes[1].set(xlabel='Time (s)',ylabel='Camera optical Y (m)')
    for ax in axes:ax.grid(alpha=.3);ax.legend()
    fig.suptitle('Same recorded input: stereo triangulation vs measured depth');fig.tight_layout();fig.savefig(output/'rgbd_evaluation.png',dpi=170);plt.close(fig)
    return result


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('run',type=Path);ap.add_argument('output',type=Path);ap.add_argument('baseline',type=Path)
    a=ap.parse_args();print(json.dumps(evaluate(a.run,a.output,a.baseline),indent=2))

if __name__=='__main__':main()
