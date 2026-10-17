"""Prepare identical stereo inputs and plot ORB/RTAB-Map camera trajectories.

RTAB input must be TUM camera-to-world poses (optical camera frame), metres.
No scale fitting or ground-truth accuracy claims are made.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from .reconstruct import pose_matrix


def orb_poses(run):
    files = sorted((run / 'orb_offline').glob('map_*_keyframes.csv'))
    if len(files) != 1:
        raise ValueError('Select a single-map ORB session for this comparison')
    rows = sorted(csv.DictReader(files[0].open()), key=lambda r: float(r['source_stamp']))
    return np.array([float(r['source_stamp']) for r in rows]), np.array([pose_matrix(r) for r in rows])


def tum_poses(path):
    rows = np.loadtxt(path, comments='#', ndmin=2)
    if rows.shape[1] != 8 or len(rows) < 2 or not np.isfinite(rows).all():
        raise ValueError('TUM requires timestamp x y z qx qy qz qw, at least two finite rows')
    if np.any(np.diff(rows[:, 0]) <= 0):
        raise ValueError('TUM timestamps must increase')
    return rows[:, 0], np.array([pose_matrix(dict(zip(('x','y','z','qx','qy','qz','qw'), r[1:]))) for r in rows])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rtab-tum', type=Path)
    args = parser.parse_args()
    run, out = args.run.resolve(), args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    stamps, poses = orb_poses(run)
    # Rebuild paths from preserved filenames; original pairs can contain old workspace paths.
    pairs = []
    for line in (run/'orb_offline/pairs.tsv').read_text().splitlines():
        stamp, left, right = line.split('\t')
        paths = [run/'raw'/side/Path(value).name for side, value in [('left', left), ('right', right)]]
        if not all(p.is_file() for p in paths):
            raise FileNotFoundError(paths)
        pairs.append('\t'.join([stamp, *map(str, paths)]))
    (out/'stereo_pairs.tsv').write_text('\n'.join(pairs)+'\n')
    (out/'calibration.json').write_bytes((run/'capture_calibration.json').read_bytes())
    from scipy.spatial.transform import Rotation
    tum = np.column_stack([stamps, poses[:, :3, 3], Rotation.from_matrix(poses[:, :3, :3]).as_quat()])
    np.savetxt(out/'orb_keyframes.tum', tum, fmt='%.12f')
    report = {'source':str(run), 'stereo_pairs':len(pairs), 'orb_keyframes':len(stamps),
              'status':'waiting_for_rtabmap', 'ground_truth_available':False,
              'alignment':'first camera pose only; rigid SE3, no scale fitting',
              'rtab_input_contract':'TUM camera-to-world, optical frame, metres, original epoch timestamps'}
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1,3,figsize=(15,5))
    paths = [('ORB-SLAM3', (np.linalg.inv(poses[0]) @ poses)[:,:3,3])]
    if args.rtab_tum:
        rt, rp = tum_poses(args.rtab_tum)
        # Anchor both trajectories at a common observed time, without interpolating orientation.
        ri = np.argmin(abs(rt-stamps[0]))
        if abs(rt[ri]-stamps[0]) > .1:
            raise ValueError('RTAB trajectory must cover the first ORB keyframe within 100 ms')
        paths.append(('RTAB-Map', (np.linalg.inv(rp[ri]) @ rp)[:,:3,3]))
        report.update(status='complete', rtab_poses=len(rt), anchor_time_difference_s=float(abs(rt[ri]-stamps[0])))
    for ax,(i,j) in zip(axes,[(0,2),(0,1),(2,1)]):
        for label, xyz in paths:
            ax.plot(xyz[:,i],xyz[:,j],label=label,linewidth=1)
        ax.set(xlabel='XYZ'[i]+' (m)',ylabel='XYZ'[j]+' (m)')
        ax.axis('equal'); ax.grid(alpha=.3); ax.legend()
    fig.suptitle('ORB-SLAM3 / RTAB-Map' if args.rtab_tum else 'ORB-SLAM3 baseline — RTAB-Map pending')
    fig.tight_layout(); fig.savefig(out/'trajectories.png',dpi=170); plt.close(fig)
    (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
