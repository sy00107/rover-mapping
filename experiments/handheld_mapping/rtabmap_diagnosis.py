import sqlite3,zlib,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from experiments.handheld_mapping.compare_rtabmap import orb_poses,tum_poses
from experiments.handheld_mapping.compare_rtabmap_maps import interpolate
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
out=Path('outputs/handheld_mapping/comparisons/orb_rtabmap_20261016_174639')
c=sqlite3.connect('file:'+str(out/'rtabmap.db')+'?mode=ro',uri=True)
rows=c.execute('select id,stamp,pose from Node order by stamp').fetchall()
raw=np.array([np.vstack([np.frombuffer(b,dtype=np.float32).reshape(3,4),[0,0,0,1]]) for _,_,b in rows]);ts=np.array([r[1] for r in rows]);ids={r[0]:i for i,r in enumerate(rows)}
ot,op=orb_poses(Path('outputs/handheld_mapping/sessions/20261016_174639_b1d8ed'));rt,rp=tum_poses(out/'rtab_optimized.tum')
np.savetxt(out/'rtab_odometry_raw.tum',np.column_stack([ts,raw[:,:3,3],Rotation.from_matrix(raw[:,:3,:3]).as_quat()]),fmt='%.12f')
stats=[]
for id,t,b in c.execute('select id,stamp,data from Statistics order by stamp'):
 d={k:float(v) for part in zlib.decompress(b[:-12]).decode().strip('\x00').split(';') if ':' in part for k,v in [part.rsplit(':',1)]};stats.append((id,t,d))
loops=[(id,t-ts[0],d.get('Loop/Id/',0),d.get('Loop/Visual_inliers/',0)) for id,t,d in stats if d.get('Loop/Id/',0)>0]
print('first loops',loops[:8]);print('loop count',len(loops))
result={'loop_events':len(loops),'first_loop_s':loops[0][1] if loops else None,'link_counts':dict(c.execute('select type,count(*) from Link group by type').fetchall()),'pose_comparisons':{}}
for name,t,p in [('raw',ts,raw),('optimized',rt,rp)]:
 diff=[];angles=[];times=[]
 for stamp,pose in zip(ot,op):
  other=interpolate(stamp,t,p)
  if other is None:continue
  diff.append(np.linalg.norm(pose[:3,3]-other[:3,3]));angles.append(Rotation.from_matrix(pose[:3,:3].T@other[:3,:3]).magnitude()*180/np.pi);times.append(stamp-ts[0])
 result['pose_comparisons'][name]={'median_position_m':float(np.median(diff)),'p95_position_m':float(np.percentile(diff,95)),'median_rotation_deg':float(np.median(angles)),'last_position_m':float(diff[-1]),'last_rotation_deg':float(angles[-1])}
 if name=='raw': rawtimes=np.array(times);rawdiff=np.array(diff);rawang=np.array(angles)
 else: opttimes=np.array(times);optdiff=np.array(diff);optang=np.array(angles)
# Optimizer displacement at matching graph timestamps
indices=[int(np.argmin(abs(ts-t))) for t in rt]
result['optimizer_displacement_m']={'median':float(np.median(np.linalg.norm(raw[indices,:3,3]-rp[:,:3,3],axis=1))),'max':float(np.max(np.linalg.norm(raw[indices,:3,3]-rp[:,:3,3],axis=1)))}
fig,ax=plt.subplots(2,2,figsize=(13,9))
for label,t,p in [('ORB',ot,op),('RTAB raw odometry',ts,raw),('RTAB optimized',rt,rp)]:
 ax[0,0].plot(p[:,0,3],p[:,2,3],label=label);ax[0,1].plot(t-ts[0],p[:,1,3],label=label)
ax[0,0].set(xlabel='X (m)',ylabel='Z (m)',aspect='equal');ax[0,1].set(xlabel='Time (s)',ylabel='Optical Y (m)')
for label,t,d,ang in [('Raw vs ORB',rawtimes,rawdiff,rawang),('Optimized vs ORB',opttimes,optdiff,optang)]:
 ax[1,0].plot(t,d,label=label);ax[1,1].plot(t,ang,label=label)
ax[1,0].set(xlabel='Time (s)',ylabel='Position disagreement (m)');ax[1,1].set(xlabel='Time (s)',ylabel='Rotation disagreement (deg)')
for a in ax.flat:a.grid(alpha=.3);a.legend()
if loops:
 for a in [ax[0,1],ax[1,0],ax[1,1]]:a.axvline(loops[0][1],color='grey',ls=':',label='First loop')
fig.suptitle('RTAB raw odometry vs graph optimization | ORB is a reference, not ground truth');fig.tight_layout();fig.savefig(out/'diagnosis.png',dpi=170)
(out/'diagnosis.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
for cutoff in [30,60,120,180,240,300,360]:
 i=np.argmin(abs(rawtimes-cutoff));print('time',rawtimes[i],'raw pos',rawdiff[i],'rot',rawang[i])
print('covariance first stats',[(t-ts[0],d.get('Memory/Odometry_variance_ang/'),d.get('Memory/Odometry_variance_lin/')) for _,t,d in stats[::1000]])
