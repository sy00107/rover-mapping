"""Display-only distances and heading changes from final camera poses, never wheel odometry."""
import csv
import math
from pathlib import Path
import re

import numpy as np

from experiments.handheld_mapping.reconstruct import pose_matrix


def load_pose_groups(geometry_path: Path) -> list[list[dict]]:
    from experiments.handheld_mapping.viewer import FOLDERS, map_frame
    path = Path(geometry_path).resolve()
    backend = next((key for key,value in FOLDERS.items() if value == path.parent.name),None)
    match = re.fullmatch(r'map_(\d+)_(?:mesh|cloud)\.ply',path.name)
    if backend is None or match is None:
        return []
    identity = int(match[1])
    source = path.parent.parent/backend/f'map_{identity}_keyframes.csv'
    if not source.is_file():
        return []
    matrix, aligned = map_frame(path)
    groups = {}
    with source.open(newline='') as stream:
        reader = csv.DictReader(stream)
        required = {'map_id','keyframe_id','source_stamp','x','y','z','qx','qy','qz','qw'}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError('거리·각도 표시에는 최종 위치와 quaternion이 필요합니다')
        for row in reader:
            stamp = float(row['source_stamp'])
            if int(row['map_id']) != identity or int(row['keyframe_id']) < 0 or not math.isfinite(stamp):
                raise ValueError('거리·각도 경로의 map/시각이 올바르지 않습니다')
            pose = matrix @ pose_matrix(row)
            groups.setdefault(row.get('source_session',''),[]).append(
                {'stamp':stamp,'position':pose[:3,3],'forward':pose[:3,2],
                 'up':np.array([0.,0.,1.]) if aligned else np.array([0.,-1.,0.])})
    result = []
    for poses in groups.values():
        poses.sort(key=lambda row:row['stamp'])
        chunk = []
        for row in poses:
            if chunk:
                dt = row['stamp']-chunk[-1]['stamp']
                if dt <= 0:
                    raise ValueError('동일 촬영의 경로 시각은 증가해야 합니다')
                # A long unobserved interval is not measured as uninterrupted travel.
                if dt > 3.:
                    result.append(chunk);chunk=[]
            chunk.append(row)
        if chunk:
            result.append(chunk)
    return result


def _knots(points: np.ndarray, tolerance: float, scalar_time=None) -> list[int]:
    """Iterative RDP: position-to-segment distance or scalar interpolation error."""
    if len(points) < 2:
        return list(range(len(points)))
    keep = {0,len(points)-1};stack=[(0,len(points)-1)]
    while stack:
        start,end=stack.pop()
        if end-start <= 1:
            continue
        if scalar_time is not None:
            t=(scalar_time[start+1:end]-scalar_time[start])/(scalar_time[end]-scalar_time[start])
            error=np.abs(points[start+1:end]-(points[start]+t*(points[end]-points[start])))
        else:
            delta=points[end]-points[start];length=float(delta@delta)
            t=np.clip((points[start+1:end]-points[start])@delta/length,0,1) if length else np.zeros(end-start-1)
            error=np.linalg.norm(points[start+1:end]-(points[start]+t[:,None]*delta),axis=1)
        index=int(np.argmax(error))+start+1
        if error[index-start-1] > tolerance:
            keep.add(index);stack.extend([(start,index),(index,end)])
    return sorted(keep)


def measurements(poses: list[dict], min_distance_m: float = .30,
                 min_turn_deg: float = 20., position_tolerance_m: float = .05) -> list[dict]:
    """Straight chord lengths and net yaw changes; pitch/roll do not count as turns."""
    if len(poses) < 2:
        return []
    points=np.asarray([row['position'] for row in poses],float)
    stamps=np.asarray([row['stamp'] for row in poses],float)
    if not np.isfinite(points).all() or not np.isfinite(stamps).all() or np.any(np.diff(stamps)<=0):
        raise ValueError('invalid measured trajectory')
    up=np.asarray(poses[0]['up'],float)
    headings=np.asarray([row['forward'] for row in poses],float)
    headings=headings-(headings@up)[:,None]*up
    norms=np.linalg.norm(headings,axis=1)
    # Near-vertical views have undefined planar heading: retain distances, omit angles.
    turns=[]
    if np.all(norms > .15):
        headings/=norms[:,None]
        ref=headings[0];left=np.cross(up,ref)
        angles=np.rad2deg(np.unwrap(np.arctan2(headings@left,headings@ref)))
        knots=_knots(angles,3.,scalar_time=stamps)
        active=None
        for start,end in zip(knots,knots[1:]):
            change=angles[end]-angles[start]
            if abs(change)<6.:
                if active is not None:
                    turns.append(active);active=None
                continue
            if active is not None and np.sign(change)==np.sign(angles[active[1]]-angles[active[0]]):
                active=(active[0],end)
            else:
                if active is not None:turns.append(active)
                active=(start,end)
        if active is not None:turns.append(active)
        turns=[(a,b) for a,b in turns if abs(angles[b]-angles[a])>=min_turn_deg]
    result=[];intervals=[];previous=0
    for start,end in turns:
        intervals.append((previous,start));previous=end
        delta=float(angles[end]-angles[start])
        result.append({'kind':'turn','value':delta,'text':f'{"L" if delta>0 else "R"} {abs(delta):.0f}°',
                       'anchor':points[(start+end)//2].copy(),'start':start,'end':end})
    intervals.append((previous,len(poses)-1))
    for start,end in intervals:
        indices=[start+i for i in _knots(points[start:end+1],position_tolerance_m)]
        for a,b in zip(indices,indices[1:]):
            distance=float(np.linalg.norm(points[b]-points[a]))
            if distance < min_distance_m:
                continue
            result.append({'kind':'distance','value':distance,'text':f'{distance:.2f} m',
                           'anchor':(points[a]+points[b])/2,'start':a,'end':b})
    return result


# Simple vector glyphs work with the existing Open3D legacy renderer (no GUI/GL replacement).
_SEGMENTS={
 'a':[(0,1),(0.6,1)],'b':[(.6,1),(.6,.5)],'c':[(.6,.5),(.6,0)],
 'd':[(0,0),(.6,0)],'e':[(0,0),(0,.5)],'f':[(0,.5),(0,1)],'g':[(0,.5),(.6,.5)]}
_DIGITS={'0':'abcdef','1':'bc','2':'abged','3':'abgcd','4':'fgbc','5':'afgcd',
         '6':'afgecd','7':'abc','8':'abcdefg','9':'abfgcd'}
_GLYPHS={'L':[[(0,1),(0,0),(.6,0)]],
         'R':[[(0,0),(0,1),(.5,1),(.6,.85),(.6,.65),(.5,.5),(0,.5)],[(.3,.5),(.6,0)]],
         'm':[[(0,0),(0,.65),(.3,.65),(.3,0)],[(.3,.65),(.6,.65),(.6,0)]],
         '.':[[(.25,0),(.3,.04),(.35,0),(.25,0)]],
         '°':[[(.18,.7),(0,.8),(0,.95),(.18,1.05),(.36,.95),(.36,.8),(.18,.7)]]}


def text_strokes(text: str):
    points=[];lines=[];x=0.
    for char in text:
        strokes=[_SEGMENTS[key] for key in _DIGITS[char]] if char in _DIGITS else _GLYPHS.get(char,[])
        for stroke in strokes:
            first=len(points);points.extend([(x+a,b) for a,b in stroke])
            lines.extend((i,i+1) for i in range(first,len(points)-1))
        x+=.45 if char in (' ','.','°') else .85
    return np.asarray(points,float).reshape(-1,2),np.asarray(lines,int).reshape(-1,2),x


class BillboardLabels:
    """Camera-facing, fixed-pixel-size 3D stroke labels with screen overlap suppression."""
    def __init__(self, labels: list[dict]):
        import open3d as o3d
        self.geometry=o3d.geometry.LineSet()
        self.labels=sorted(labels,key=lambda row:0 if row['kind']=='turn' else 1)
        self.glyphs=[text_strokes(row['text']) for row in self.labels]
        self.last=None
        self.visible_count=0

    def update(self, parameters) -> bool:
        import open3d as o3d
        k=np.asarray(parameters.intrinsic.intrinsic_matrix)
        extrinsic=np.array(parameters.extrinsic,copy=True)
        # Legacy ViewControl can export a non-orthogonal up vector after set_front/set_up.
        # Its OpenGL look-at camera orthogonalizes it; match that camera for projection too.
        rotation=extrinsic[:3,:3]
        if not np.allclose(rotation@rotation.T,np.eye(3),atol=1e-6):
            eye=np.linalg.solve(rotation,-extrinsic[:3,3])
            forward=rotation[2]/np.linalg.norm(rotation[2])
            right=np.cross(rotation[1],forward);right/=np.linalg.norm(right)
            rotation=np.array([right,np.cross(forward,right),forward])
            extrinsic[:3,:3]=rotation;extrinsic[:3,3]=-rotation@eye
        state=np.r_[k.ravel(),extrinsic.ravel(),parameters.intrinsic.width,parameters.intrinsic.height]
        if self.last is not None and np.allclose(state,self.last,rtol=0,atol=1e-7):return False
        self.last=state.copy();points=[];lines=[];colors=[];boxes=[]
        height=16.;width=parameters.intrinsic.width;screen_height=parameters.intrinsic.height
        inverse=np.linalg.inv(extrinsic)
        # Draw as a foreground overlay, keeping text readable over bright/occluding surfaces.
        # Anchor projection still comes from the exact saved trajectory position.
        display_depth=.05
        def unproject(pixels,depth):
            pixels=np.asarray(pixels,float)
            rays=np.column_stack(((pixels[:,0]-k[0,2])/k[0,0],(pixels[:,1]-k[1,2])/k[1,1],np.ones(len(pixels))))
            return (rays*depth)@inverse[:3,:3].T+inverse[:3,3]
        for row,(glyph,segments,advance) in zip(self.labels,self.glyphs):
            if len(boxes)>=30:break
            camera=extrinsic[:3,:3]@row['anchor']+extrinsic[:3,3]
            if camera[2]<=.01:continue
            u=k[0,0]*camera[0]/camera[2]+k[0,2];v=k[1,1]*camera[1]/camera[2]+k[1,2]
            if not (0<=u<width and 0<=v<screen_height):continue
            w=advance*height
            chosen=None
            for dx,dy in [(-w/2,-24),(-w/2,40),(18,8),(-w-18,8),(-w/2,-55)]:
                x,y=u+dx,v+dy;box=(x-5,y-height-5,x+w+5,y+5)
                if box[0]<5 or box[2]>width-5 or box[1]<5 or box[3]>screen_height-5:continue
                if any(not(box[2]<b[0] or box[0]>b[2] or box[3]<b[1] or box[1]>b[3]) for b in boxes):continue
                chosen=(x,y,box);break
            if chosen is None:continue
            x,y,box=chosen;boxes.append(box)
            pixels=glyph*np.array([height,-height])+[x,y]
            world=unproject(pixels,display_depth)
            offset=len(points);points.extend(world);lines.extend(segments+offset)
            color=[1.,.87,.40] if row['kind']=='turn' else [.80,.94,1.]
            colors.extend([color]*len(segments))
            target_pixel=[np.clip(u,box[0]+5,box[2]-5),np.clip(v,box[1]+5,box[3]-5)]
            offset=len(points);points.extend(unproject([[u,v],target_pixel],display_depth))
            lines.append([offset,offset+1]);colors.append([.35,.55,.65])
        self.visible_count=len(boxes)
        self.geometry.points=o3d.utility.Vector3dVector(np.asarray(points).reshape(-1,3))
        self.geometry.lines=o3d.utility.Vector2iVector(np.asarray(lines,dtype=int).reshape(-1,2))
        self.geometry.colors=o3d.utility.Vector3dVector(np.asarray(colors).reshape(-1,3))
        return True
