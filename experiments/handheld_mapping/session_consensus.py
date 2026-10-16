"""Visibility-aware cross-session evidence, preserving unobserved/occluded areas."""
from pathlib import Path
import math

import numpy as np
from scipy.ndimage import maximum_filter

from experiments.handheld_mapping.depth_filter import filtered_depths
from experiments.handheld_mapping.reconstruct import pose_matrix, transform
from experiments.handheld_mapping.terrain_replay import project_depth


class SessionConsensus:
    """One vote per recording; same-recording frames never verify themselves.

    Surface support needs two frames. Clear-space evidence needs three frames
    whose measured surfaces lie at least 10cm farther along their camera rays.
    Occlusion does not constitute a conflict. Unobserved regions are retained.
    """

    def __init__(self, low, high, session_count, resolution=.05, max_cells=8_000_000):
        low,high=np.asarray(low,float),np.asarray(high,float)
        if (low.shape!=(3,) or high.shape!=(3,) or not np.isfinite([low,high]).all()
                or np.any(high<=low) or type(session_count) is not int or not 2<=session_count<=15
                or not math.isfinite(resolution) or resolution<=0 or max_cells<1):
            raise ValueError('invalid cross-session evidence bounds/count')
        resolution=max(resolution,math.ceil((np.prod(high-low)/max_cells)**(1/3)*100)/100)
        shape=np.ceil((high-low)/resolution).astype(int)
        while np.prod(shape)>max_cells:
            resolution+=.01;shape=np.ceil((high-low)/resolution).astype(int)
        self.low,self.shape,self.resolution=low,shape,resolution
        self.surface=np.zeros(int(np.prod(shape)),np.uint16)
        self.clear=np.zeros_like(self.surface)
        self.surface_count=None;self.free_count=None
        self.session_count=session_count;self.current=None;self.finished=set();self.stats=[]

    def keys(self, points):
        points=np.asarray(points,float)
        xyz=np.floor((points-self.low)/self.resolution).astype(np.int64)
        valid=np.isfinite(points).all(axis=1)&(xyz>=0).all(axis=1)&(xyz<self.shape).all(axis=1)
        x,y,z=xyz[valid].T
        return ((z*self.shape[1]+y)*self.shape[0]+x),valid

    def start(self,index):
        if self.current is not None or index in self.finished or not 0<=index<self.session_count:
            raise ValueError('invalid evidence session order')
        self.current=index
        self.surface_count=np.zeros_like(self.surface);self.free_count=np.zeros_like(self.surface)

    def add_points(self,surface,free):
        if self.current is None:raise ValueError('no active evidence session')
        for points,counts in [(surface,self.surface_count),(free,self.free_count)]:
            keys,_=self.keys(points);keys=np.unique(keys)
            counts[keys]=np.minimum(counts[keys].astype(np.uint32)+1,65535)

    def add_depth(self,depth,info,pose,depth_max):
        surface=project_depth(depth,info,pose,stride=4,depth_max=depth_max)
        k=np.asarray(info['k']).reshape(3,3)
        v,u=np.mgrid[0:depth.shape[0]:8,0:depth.shape[1]:8]
        z=depth[::8,::8].astype(float)*info['depth_scale_m']
        good=(z>=.4)&(z<=depth_max)
        u,v,z=u[good],v[good],z[good]
        samples=np.arange(.25,depth_max-.1,.04)
        ray,step=np.nonzero(samples[None,:]<z[:,None]-.1)
        distances=samples[step]
        local=np.c_[(u[ray]-k[0,2])*distances/k[0,0],
                    (v[ray]-k[1,2])*distances/k[1,1],distances]
        free=local@pose[:3,:3].T+pose[:3,3]
        self.add_points(surface,free)

    def finish(self):
        if self.current is None:raise ValueError('no active evidence session')
        # A neighbouring occupied cell protects boundaries against voxel rounding
        # and modest registration/depth error. Clear space itself is not dilated.
        support=maximum_filter((self.surface_count>=2).reshape(tuple(self.shape[::-1])),size=3).ravel()
        clear=(self.free_count>=3)&~support
        bit=np.uint16(1<<self.current)
        self.surface[support]|=bit;self.clear[clear]|=bit
        self.stats.append({'session_index':self.current,'supported_cells':int(support.sum()),
                           'clear_cells':int(clear.sum())})
        self.finished.add(self.current);self.current=None
        self.surface_count=None;self.free_count=None

    def classify(self,points,session_index):
        if len(self.finished)!=self.session_count:raise ValueError('incomplete session evidence')
        keys,inside=self.keys(points)
        other=np.uint16(((1<<self.session_count)-1)^(1<<session_index))
        agree=(self.surface[keys]&other)!=0
        conflict=(self.clear[keys]&other)!=0
        reject=np.zeros(len(points),bool);confirmed=np.zeros(len(points),bool)
        reject[inside]=conflict&~agree;confirmed[inside]=agree
        return reject,confirmed

    def filter_depth(self,depth,info,pose,session_index):
        v,u=np.nonzero(depth)
        z=depth[v,u].astype(float)*info['depth_scale_m'];k=np.asarray(info['k']).reshape(3,3)
        local=np.c_[(u-k[0,2])*z/k[0,0],(v-k[1,2])*z/k[1,1],z]
        points=local@pose[:3,:3].T+pose[:3,3]
        reject,confirmed=self.classify(points,session_index)
        result=depth.copy();result[v[reject],u[reject]]=0
        return result,{'input_pixels':len(z),'rejected_conflict_pixels':int(reject.sum()),
                       'confirmed_other_session_pixels':int(confirmed.sum()),
                       'unconfirmed_retained_pixels':int((~reject&~confirmed).sum())}

    def metadata(self):
        return {'method':'cross_session_surface_and_observed_free_space_voxels_v1',
                'voxel_size_m':self.resolution,'surface_min_frames':2,'clear_min_frames':3,
                'clearance_before_measured_surface_m':.1,'surface_neighbour_radius_cells':1,
                'vote_unit':'independent recording, excluding current recording',
                'conflict_rule':'reject only when another recording sees free space and none supports surface',
                'unobserved_or_occluded':'retain without claiming cross-session confirmation',
                'bounds_low':self.low.tolist(),'shape_xyz':self.shape.tolist(),
                'session_evidence':self.stats,'accuracy_verified':False}


def build_consensus_cache(sources,filtering,cache_root,progress):
    positions=np.array([(s['world_from_slam']@pose_matrix(row))[:3,3]
                        for s in sources for _,row in s['keyframes']])
    padding=filtering.max_depth_m*1.8
    identities=list(dict.fromkeys(str(s['run']) for s in sources))
    evidence=SessionConsensus(positions.min(axis=0)-padding,positions.max(axis=0)+padding,len(identities))
    active=None
    for index,source in enumerate(sources):
        voter=identities.index(str(source['run']))
        if active!=voter:
            if active is not None:evidence.finish()
            evidence.start(voter);active=voter
        source['consensus_index']=voter;source['cached_frames']=[]
        info=source['calibration']['depth'];stats={}
        folder=Path(cache_root)/str(index);folder.mkdir()
        for row,frame,gap,depth in filtered_depths(source['run'],source['keyfile'],source['map_id'],info,
                                                  source['world_from_slam'],filtering,stats):
            path=None
            if frame is not None and np.count_nonzero(depth)>=100:
                pose=source['world_from_slam']@pose_matrix(row)@transform(info['T_left_depth'])
                evidence.add_depth(depth,info,pose,filtering.max_depth_m)
                path=folder/f'{len(source["cached_frames"])}.npy'
                np.save(path,depth,allow_pickle=False)
            source['cached_frames'].append((row,frame,gap,path))
            if len(source['cached_frames'])%150==0:
                progress(f'{source["run"].name}: cross-session evidence {len(source["cached_frames"])} keyframes')
        source['cached_filter_stats']=stats
    if active is not None:evidence.finish()
    return evidence


def cached_depths(source,stats):
    stats.update(source['cached_filter_stats'])
    for row,frame,gap,path in source['cached_frames']:
        yield row,frame,gap,np.load(path,allow_pickle=False) if path else None
