"""Independent-feature validated relocalization of archived views into a reference map."""
import cv2
import numpy as np


def solve_view(world,uv,k,feature_ids):
    """Fit from two-thirds of query features; reserve the rest before RANSAC/refinement."""
    train=np.asarray(feature_ids)%3!=0;held=~train
    if train.sum()<25 or held.sum()<12:return None
    if min(np.ptp(uv[train],axis=0))<60 or min(np.ptp(uv[held],axis=0))<40:return None
    cv2.setRNGSeed(51)
    okay,rvec,tvec,ids=cv2.solvePnPRansac(world[train],uv[train],k,None,
        iterationsCount=1000,reprojectionError=3.,confidence=.999,flags=cv2.SOLVEPNP_EPNP)
    if not okay or ids is None or len(ids)<25:return None
    w,u=world[train][ids[:,0]],uv[train][ids[:,0]]
    rvec,tvec=cv2.solvePnPRefineLM(w,u,k,None,rvec,tvec)
    camera_from_world=np.eye(4);camera_from_world[:3,:3]=cv2.Rodrigues(rvec)[0];camera_from_world[:3,3]=tvec.ravel()
    points=world[held]@camera_from_world[:3,:3].T+camera_from_world[:3,3]
    if (points[:,2]<=.15).any():return None
    predicted=(points@k.T)[:,:2]/points[:,2,None]
    errors=np.linalg.norm(predicted-uv[held],axis=1)
    p50,p90=np.percentile(errors,[50,90])
    return dict(pose=np.linalg.inv(camera_from_world),training_inliers=len(ids),held_features=int(held.sum()),
        held_p50_px=float(p50),held_p90_px=float(p90),held_out_used_for_fit=False,
        passed=bool(p50<=3 and p90<=6))


def candidate(source,target,si,ti):
    su,sd,sw=source.features(si);tu,td,tw=target.features(ti)
    if min(len(sd),len(td))<40:return None
    matcher=cv2.BFMatcher(cv2.NORM_L2)
    forward=matcher.knnMatch(sd,td,k=2);back=matcher.knnMatch(td,sd,k=2)
    reverse={a.queryIdx:a.trainIdx for a,b in back if a.distance<.72*b.distance}
    pairs=[(a.queryIdx,a.trainIdx) for a,b in forward if a.distance<.72*b.distance
        and reverse.get(a.trainIdx)==a.queryIdx and np.isfinite(tw[a.trainIdx]).all()]
    if len(pairs)<40:return None
    q,t=np.asarray(pairs).T
    result=solve_view(tw[t].astype(float),su[q].astype(float),source.k,q)
    if result is None:return None
    # Evaluate metric query depth independently; never used to fit the camera pose.
    finite=np.isfinite(sw[q]).all(axis=1)
    if finite.sum()>=12:
        original=source.records[si]['pose'];local=(sw[q[finite]]-original[:3,3])@original[:3,:3]
        reference_local=(tw[t[finite]]-result['pose'][:3,3])@result['pose'][:3,:3]
        dz=abs(local[:,2]-reference_local[:,2]);result['depth_check_points']=int(finite.sum())
        result['depth_error_p50_p90_m']=np.percentile(dz,[50,90]).tolist()
        result['passed']=bool(result['passed'] and np.median(dz)<.06 and np.percentile(dz,90)<.15)
    else:
        result['passed']=False;result['depth_check_points']=int(finite.sum())
    result.update(source_index=si,target_index=ti,matches=len(pairs))
    return result


def relocalize(source,target,progress=print):
    from .session_registration import appearance_database,appearance_signature
    indices,centers,signatures=appearance_database(target);accepted=[];rejected=[]
    for si in range(len(source.records)):
        query=appearance_signature(source.features(si)[1],centers)
        best=np.argsort(signatures@query)[-8:]
        candidates=[r for ti in best if (r:=candidate(source,target,si,int(indices[ti]))) is not None]
        # Select by training inliers; held-out residuals may reject but never select the winner.
        if not candidates:continue
        selected=max(candidates,key=lambda r:r['training_inliers'])
        selected['source_stamp_ns']=source.records[si]['stamp']
        if selected['passed']:
            accepted.append(selected);progress(f'POSE_OK {source.run.name}:{source.map_id} #{si} train={selected["training_inliers"]} held={selected["held_features"]} p90={selected["held_p90_px"]:.2f}px')
        else:rejected.append(selected)
    # Reject unsupported jumps between temporally neighbouring relocalized views.
    stable=[]
    for i,item in enumerate(accepted):
        neighbors=[]
        for j in [i-1,i+1]:
            if not 0<=j<len(accepted):continue
            other=accepted[j];dt=abs(item['source_stamp_ns']-other['source_stamp_ns'])/1e9
            distance=np.linalg.norm(item['pose'][:3,3]-other['pose'][:3,3])
            relative=item['pose'][:3,:3].T@other['pose'][:3,:3]
            angle=np.arccos(np.clip((np.trace(relative)-1)/2,-1,1))
            neighbors.append(distance<=.3*dt+.12 and angle<=.8*dt+.15)
        if neighbors and all(neighbors):stable.append(item)
    return stable,dict(independent_candidate_poses=len(accepted),temporally_stable_poses=len(stable),
        held_out_rejections=len(rejected),method='reference_metric_features_query_PnP_disjoint_feature_validation',
        maximum_held_p50_px=3,maximum_held_p90_px=6,query_depth_p50_limit_m=.06,query_depth_p90_limit_m=.15)
