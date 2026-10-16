"""Image/depth evidence for rigid registration of independently recorded sessions."""
from functools import lru_cache
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from experiments.handheld_mapping.depth_filter import DepthFilter, spatial_filter
from experiments.handheld_mapping.gravity import load_alignment, fingerprint
from experiments.handheld_mapping.recording import ordered_keyframes
from experiments.handheld_mapping.reconstruct import nearest, pose_matrix, transform


class SessionImages:
    """Read archived final poses, left images and calibrated depth without changing them."""

    def __init__(self, run: Path, map_id: int = 0, allow_visual_frame: bool = False):
        self.run = Path(run).resolve()
        self.map_id = map_id
        self.calibration = json.loads((self.run/'capture_calibration.json').read_text())
        self.k = np.asarray(self.calibration['stereo']['left']['k']).reshape(3, 3)
        gravity = load_alignment(self.run, 'orb_offline', map_id)
        self.gravity_available = gravity is not None
        if gravity is None and not allow_visual_frame:
            raise ValueError(f'gravity reference required: {self.run.name}')
        self.gravity = transform(gravity['world_from_slam']) if gravity else np.eye(4)
        names = ['frames.jsonl','capture_calibration.json',f'orb_offline/map_{map_id}_keyframes.csv']
        if gravity is not None:
            names.append(f'gravity/orb_offline/map_{map_id}.json')
        self.provenance = {str(self.run/name):fingerprint(self.run/name) for name in names}
        self.frames = {'left': {}, 'depth': {}}
        with (self.run/'frames.jsonl').open() as stream:
            for line in stream:
                row = json.loads(line)
                if row['stream'] in self.frames:
                    self.frames[row['stream']][row['stamp_ns']] = row
        self.stamps = {side: sorted(rows) for side, rows in self.frames.items()}
        self.records = []
        for stamp, row in ordered_keyframes(self.run/f'orb_offline/map_{map_id}_keyframes.csv', map_id):
            left = nearest(self.stamps['left'], stamp)
            depth = nearest(self.stamps['depth'], stamp)
            if left is not None and depth is not None:
                self.records.append({'stamp': stamp, 'row': row, 'left': left, 'depth': depth,
                                     'pose': self.gravity @ pose_matrix(row)})
        if len(self.records) < 10:
            raise ValueError('too few archived keyframes for session registration')
        self.poses = np.array([row['pose'] for row in self.records])

    def raw_path(self, side, stamp):
        path = (self.run/self.frames[side][stamp]['path']).resolve()
        if not path.is_relative_to((self.run/'raw').resolve()):
            raise ValueError('image path outside source raw directory')
        return path

    @lru_cache(maxsize=300)
    def features(self, index):
        row = self.records[index]
        image = cv2.imread(str(self.raw_path('left', row['left'])), cv2.IMREAD_GRAYSCALE)
        depth = cv2.imread(str(self.raw_path('depth', row['depth'])), cv2.IMREAD_UNCHANGED)
        if image is None or depth is None:
            raise ValueError('missing source image/depth')
        info = self.calibration['depth']
        depth, _ = spatial_filter(depth, info['depth_scale_m'], DepthFilter(max_depth_m=2.))
        # Reproject rectified metric depth to the left camera, including extrinsics.
        dk = np.asarray(info['k']).reshape(3, 3)
        v, u = np.nonzero(depth)
        z = depth[v, u]*info['depth_scale_m']
        xyz = np.c_[(u-dk[0, 2])*z/dk[0, 0], (v-dk[1, 2])*z/dk[1, 1], z]
        extrinsic = transform(info['T_left_depth'])
        xyz = xyz @ extrinsic[:3, :3].T+extrinsic[:3, 3]
        good = xyz[:, 2] > .1
        xyz = xyz[good]
        pixels = np.rint((xyz @ self.k.T)[:, :2]/xyz[:, 2, None]).astype(int)
        h, w = image.shape
        good = (pixels[:, 0] >= 0) & (pixels[:, 0] < w) & (pixels[:, 1] >= 0) & (pixels[:, 1] < h)
        aligned = np.full(h*w, np.inf)
        np.minimum.at(aligned, pixels[good, 1]*w+pixels[good, 0], xyz[good, 2])
        aligned = aligned.reshape(h, w)
        keys, descriptors = cv2.SIFT_create(nfeatures=1600, contrastThreshold=.025).detectAndCompute(image, None)
        if descriptors is None:
            return np.empty((0, 2)), np.empty((0, 128), np.float32), np.empty((0, 3))
        uv = np.array([key.pt for key in keys])
        ij = np.rint(uv).astype(int)
        values = aligned[np.clip(ij[:, 1], 0, h-1), np.clip(ij[:, 0], 0, w-1)]
        local = np.c_[(uv[:, 0]-self.k[0, 2])*values/self.k[0, 0],
                      (uv[:, 1]-self.k[1, 2])*values/self.k[1, 1], values]
        world = np.full(local.shape, np.nan)
        valid = np.isfinite(local).all(axis=1)
        world[valid] = local[valid] @ row['pose'][:3, :3].T+row['pose'][:3, 3]
        descriptors = np.sqrt(descriptors/np.maximum(descriptors.sum(axis=1, keepdims=True), 1e-12))
        return uv, descriptors, world


def match_view(source, target, si, ti):
    su, sd, sw = source.features(si)
    tu, td, _ = target.features(ti)
    if min(len(sd), len(td)) < 20:
        return None
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    forward = matcher.knnMatch(sd, td, k=2)
    backward = matcher.knnMatch(td, sd, k=2)
    reverse = {a.queryIdx: a.trainIdx for a, b in backward if a.distance < .72*b.distance}
    matches = [(a.queryIdx, a.trainIdx) for a, b in forward
               if a.distance < .72*b.distance and reverse.get(a.trainIdx) == a.queryIdx
               and np.isfinite(sw[a.queryIdx]).all()]
    if len(matches) < 15:
        return None
    a, b = np.array(matches).T
    cv2.setRNGSeed(43)
    okay, rvec, tvec, inliers = cv2.solvePnPRansac(
        sw[a].astype(np.float64), tu[b].astype(np.float64), target.k, None,
        iterationsCount=500, reprojectionError=3., confidence=.999, flags=cv2.SOLVEPNP_EPNP)
    if not okay or inliers is None or len(inliers) < 15 or len(inliers)/len(matches) < .35:
        return None
    keep = inliers[:, 0]
    xyz, uv = sw[a[keep]], tu[b[keep]]
    if np.ptp(uv[:, 0]) < 100 or np.ptp(uv[:, 1]) < 60:
        return None
    rvec, tvec = cv2.solvePnPRefineLM(xyz, uv, target.k, None, rvec, tvec)
    camera_from_source = np.eye(4)
    camera_from_source[:3, :3] = cv2.Rodrigues(rvec)[0]
    camera_from_source[:3, 3] = tvec.ravel()
    candidate = target.records[ti]['pose'] @ camera_from_source
    if getattr(source,'gravity_available',True) and candidate[2, 2] < np.cos(np.deg2rad(5.)):
        return None
    return {'source_index': si, 'target_index': ti, 'matches': len(matches), 'inliers': len(keep),
            'transform': candidate, 'source_xyz': xyz, 'target_uv': uv,
            'target_pose': target.records[ti]['pose']}


def projected_errors(matrix, pairs, k):
    errors = []
    for pair in pairs:
        relative = np.linalg.inv(pair['target_pose']) @ matrix
        xyz = pair['source_xyz'] @ relative[:3, :3].T+relative[:3, 3]
        pixels = (xyz @ k.T)[:, :2]/np.maximum(xyz[:, 2, None], .01)
        error = pixels-pair['target_uv']
        error[xyz[:, 2] <= .1] = 1000.
        errors.append(error)
    return np.concatenate(errors)


@lru_cache(maxsize=2)
def appearance_database(target):
    """Build compact VLAD retrieval signatures to find views without a pose prior."""
    indices = np.unique(np.linspace(0,len(target.records)-1,min(110,len(target.records))).astype(int))
    descriptors = [target.features(int(i))[1] for i in indices]
    available = [d for d in descriptors if len(d)]
    if not available:
        raise ValueError('reference images have no usable features')
    training = np.concatenate(available)
    rng = np.random.default_rng(37)
    training = training[rng.choice(len(training),min(14000,len(training)),replace=False)]
    cv2.setRNGSeed(37)
    _,_,centers = cv2.kmeans(training,64,None,
        (cv2.TERM_CRITERIA_MAX_ITER|cv2.TERM_CRITERIA_EPS,15,.001),1,cv2.KMEANS_PP_CENTERS)
    signatures = np.array([appearance_signature(d,centers) for d in descriptors])
    return indices, centers, signatures


def appearance_signature(descriptors, centers):
    if len(descriptors)==0:
        return np.zeros(centers.size,np.float32)
    labels = np.argmax(2*descriptors@centers.T-np.sum(centers**2,axis=1),axis=1)
    residual = np.zeros_like(centers)
    np.add.at(residual,labels,descriptors-centers[labels])
    residual /= np.maximum(np.linalg.norm(residual,axis=1,keepdims=True),1e-12)
    signature = (np.sign(residual)*np.sqrt(np.abs(residual))).ravel()
    return signature/max(np.linalg.norm(signature),1e-12)


def visual_registration(source, target, priors, progress=print, appearance_retrieval=False):
    """Use geometric candidates only to retrieve views; verify independent image evidence."""
    pairs = set()
    sample = np.unique(np.linspace(0, len(source.records)-1, min(55, len(source.records))).astype(int))
    if appearance_retrieval:
        progress(f'{source.run.name}: retrieving similar archived images')
        target_indices, centers, signatures = appearance_database(target)
        for si in sample:
            query=appearance_signature(source.features(int(si))[1],centers)
            best=np.argsort(signatures@query)[-8:]
            pairs.update((int(si),int(target_indices[ti])) for ti in best)
    for prior in priors:
        poses = np.asarray([prior @ source.poses[i] for i in sample])
        for si, pose in zip(sample, poses):
            distances = np.linalg.norm(target.poses[:, :3, 3]-pose[:3, 3], axis=1)
            angles = np.arccos(np.clip(target.poses[:, :3, 2] @ pose[:3, 2], -1., 1.))
            candidates = np.flatnonzero((distances < 1.6) & (angles < np.deg2rad(50)))
            order = candidates[np.argsort(distances[candidates]+angles[candidates]*.6)[:3]]
            for ti in order:
                pairs.add((int(si), int(ti)))
    evidence = []
    for index, (si, ti) in enumerate(sorted(pairs)):
        item = match_view(source, target, si, ti)
        if item is not None:
            evidence.append(item)
            progress(f'{source.run.name}: image pair {si}/{ti}: {item["inliers"]} verified features')
        elif index % 40 == 0:
            progress(f'{source.run.name}: checked {index+1}/{len(pairs)} image pairs')
    if len(evidence) < 4:
        raise ValueError(f'insufficient independent image matches: {len(evidence)}')
    held_ids = set(int(i) for i in sample[::3])
    return fit_visual_pairs(evidence, source, target, held_ids, len(pairs))


def fit_visual_pairs(evidence, source, target, held_ids, candidate_pairs=0):
    """Fit using training views only, then test separately reserved source images."""
    training = [p for p in evidence if p['source_index'] not in held_ids]
    held = [p for p in evidence if p['source_index'] in held_ids]
    if not training or len({p['source_index'] for p in held}) < 2:
        raise ValueError('insufficient independent held-out image views')
    # Neither consensus selection nor its initial transform uses held-out images.
    clusters = []
    for seed in training:
        support = [pair for pair in training
                   if np.median(np.linalg.norm(projected_errors(seed['transform'], [pair], target.k), axis=1)) < 5.]
        clusters.append(support)
    support = max(clusters, key=lambda cluster: (len({p['source_index'] for p in cluster}), len(cluster),
                                                sum(p['inliers'] for p in cluster)))
    unique = sorted({p['source_index'] for p in support})
    if len(unique) < 4:
        raise ValueError(f'image transforms inconsistent across source views: {len(unique)}')
    train = support
    seed = support[0]['transform']
    initial = np.r_[Rotation.from_matrix(seed[:3, :3]).as_rotvec(), seed[:3, 3]]

    def unpack(value):
        result = np.eye(4)
        result[:3, :3] = Rotation.from_rotvec(value[:3]).as_matrix()
        result[:3, 3] = value[3:]
        return result

    fit = least_squares(lambda value: projected_errors(unpack(value), train, target.k).ravel(),
                        initial, loss='huber', f_scale=2., max_nfev=100)
    result = unpack(fit.x)
    held_errors = np.linalg.norm(projected_errors(result, held, target.k), axis=1)
    train_errors = np.linalg.norm(projected_errors(result, train, target.k), axis=1)
    if np.median(held_errors) > 3. or np.percentile(held_errors, 90) > 6.:
        raise ValueError('held-out image reprojection inconsistent')
    positions = source.poses[unique, :3, 3]
    span = float(np.max(np.linalg.norm(positions[:, None]-positions[None, :], axis=2)))
    gravity_available=getattr(source,'gravity_available',True)
    if (span < .5 or not fit.success or not np.isfinite(result).all()
            or (gravity_available and result[2, 2] < np.cos(np.deg2rad(3.)))):
        raise ValueError('insufficient spatial support or incompatible gravity')
    report = {'source': str(source.run), 'reference': str(target.run),
              'method': 'mutual_RootSIFT_metric_depth_PnP_multiview_reprojection',
              'source_frame':'gravity' if gravity_available else 'slam',
              'source_map_id':getattr(source,'map_id',0),
              'source_to_reference':result.tolist(),
              'candidate_image_pairs': candidate_pairs, 'matched_pairs': len(evidence),
              'consensus_pairs': len(support), 'distinct_source_views': len(unique),
              'held_out_source_views': sorted({p['source_index'] for p in held}),
              'held_out_image_pairs': len(held), 'held_out_used_for_consensus_or_fit': False,
              'camera_support_span_m': span,
              'train_reprojection_p50_p90_px': np.percentile(train_errors, [50, 90]).tolist(),
              'held_out_reprojection_p50_p90_px': np.percentile(held_errors, [50, 90]).tolist(),
              'matched_features': sum(p['inliers'] for p in support),
              'evidence_views': [{k:p[k] for k in ('source_index','target_index','matches','inliers')}
                                 for p in support], 'accuracy_verified': False}
    if gravity_available:
        report['source_gravity_to_reference_gravity']=result.tolist()
    if hasattr(source,'provenance') and hasattr(target,'provenance'):
        report['source_sha256']={**source.provenance,**target.provenance}
    return result, report
