"""같은 RUN/backend/map의 지면 층을 3D 표면 높이와 대조하여 표시 색을 정한다."""
import hashlib
import json
import math
from pathlib import Path
import re

import numpy as np

from experiments.handheld_mapping.reconstruct import transform


GROUND, OBSTACLE, UNKNOWN = 1, 2, 0
PALETTE = np.array([[.52, .57, .62], [.20, .78, .35], [1., .38, .10]])
SLOPE_COLOR = np.array([1., .20, .65])
SLOPE_THRESHOLD_DEG = 5.
LEGEND = '초록: 평지 | 핑크: 경사 지면(5° 이상) | 주황: 장애물 표면 | 회색: 미분류'


def load_surface_layers(geometry_path: Path):
    """최신 지형 revision만 사용하며 좌표 변환·입력 출처가 다르면 색칠하지 않는다."""
    from experiments.handheld_mapping.viewer import FOLDERS
    path = Path(geometry_path).resolve()
    backend = next((name for name, folder in FOLDERS.items() if folder == path.parent.name), None)
    match = re.fullmatch(r'map_(\d+)_(?:mesh|cloud)\.ply', path.name)
    if backend is None or match is None:
        return None
    identity, run = int(match[1]), path.parent.parent
    from experiments.handheld_mapping.depth_variants import variant_source
    source_run = variant_source(run, backend)
    root = run/'terrain'/backend
    candidates = []
    if root.is_dir():
        for folder in root.iterdir():
            revision = re.fullmatch(rf'map_{identity}(?:_v([2-9]\d*|1\d+))?', folder.name)
            if folder.is_dir() and revision:
                candidates.append((int(revision[1] or 1), folder))
    if not candidates:
        return None
    folder = max(candidates, key=lambda entry: entry[0])[1]
    summary = json.loads((folder/'summary.json').read_text())
    if (summary.get('backend') != backend or summary.get('map_id') != identity
            or Path(summary.get('source', '')).resolve() != source_run):
        raise ValueError('지형 결과의 촬영/backend/map이 3D 지도와 다릅니다')
    # No fallback to an older revision after a failed/newer terrain replay.
    sources = ('capture_calibration.json', f'{backend}/map_{identity}_keyframes.csv')
    for name in sources:
        expected = summary.get('source_sha256', {}).get(name)
        if expected is None:
            raise ValueError('지형 결과를 한 번 다시 생성하면 지면 색을 표시할 수 있습니다')
        if hashlib.sha256((run/name).read_bytes()).hexdigest() != expected:
            raise ValueError('지형 생성 이후 입력이 바뀌었습니다. 지형을 다시 생성하세요')
    dense = json.loads((path.parent/'summary.json').read_text())
    row = next((item for item in dense['maps'] if item['map_id'] == identity), None)
    if row is None:
        raise ValueError('3D 지도 좌표계 정보가 없습니다')
    geometry_frame = row.get('coordinate_system')
    if geometry_frame == 'gravity_z_up':
        world_from_slam = transform(row['world_from_slam'])
    elif geometry_frame == 'initial_camera_optical':
        world_from_slam = np.eye(4)
    else:
        raise ValueError('3D 지도 좌표계가 명시되지 않았습니다')
    if summary.get('depth_filter') != dense.get('depth_filter'):
        raise ValueError('3D 지도와 지형의 depth 처리 설정이 다릅니다')
    grid_from_geometry = transform(summary['grid_from_world']) @ np.linalg.inv(world_from_slam)
    grid = summary['grid']
    bounds, resolution = np.asarray(grid['bounds'], float), float(grid['resolution'])
    if (bounds.shape != (4,) or not np.isfinite(bounds).all() or not math.isfinite(resolution)
            or resolution <= 0 or bounds[1] <= bounds[0] or bounds[3] <= bounds[2]):
        raise ValueError('invalid terrain bounds/resolution')
    shape = (math.ceil((bounds[3]-bounds[2])/resolution), math.ceil((bounds[1]-bounds[0])/resolution))
    required = ('ground', 'surface_observed', 'obstacle_observed', 'reference_ground_height_m',
                'reference_ground_gradient_x', 'reference_ground_gradient_y')
    with np.load(folder/'terrain.npz', allow_pickle=False) as arrays:
        if not all(key in arrays for key in required):
            raise ValueError('지형의 지면 기준 층이 없습니다. 지형을 다시 생성하세요')
        layers = {key: arrays[key] for key in required}
    if any(array.shape != shape for array in layers.values()):
        raise ValueError('지형 격자 크기가 맞지 않습니다')
    for key in ('ground', 'surface_observed', 'obstacle_observed'):
        if layers[key].dtype != bool:
            raise ValueError('invalid terrain classification mask')
    band = float(summary['ground_separation']['surface_band_m'])
    voxel = float(dense['voxel_size_m'])
    obstacle_height = float(summary['rover_limits']['max_step_m'])
    if not all(math.isfinite(v) and v > 0 for v in (band, voxel, obstacle_height)):
        raise ValueError('invalid surface colour thresholds')
    return {'layers': layers, 'bounds': bounds, 'resolution': resolution,
            'grid_from_geometry': transform(grid_from_geometry), 'output_dir': str(folder),
            'ground_tolerance_m': max(band/2, voxel*2),
            'obstacle_threshold_m': max(obstacle_height, band/2, voxel*2)}


def classify_surface_points(points, terrain):
    """같은 XY의 위쪽 물체와 지면을 높이로 구분하고 셀 내부의 경사도 반영한다."""
    points = np.asarray(points, float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('expected Nx3 surface points')
    labels = np.zeros(len(points), np.uint8)
    layers, bounds, res = terrain['layers'], terrain['bounds'], terrain['resolution']
    matrix = terrain['grid_from_geometry']
    for start in range(0, len(points), 200_000):
        stop = min(start+200_000, len(points))
        xyz = points[start:stop] @ matrix[:3, :3].T+matrix[:3, 3]
        finite = np.isfinite(xyz).all(axis=1)
        inside = (finite & (xyz[:, 0] >= bounds[0]) & (xyz[:, 0] < bounds[1])
                  & (xyz[:, 1] >= bounds[2]) & (xyz[:, 1] < bounds[3]))
        indices = np.flatnonzero(inside)
        x = np.floor((xyz[indices, 0]-bounds[0])/res).astype(int)
        y = np.floor((xyz[indices, 1]-bounds[2])/res).astype(int)
        if not len(indices):
            continue
        h = layers['reference_ground_height_m'][y, x]
        gx, gy = layers['reference_ground_gradient_x'][y, x], layers['reference_ground_gradient_y'][y, x]
        expected = h+gx*(xyz[indices, 0]-(bounds[0]+(x+.5)*res))+gy*(xyz[indices, 1]-(bounds[2]+(y+.5)*res))
        delta = xyz[indices, 2]-expected
        known = layers['surface_observed'][y, x] & layers['obstacle_observed'][y, x] & np.isfinite(delta)
        ground = known & layers['ground'][y, x] & (np.abs(delta) <= terrain['ground_tolerance_m'])
        obstacle = known & (delta >= terrain['obstacle_threshold_m'])
        labels[start+indices[ground]] = GROUND
        labels[start+indices[obstacle]] = OBSTACLE
    return labels


def surface_point_colors(points, labels, terrain):
    """Colour observed ground slopes relative to gravity without changing semantic labels."""
    points = np.asarray(points)
    colors = PALETTE[labels].copy()
    xyz = points @ terrain['grid_from_geometry'][:3,:3].T + terrain['grid_from_geometry'][:3,3]
    bounds, res = terrain['bounds'], terrain['resolution']
    ids = np.flatnonzero((labels == GROUND) & np.isfinite(xyz).all(axis=1)
                        & (xyz[:,0] >= bounds[0]) & (xyz[:,0] < bounds[1])
                        & (xyz[:,1] >= bounds[2]) & (xyz[:,1] < bounds[3]))
    x = np.floor((xyz[ids,0]-bounds[0])/res).astype(int)
    y = np.floor((xyz[ids,1]-bounds[2])/res).astype(int)
    layers = terrain['layers']
    slope = np.degrees(np.arctan(np.hypot(layers['reference_ground_gradient_x'][y,x],
                                        layers['reference_ground_gradient_y'][y,x])))
    colors[ids[np.isfinite(slope) & (slope >= SLOPE_THRESHOLD_DEG)]] = SLOPE_COLOR
    return colors


def apply_surface_colors(geometry, geometry_path, smooth_obstacles: bool = False,
                         fill_ground_gaps: bool = False):
    """저장 표면으로 분류하고 선택적으로 화면의 주황 메시만 평활화한다."""
    import open3d as o3d
    terrain = load_surface_layers(geometry_path)
    if terrain is None:
        return None
    mesh = isinstance(geometry, o3d.geometry.TriangleMesh)
    points = np.asarray(geometry.vertices if mesh else geometry.points)
    labels = classify_surface_points(points, terrain)
    ground_gaps = None
    if mesh and fill_ground_gaps:
        from experiments.handheld_mapping.ground_display import fill_small_ground_gaps
        ground_gaps = fill_small_ground_gaps(geometry, labels)
    smoothing = None
    if mesh and smooth_obstacles:
        from experiments.handheld_mapping.obstacle_display import smooth_obstacle_display
        smoothing = smooth_obstacle_display(geometry, labels)
    colors = o3d.utility.Vector3dVector(surface_point_colors(np.asarray(geometry.vertices if mesh else geometry.points), labels, terrain))
    if mesh:
        geometry.vertex_colors = colors
    else:
        geometry.colors = colors
    counts = np.bincount(labels, minlength=3)
    return {'ground': int(counts[GROUND]), 'obstacle': int(counts[OBSTACLE]),
            'unknown': int(counts[UNKNOWN]), 'terrain': terrain['output_dir'],
            'obstacle_smoothing': smoothing,
            'ground_gap_fill': ground_gaps,
            'ground_tolerance_m': terrain['ground_tolerance_m'],
            'obstacle_threshold_m': terrain['obstacle_threshold_m']}
