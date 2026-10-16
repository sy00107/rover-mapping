# ================================================================
# reconstruct.py
# 설명: 최종 map별 keyframe과 동기화된 원본 depth를 노트북에서 TSDF로 융합한다.
# 사용법: .venv-mapping/bin/python -m experiments.handheld_mapping.reconstruct RUN
# ================================================================
import argparse
from bisect import bisect_left
import json
import math
from pathlib import Path

import numpy as np

from experiments.handheld_mapping.storage import atomic_json
from experiments.handheld_mapping.depth_filter import DepthFilter, filtered_depths


def transform(value: list) -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("finite SE3 transform required")
    rotation = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-7)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-5)):
        raise ValueError("invalid SE3 transform")
    return matrix


def pose_matrix(row: dict) -> np.ndarray:
    x, y, z, qx, qy, qz, qw = [float(row[key]) for key in ("x", "y", "z", "qx", "qy", "qz", "qw")]
    quaternion = np.array([qx, qy, qz, qw])
    norm = np.linalg.norm(quaternion)
    if not np.isfinite([x, y, z, *quaternion]).all() or not .98 < norm < 1.02:
        raise ValueError("invalid camera pose")
    qx, qy, qz, qw = quaternion / norm
    matrix = np.eye(4)
    matrix[:3, :3] = [[1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
                        [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
                        [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]]
    matrix[:3, 3] = [x, y, z]
    return matrix


def nearest(stamps: list[int], stamp: int, tolerance_ns: int = 2_000_000) -> int | None:
    index = bisect_left(stamps, stamp)
    candidates = stamps[max(0, index-1):index+1]
    if not candidates:
        return None
    result = min(candidates, key=lambda value: abs(value-stamp))
    return result if abs(result-stamp) <= tolerance_ns else None


def reconstruct(run_dir: Path, voxel_size: float = .01, depth_max: float = 3.,
                progress=None, backend: str = "orb", gravity_alignments: dict | None = None,
                depth_filter: DepthFilter | None = None, output_root: Path | None = None) -> dict:
    import open3d as o3d
    run_dir = Path(run_dir).resolve()
    filtering = depth_filter or DepthFilter(max_depth_m=depth_max)
    if not math.isclose(filtering.max_depth_m, depth_max, rel_tol=0., abs_tol=1e-9):
        raise ValueError('depth filter and TSDF maximum distance differ')
    if backend not in ("orb", "orbline", "orb_offline", "orb_inertial"):
        raise ValueError("invalid reconstruction backend")
    if not math.isfinite(voxel_size) or not .002 <= voxel_size <= .1 or not .3 < depth_max <= 10:
        raise ValueError("invalid voxel/depth bounds")
    calibration = json.loads((run_dir / "capture_calibration.json").read_text())
    try:
        info = calibration["depth"]
        scale = float(info["depth_scale_m"])
        left_depth = transform(info["T_left_depth"])
        width, height = int(info["width"]), int(info["height"])
        k = np.asarray(info["k"], dtype=float).reshape(3, 3)
        if (not math.isfinite(scale) or not 0 < scale < .1 or width <= 0 or height <= 0
                or not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0
                or not np.allclose(k[2], [0, 0, 1]) or not np.allclose(k[[0, 1], [1, 0]], 0)
                or any(abs(float(v)) > 1e-8 for v in info.get("d", []))):
            raise ValueError("unsupported depth calibration")
    except (KeyError, TypeError, IndexError) as error:
        raise ValueError("depth scale and depth-to-left TF calibration required") from error
    keyfiles = sorted((run_dir / backend).glob("map_*_keyframes.csv"))
    if backend == 'orb_inertial':
        fusion = json.loads((run_dir/backend/'summary.json').read_text())
        identities = fusion.get('exported_imu_initialized_maps')
        # Compatibility with the existing measured-profile runner: it exports only
        # initialized maps and records their count in its completion contract.
        if identities is None and fusion.get('inertial_input', {}).get('clock_verified') is True:
            completion = json.loads((run_dir/backend/'runner_complete.json').read_text())
            if completion.get('imu_initialized_maps', 0) > 0:
                identities = [int(path.stem.split('_')[1]) for path in keyfiles]
        if (fusion.get('orb_fused') is not True or not isinstance(identities, list) or not identities
                or any(type(value) is not int or value < 0 for value in identities)):
            raise ValueError('initialized inertial export required for TSDF')
        keyfiles = [path for path in keyfiles if path.name in {f'map_{value}_keyframes.csv' for value in identities}]
    if not keyfiles:
        raise ValueError("원본 depth 또는 최종 keyframe이 없습니다")
    output_name = {"orb": "dense", "orbline": "dense_orbline", "orb_offline": "dense_offline",
                   "orb_inertial":"dense_inertial"}[backend]
    destination = Path(output_root).resolve() if output_root is not None else run_dir
    output = destination / output_name
    output.mkdir(parents=True, exist_ok=True)
    # 이전 결과는 원본과 분리한 채 새 summary가 성공한 경우에만 갱신한다.
    (output / "summary.json").unlink(missing_ok=True)
    intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, k[0, 0], k[1, 1], k[0, 2], k[1, 2])
    result = {"status": "complete", "method": "keyframe_TSDF", "voxel_size_m": voxel_size,
              "depth_max_m": depth_max, "timestamp_tolerance_ns": 2_000_000,
              "coordinate_system": "per-map initial camera optical; gravity alignment not applied",
              "accuracy_verified_on_real_camera": False, "maps": [], "skipped_maps": []}
    result['depth_filter'] = filtering.metadata()
    if backend == 'orb_inertial':
        result.update(coordinate_system='ORB stereo-inertial map; gravity estimate unverified',
                      imu_clock_verified=False, pose_source='orb_inertial')
    for keyfile in keyfiles:
        map_id = keyfile.name.removeprefix("map_").removesuffix("_keyframes.csv")
        if not map_id.isdecimal():
            raise ValueError("keyframe map identity mismatch")
        from experiments.handheld_mapping.gravity import load_alignment
        gravity = (load_alignment(run_dir,backend,int(map_id)) if gravity_alignments is None
                   else gravity_alignments.get(int(map_id)))
        world_from_slam = transform(gravity['world_from_slam']) if gravity else np.eye(4)
        volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=voxel_size, sdf_trunc=voxel_size*4,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.NoColor)
        integrated = 0
        camera_positions = []
        total = max_gap = 0
        filter_stats = {}
        for row, frame, gap, depth in filtered_depths(run_dir, keyfile, int(map_id), info,
                                                     world_from_slam, filtering, filter_stats):
            total += 1
            if frame is None:
                continue
            if np.count_nonzero(depth) < 100:
                continue
            world_depth = world_from_slam @ pose_matrix(row) @ left_depth
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                o3d.geometry.Image(np.zeros((height, width, 3), np.uint8)),
                o3d.geometry.Image(depth), depth_scale=1/scale, depth_trunc=depth_max,
                convert_rgb_to_intensity=False)
            volume.integrate(rgbd, intrinsic, np.linalg.inv(world_depth))
            camera_positions.append(world_depth[:3, 3])
            integrated += 1
            max_gap = max(max_gap, gap)
            if progress:
                progress(f"지도 {map_id}: {integrated} keyframe 융합")
        if integrated == 0:
            result["skipped_maps"].append(map_id)
            continue
        mesh, cloud = volume.extract_triangle_mesh(), volume.extract_point_cloud()
        mesh.compute_vertex_normals()
        if len(mesh.triangles) == 0 or len(cloud.points) == 0:
            raise ValueError("empty TSDF surface")
        cleanup_stats = None
        if filtering.surface_cleanup:
            from experiments.handheld_mapping.surface_cleanup import clean_surface_mesh
            mesh, cleanup_stats = clean_surface_mesh(mesh, camera_positions)
            if not len(mesh.triangles):
                raise ValueError('surface cleanup removed all geometry')
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(np.asarray(mesh.vertices))
            cloud.normals = o3d.utility.Vector3dVector(np.asarray(mesh.vertex_normals))
        mesh_path, cloud_path = f"{output_name}/map_{map_id}_mesh.ply", f"{output_name}/map_{map_id}_cloud.ply"
        if not o3d.io.write_triangle_mesh(str(destination / mesh_path), mesh):
            raise OSError("mesh write failed")
        if not o3d.io.write_point_cloud(str(destination / cloud_path), cloud):
            raise OSError("point cloud write failed")
        result["maps"].append({"map_id": int(map_id), "keyframes": total, "integrated_frames": integrated,
                              "unintegrated_keyframes": total-integrated,
                              "max_timestamp_difference_ns": max_gap, "mesh": mesh_path, "cloud": cloud_path,
                              "vertices": len(mesh.vertices), "triangles": len(mesh.triangles),
                              "coordinate_system":"gravity_z_up" if gravity else "initial_camera_optical",
                              "world_from_slam":world_from_slam.tolist(),
                              "gravity_method":gravity['method'] if gravity else None,
                              "depth_filter_stats":filter_stats,
                              "surface_cleanup": cleanup_stats,
                              "gravity_extrinsic_verified":gravity.get('extrinsic_verified',False) if gravity else False})
    if not result["maps"]:
        raise ValueError("No matching depth/keyframes with valid depth: 일치하는 유효 depth가 없습니다")
    if result["skipped_maps"] or any(item["unintegrated_keyframes"] for item in result["maps"]):
        result["status"] = "partial"
    if any(row['coordinate_system'] == 'gravity_z_up' for row in result['maps']):
        result['coordinate_system'] = 'per-map; see maps[].coordinate_system; elevation relative to anchor camera'
    atomic_json(output / "summary.json", result)
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--voxel-size", type=float, default=.01)
    parser.add_argument('--backend', choices=('orb', 'orb_offline', 'orbline', 'orb_inertial'), default='orb')
    parser.add_argument('--depth-max', type=float, default=3., help='사용할 최대 optical Z depth [m]')
    parser.add_argument('--output-root', type=Path, help='원본과 분리한 결과 저장 폴더')
    args = parser.parse_args()
    print(json.dumps(reconstruct(args.run_dir, args.voxel_size, args.depth_max,
                                 backend=args.backend, output_root=args.output_root), indent=2))


if __name__ == "__main__":
    main()
