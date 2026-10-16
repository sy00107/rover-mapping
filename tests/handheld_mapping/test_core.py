# ================================================================
# test_core.py
# 설명: binary 전송 오류와 TSDF의 단위·좌표·지도 분리를 실제 데이터로 검증한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_core.py -q
# ================================================================
import io
import json
import struct
import zlib

import cv2
import numpy as np
import pytest


def test_packet_round_trip_and_truncated_payload():
    from experiments.handheld_mapping.protocol import read_packet, write_packet
    stream = io.BytesIO()
    write_packet(stream, {"kind": "frame", "stamp_ns": 123}, b"abc\x00")
    stream.seek(0)
    assert read_packet(stream) == ({"kind": "frame", "stamp_ns": 123}, b"abc\x00")
    assert read_packet(stream) is None
    with pytest.raises(EOFError):
        read_packet(io.BytesIO(stream.getvalue()[:-1]))


def test_packet_limits_and_partial_header():
    from experiments.handheld_mapping.protocol import read_packet
    with pytest.raises(ValueError):
        read_packet(io.BytesIO(struct.pack(">II", 999999999, 1)))
    with pytest.raises(EOFError):
        read_packet(io.BytesIO(b"\x00\x01"))


def calibration() -> dict:
    transform = np.eye(4)
    transform[0, 3] = .2
    return {"width": 80, "height": 60, "k": [70, 0, 39.5, 0, 70, 29.5, 0, 0, 1],
            "depth_scale_m": .001, "T_left_depth": transform.tolist(),
            "depth_frame": "depth_optical", "left_frame": "left_optical"}


def make_run(path, maps=(0,)):
    from experiments.handheld_mapping.storage import RunWriter
    writer = RunWriter(path)
    writer.accept({"kind": "calibration", "depth": calibration()}, b"")
    depth = np.full((60, 80), 2000, np.uint16)
    for i, map_id in enumerate(maps):
        stamp = (i + 1) * 1_000_000_000
        writer.accept({"kind": "image", "stream": "depth", "stamp_ns": stamp,
                       "width": 80, "height": 60, "step": 160, "encoding": "16UC1",
                       "frame_id": "depth_optical", "compression": "zlib"}, zlib.compress(depth.tobytes()))
        folder = path / "orb"
        folder.mkdir(exist_ok=True)
        with (folder / f"map_{map_id}_keyframes.csv").open("w") as stream:
            stream.write("map_id,keyframe_id,source_stamp,x,y,z,qx,qy,qz,qw\n")
            stream.write(f"{map_id},0,{i+1}.0,1,0,0,0,0,0,1\n")
    writer.close()
    return path


def test_storage_preserves_depth_and_rejects_path_traversal(tmp_path):
    from experiments.handheld_mapping.storage import RunWriter
    run = make_run(tmp_path / "run")
    depth = cv2.imread(str(run / "raw/depth/1000000000.png"), cv2.IMREAD_UNCHANGED)
    assert depth.dtype == np.uint16
    assert int(depth[0, 0]) == 2000
    with pytest.raises(FileExistsError):
        RunWriter(run)
    writer = RunWriter(tmp_path / "other")
    with pytest.raises(ValueError):
        writer.accept({"kind": "artifact", "name": "../../bad"}, b"bad")
    writer.close()


def test_tsdf_transform_direction_and_map_separation(tmp_path):
    from experiments.handheld_mapping.reconstruct import reconstruct
    import open3d as o3d
    run = make_run(tmp_path / "run", (0, 7))
    result = reconstruct(run, voxel_size=.025)
    assert result["status"] == "complete"
    assert len(result["maps"]) == 2
    for item in result["maps"]:
        cloud = o3d.io.read_point_cloud(str(run / item["cloud"]))
        points = np.asarray(cloud.points)
        assert len(points) > 500
        assert abs(np.median(points[:, 2]) - 2) < .025
        assert abs(np.median(points[:, 0]) - 1.2) < .05
        assert item["integrated_frames"] == 1
    assert (run / "dense/map_0_mesh.ply").stat().st_size > 100
    assert (run / "dense/map_7_mesh.ply").stat().st_size > 100


def test_tsdf_streams_indexes_instead_of_loading_full_recording(tmp_path, monkeypatch):
    from pathlib import Path
    from experiments.handheld_mapping.reconstruct import reconstruct
    run = make_run(tmp_path/'run')
    original = Path.read_text
    def checked(path, *args, **kwargs):
        if path.name == 'frames.jsonl':
            raise AssertionError('full frame index read')
        return original(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'read_text', checked)
    assert reconstruct(run)['status'] == 'complete'


@pytest.mark.parametrize("missing", ["T_left_depth", "depth_scale_m"])
def test_missing_calibration_refuses_integration(tmp_path, missing):
    from experiments.handheld_mapping.reconstruct import reconstruct
    run = make_run(tmp_path / "run")
    info = json.loads((run / "capture_calibration.json").read_text())
    info["depth"].pop(missing)
    (run / "capture_calibration.json").write_text(json.dumps(info))
    with pytest.raises(ValueError):
        reconstruct(run)
    assert not (run / "dense/summary.json").exists()


def test_unsynchronized_depth_never_builds_a_success_map(tmp_path):
    from experiments.handheld_mapping.reconstruct import reconstruct
    run = make_run(tmp_path / "run")
    path = run / "orb/map_0_keyframes.csv"
    path.write_text(path.read_text().replace("1.0,1,", "1.1,1,"))
    with pytest.raises(ValueError, match="matching|일치"):
        reconstruct(run)


def test_missing_depth_for_some_keyframes_reports_partial(tmp_path):
    from experiments.handheld_mapping.reconstruct import reconstruct
    run = make_run(tmp_path / "run")
    path = run / "orb/map_0_keyframes.csv"
    with path.open("a") as stream:
        stream.write("0,1,2.0,1,0,0,0,0,0,1\n")
    result = reconstruct(run, voxel_size=.025)
    assert result["status"] == "partial"
    assert result["maps"][0]["integrated_frames"] == 1
    assert result["maps"][0]["unintegrated_keyframes"] == 1


def test_invalid_depth_does_not_report_complete(tmp_path):
    from experiments.handheld_mapping.reconstruct import reconstruct
    run = make_run(tmp_path / "run")
    cv2.imwrite(str(run / "raw/depth/1000000000.png"), np.zeros((60, 80), np.uint16))
    with pytest.raises(ValueError):
        reconstruct(run)


def test_tsdf_rotated_camera_uses_world_to_camera_extrinsic(tmp_path):
    from experiments.handheld_mapping.reconstruct import reconstruct
    import open3d as o3d
    run = make_run(tmp_path / "run")
    path = run / "orb/map_0_keyframes.csv"
    path.write_text("map_id,keyframe_id,source_stamp,x,y,z,qx,qy,qz,qw\n"
                    "0,0,1.0,1,0,0,0,0.7071067811865476,0,0.7071067811865476\n")
    reconstruct(run, voxel_size=.025)
    points = np.asarray(o3d.io.read_point_cloud(str(run / "dense/map_0_cloud.ply")).points)
    assert abs(np.median(points[:, 0])-3) < .025
    assert abs(np.median(points[:, 2])+.2) < .05


def test_line_tsdf_uses_line_poses_and_preserves_point_result(tmp_path):
    from experiments.handheld_mapping.reconstruct import reconstruct
    import open3d as o3d
    run = make_run(tmp_path / "run")
    reconstruct(run, voxel_size=.025)
    original_mesh = (run / "dense/map_0_mesh.ply").read_bytes()
    (run / "orbline").mkdir()
    (run / "orbline/map_0_keyframes.csv").write_text(
        "map_id,keyframe_id,source_stamp,x,y,z,qx,qy,qz,qw\n"
        "0,0,1.0,3,0,0,0,0,0,1\n")
    result = reconstruct(run, voxel_size=.025, backend="orbline")
    assert result["maps"][0]["cloud"] == "dense_orbline/map_0_cloud.ply"
    points = np.asarray(o3d.io.read_point_cloud(str(run / result["maps"][0]["cloud"])).points)
    assert abs(np.median(points[:, 0])-3.2) < .05
    assert (run / "dense/map_0_mesh.ply").read_bytes() == original_mesh


def test_big_endian_depth_and_stride_are_preserved():
    from experiments.handheld_mapping.storage import decode_image
    image = np.array([[1000, 2000, 0], [3000, 4000, 0]], dtype=">u2")
    header = {"width": 2, "height": 2, "step": 6, "encoding": "16UC1", "is_bigendian": True,
              "compression": "zlib"}
    decoded = decode_image(header, zlib.compress(image.tobytes()))
    assert decoded.tolist() == [[1000, 2000], [3000, 4000]]


def test_compressed_image_cannot_expand_past_declared_size():
    from experiments.handheld_mapping.storage import decode_image
    header = {"width": 2, "height": 2, "step": 2, "encoding": "mono8", "compression": "zlib"}
    with pytest.raises(ValueError):
        decode_image(header, zlib.compress(b"a"*1000000))


def test_inertial_tsdf_requires_initialized_export_and_uses_separate_output(tmp_path):
    from experiments.handheld_mapping.reconstruct import reconstruct
    from experiments.handheld_mapping.viewer import select_maps
    run=make_run(tmp_path/'run')
    (run/'orb').rename(run/'orb_inertial')
    (run/'orb_inertial/summary.json').write_text(json.dumps({'orb_fused':True,'exported_imu_initialized_maps':[0]}))
    result=reconstruct(run, voxel_size=.025, backend='orb_inertial')
    assert result['maps'][0]['mesh']=='dense_inertial/map_0_mesh.ply'
    assert result['coordinate_system']=='ORB stereo-inertial map; gravity estimate unverified'
    assert result['imu_clock_verified'] is False
    assert not (run/'dense/summary.json').exists()
    assert select_maps(run,backend='orb_inertial')[0][1]==run/'dense_inertial/map_0_mesh.ply'


def test_uninitialized_inertial_map_is_not_presented_as_fused_tsdf(tmp_path):
    from experiments.handheld_mapping.reconstruct import reconstruct
    run=make_run(tmp_path/'run')
    (run/'orb').rename(run/'orb_inertial')
    (run/'orb_inertial/summary.json').write_text(json.dumps({'orb_fused':False,'exported_imu_initialized_maps':[]}))
    with pytest.raises(ValueError,match='initialized'):
        reconstruct(run,backend='orb_inertial')
