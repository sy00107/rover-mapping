import io
# ================================================================
# test_imu_session.py
# 설명: IMU 수집 경계와 실제 카메라 subprocess로 세션 기록·종료·Qt 설정을 검증한다.
# 사용법: QT_QPA_PLATFORM=offscreen .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_imu_session.py -q
# ================================================================
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import pytest

from experiments.handheld_mapping.controller import Session
from test_session import wait_state


@pytest.fixture
def imu_boundary(monkeypatch):
    class Capture:
        """하드웨어 연결 경계에서 준비·오류·정지 결과를 제공한다."""

        start_error = None
        stop_error = None
        instances = []

        def __init__(self, host: str, run: Path, calibration_path: Path, urdf_path: Path):
            self.run = run
            self.state = dict(state="recording", recorded=12, valid=12, unique=10,
                              duplicates=2, hz=98., stale=False, error=None,
                              gyro_dps=[0., 0., 0.], acc_g=[0., 0., 1.], acc_norm_g=1.,
                              roll_deg=0., pitch_deg=0., imu_clock_verified=False, orb_fused=False)
            self.instances.append(self)

        def start(self, timeout: float = 8.) -> None:
            if self.start_error:
                raise RuntimeError(self.start_error)

        def snapshot(self) -> dict:
            return dict(self.state)

        def stop(self) -> dict:
            self.state["state"] = "stopped"
            (self.run / "imu_summary.json").write_text(json.dumps(self.state))
            if self.stop_error:
                raise RuntimeError(self.stop_error)
            return self.snapshot()

    monkeypatch.setitem(sys.modules, "experiments.handheld_mapping.imu", SimpleNamespace(ImuCapture=Capture))
    return Capture


def config(tmp_path: Path) -> dict:
    return {"output_root": str(tmp_path), "imu_enabled": True,
            "imu_calibration": str(tmp_path / "calibration.json"),
            "worker_command": [sys.executable, "-u", "-m", "experiments.handheld_mapping.demo"]}


def test_imu_not_ready_refuses_camera_and_preserves_initial_error(tmp_path, imu_boundary):
    imu_boundary.start_error = "fresh IMU unavailable"
    imu_boundary.stop_error = "cleanup fixture failure"
    session = Session()
    session.start(config(tmp_path))
    session.thread.join(timeout=1)
    result = session.snapshot()
    if session.thread.is_alive():
        session.stop()
        session.thread.join(timeout=20)
    assert result["state"] == "failed", result
    assert result["error"] == "fresh IMU unavailable"
    assert "cleanup fixture failure" in result["imu_cleanup_error"]
    assert session.process is None
    assert json.loads((Path(result["run_dir"]) / "session.json").read_text())["state"] == "failed"


def test_imu_stops_in_same_run_before_reconstruction_and_snapshot_is_fresh(tmp_path, imu_boundary, monkeypatch):
    from experiments.handheld_mapping import reconstruct
    original = reconstruct.reconstruct

    def reconstruct_after_imu(run: Path, **kwargs) -> dict:
        assert json.loads((run / "imu_summary.json").read_text())["state"] == "stopped"
        return original(run, **kwargs)

    monkeypatch.setattr(reconstruct, "reconstruct", reconstruct_after_imu)
    session = Session()
    session.start(config(tmp_path))
    try:
        wait_state(session, {"recording"})
        imu_boundary.instances[-1].state["unique"] = 42
        assert session.snapshot()["imu"]["unique"] == 42
        time.sleep(.2)
    finally:
        session.stop()
        session.thread.join(timeout=20)
    result = session.snapshot()
    assert result["state"] == "complete", result
    saved = json.loads((Path(result["run_dir"]) / "session.json").read_text())
    assert saved["imu"]["unique"] == 42
    assert saved["imu"]["state"] == "stopped"
    assert saved["imu"]["orb_fused"] is False
    assert saved["imu"]["imu_clock_verified"] is False


def test_runtime_imu_error_fails_capture_but_keeps_camera_originals(tmp_path, imu_boundary):
    session = Session()
    session.start(config(tmp_path))
    try:
        wait_state(session, {"recording"})
        time.sleep(.2)
        imu_boundary.instances[-1].state["error"] = "IMU transport closed"
        wait_state(session, {"failed"}, timeout=5)
    finally:
        session.stop()
        session.thread.join(timeout=10)
    result = session.snapshot()
    assert "IMU transport closed" in result["error"]
    assert session.process.poll() is not None
    run = Path(result["run_dir"])
    assert list((run / "raw/left").glob("*.png"))
    assert (run / "imu_summary.json").is_file()


def test_camera_launch_failure_stops_prepared_imu(tmp_path, imu_boundary):
    session = Session()
    session.start({**config(tmp_path), "worker_command": [str(tmp_path / "missing-camera-worker")]})
    session.thread.join(timeout=5)
    result = session.snapshot()
    assert result["state"] == "failed", result
    assert "missing-camera-worker" in result["error"]
    saved = json.loads((Path(result["run_dir"]) / "session.json").read_text())
    assert saved["imu"]["state"] == "stopped"
    assert (Path(result["run_dir"]) / "imu_summary.json").is_file()


def test_stale_imu_produces_partial_result_and_demo_never_connects(tmp_path, imu_boundary):
    session = Session()
    session.start(config(tmp_path))
    try:
        wait_state(session, {"recording"})
        imu_boundary.instances[-1].state["stale"] = True
        time.sleep(.2)
    finally:
        session.stop()
        session.thread.join(timeout=20)
    assert session.snapshot()["state"] == "partial", session.snapshot()
    imu_boundary.start_error = "demo must not connect"
    session.start({**config(tmp_path), "demo": True})
    try:
        wait_state(session, {"recording", "failed"})
        assert "imu" not in session.snapshot()
        time.sleep(.2)
    finally:
        session.stop()
        session.thread.join(timeout=20)
    assert session.snapshot()["state"] == "complete", session.snapshot()


def test_qt_imu_defaults_demo_toggle_and_recording_status(tmp_path, monkeypatch):
    from PyQt5.QtWidgets import QApplication
    from experiments.handheld_mapping import app as mapping_app
    calibration = tmp_path / "outputs/handheld_mapping/calibration/imu_calibration.json"
    calibration.parent.mkdir(parents=True)
    calibration.write_text("{}")
    monkeypatch.setattr(mapping_app, "ROOT", tmp_path)
    app = QApplication.instance() or QApplication([])
    session = Session()
    window = mapping_app.MappingWindow(session)
    try:
        assert window.imu_enabled.isChecked()
        assert window.imu_calibration.text() == str(calibration)
        window.demo.setChecked(True)
        window.refresh()
        assert not window.imu_enabled.isChecked()
        assert not window.imu_enabled.isEnabled()
        window.demo.setChecked(False)
        window.imu_enabled.setChecked(True)
        session.update(state="recording", imu=dict(valid=24, unique=20, hz=100., stale=True,
                                                   roll_deg=2., pitch_deg=-1., orb_fused=False))
        window.refresh()
        assert not window.imu_enabled.isEnabled()
        assert not window.imu_calibration.isEnabled()
        assert "20" in window.imu_status.text()
        assert "100.0" in window.imu_status.text()
        assert "지연" in window.imu_status.text()
        assert "ORB 융합 안 함" in window.imu_status.text()
        app.processEvents()
    finally:
        session.update(state="idle")
        window.close()


def test_qt_start_records_enabled_imu_in_camera_session(tmp_path, imu_boundary, monkeypatch):
    from PyQt5.QtWidgets import QApplication
    from experiments.handheld_mapping.app import MappingWindow
    app = QApplication.instance() or QApplication([])
    session = Session()
    original_start = session.start

    def local_camera(settings: dict) -> None:
        original_start({**settings, "worker_command": config(tmp_path)["worker_command"]})

    monkeypatch.setattr(session, "start", local_camera)
    window = MappingWindow(session, output_root=tmp_path)
    window.imu_enabled.setChecked(True)
    window.imu_calibration.setText(str(tmp_path / "calibration.json"))
    # 이 시험은 IMU 동시 기록만 격리한다. 정지 기준/장착 데이터는 제공하지 않는다.
    window.gravity_enabled.setChecked(False)
    try:
        window.start_button.click()
        wait_state(session, {"recording"})
        assert session.snapshot()["imu"]["valid"] == 12
        time.sleep(.2)
    finally:
        session.stop()
        session.thread.join(timeout=20)
        window.close()
        app.processEvents()
    result = session.snapshot()
    assert result["state"] == "complete", result
    assert json.loads((Path(result["run_dir"]) / "session.json").read_text())["imu"]["orb_fused"] is False


def test_requested_gravity_without_reference_keeps_raw_and_reports_partial(tmp_path,imu_boundary):
    session=Session()
    session.start({**config(tmp_path),'gravity_enabled':True})
    try:
        wait_state(session,{'recording'})
        time.sleep(.2)
    finally:
        session.stop(); session.thread.join(timeout=20)
    result=session.snapshot()
    assert result['state']=='partial',result
    assert result['gravity']['status']=='unavailable'
    assert result['terrain']['status']=='unavailable'
    assert result['dense']['maps'][0]['coordinate_system']=='initial_camera_optical'
    assert list((Path(result['run_dir'])/'raw/depth').glob('*.png'))


def test_gravity_requires_imu_before_camera_launch(tmp_path):
    session=Session()
    session.start({'output_root':str(tmp_path),'gravity_enabled':True,'imu_enabled':False})
    session.thread.join(timeout=5)
    assert session.snapshot()['state']=='failed'
    assert session.process is None
    assert 'IMU' in session.snapshot()['message']


def test_real_imu_capture_and_camera_share_run_without_clock_or_fusion_claims(tmp_path, monkeypatch):
    from experiments.handheld_mapping import imu
    calibration = {"version": 1, "sensor": "ISM330DHCX", "frame": "rover_body",
                   "chip_to_body": [[0, -1], [1, -1], [2, 1]],
                   "acc_unit": "g", "gyro_unit": "deg/s", "gyro_bias_dps": [.5, 0., 0.],
                   "acc_offset_g": [0., 0., 0.], "acc_scale": [1., 1., 1.]}
    (tmp_path / "calibration.json").write_text(json.dumps(calibration))
    producer = """import json,sys,time,threading
stop=threading.Event()
threading.Thread(target=lambda:(sys.stdin.readline(),stop.set()),daemon=True).start()
print(json.dumps({'kind':'ready','chip_to_body':[[0,-1],[1,-1],[2,1]]}),flush=True)
n=0
while not stop.is_set():
 print(json.dumps({'kind':'telemetry','schema':1,'host_receipt_ns':time.time_ns(),
  'host_monotonic_ns':time.monotonic_ns(),'tee_drops':0,
  'telemetry':{'flags':16,'tick':n,'t_ms':1000+n*20,'gyro_dps':[1.5,0.,0.],
  'acc_g':[0.,0.,1.],'imu_raw':[1,2,3,4,5,6],'imu_n':4}}),flush=True)
 n+=1
 stop.wait(.02)
"""
    original = imu.ImuCapture

    def local_imu(host: str, run: Path, calibration_path: Path, urdf_path: Path):
        return original(host, run, calibration_path, urdf_path,
                        command=[sys.executable, "-u", "-c", producer])

    monkeypatch.setattr(imu, "ImuCapture", local_imu)
    session = Session()
    session.start(config(tmp_path))
    try:
        wait_state(session, {"recording", "failed"})
        assert session.snapshot()["state"] == "recording", session.snapshot()
        time.sleep(.3)
    finally:
        session.stop()
        session.thread.join(timeout=20)
    result = session.snapshot()
    assert result["state"] == "complete", result
    run = Path(result["run_dir"])
    rows = [json.loads(line) for line in (run / "imu.jsonl").read_text().splitlines()]
    assert rows[-1]["raw"]["telemetry"]["gyro_dps"] == [1.5, 0., 0.]
    assert rows[-1]["calibrated"]["gyro_dps"] == [1., 0., 0.]
    manifest = json.loads((run / "imu_manifest.json").read_text())
    saved = json.loads((run / "session.json").read_text())
    assert manifest["run_id"] == run.name == saved["imu"]["run_id"]
    assert saved["imu"]["unique"] > 2
    assert saved["imu"]["orb_fused"] is False
    assert saved["imu"]["imu_clock_verified"] is False
    assert (run / "imu_calibration.json").read_bytes() == (tmp_path / "calibration.json").read_bytes()
    assert (run / "robot.urdf").is_file()
    assert list((run / "raw/left").glob("*.png"))


@pytest.fixture
def fusion_boundary(monkeypatch, imu_boundary):
    from experiments.handheld_mapping import controller
    from experiments.handheld_mapping.protocol import write_packet
    stream = io.BytesIO()
    write_packet(stream, {"kind": "ready"})
    write_packet(stream, {"kind": "finished", "status": "saved", "dropped": 0})

    def camera_process(*args, **kwargs):
        return SimpleNamespace(stdin=io.BytesIO(), stdout=io.BytesIO(stream.getvalue()),
                               poll=lambda: 0, wait=lambda **options: 0)

    def reconstruct(run: Path, backend: str = "orb", **kwargs) -> dict:
        folder = run / ("dense_inertial" if backend == "orb_inertial" else "dense")
        folder.mkdir(exist_ok=True)
        (folder / "surface.ply").write_text(backend)
        return {"status": "complete", "backend": backend}

    monkeypatch.setattr(controller.subprocess, "Popen", camera_process)
    monkeypatch.setitem(sys.modules, "experiments.handheld_mapping.reconstruct", SimpleNamespace(reconstruct=reconstruct))
    return reconstruct


def test_inertial_fusion_default_preserves_baseline_without_calling_helper(tmp_path, fusion_boundary, monkeypatch):
    def forbidden(run: Path) -> dict:
        raise AssertionError("default must not run inertial fusion")

    monkeypatch.setitem(sys.modules, "experiments.handheld_mapping.inertial", SimpleNamespace(run_inertial_auto=forbidden))
    session = Session()
    session.start(config(tmp_path))
    session.thread.join(timeout=5)
    result = session.snapshot()
    assert result["state"] == "complete", result
    assert result["dense"]["backend"] == "orb"
    assert "inertial_fusion" not in result


def test_inertial_opt_in_enables_recording_then_saves_separate_dense(tmp_path, fusion_boundary, imu_boundary, monkeypatch):
    def fuse_after_baseline(run: Path) -> dict:
        assert (run / "imu_summary.json").is_file()
        assert (run / "dense/surface.ply").read_text() == "orb"
        return {"status": "initialized", "orb_fused": True}

    monkeypatch.setitem(sys.modules, "experiments.handheld_mapping.inertial", SimpleNamespace(run_inertial_auto=fuse_after_baseline))
    session = Session()
    session.start({**config(tmp_path), "imu_enabled": False, "inertial_fusion": True})
    session.thread.join(timeout=5)
    result = session.snapshot()
    assert result["state"] == "complete", result
    assert len(imu_boundary.instances) == 1
    assert result["inertial_fusion"]["orb_fused"] is True
    assert result["dense"]["backend"] == "orb"
    assert result["inertial_dense"]["backend"] == "orb_inertial"
    run = Path(result["run_dir"])
    assert (run / "dense/surface.ply").read_text() == "orb"
    assert (run / "dense_inertial/surface.ply").read_text() == "orb_inertial"


@pytest.mark.parametrize("failure", ["not_initialized", "runner_failed", "dense_failed"])
def test_inertial_postprocessing_failure_keeps_baseline(tmp_path, fusion_boundary, monkeypatch, failure):
    def fuse(run: Path) -> dict:
        if failure == "runner_failed":
            raise RuntimeError("inertial runner unavailable")
        return {"status": "not_initialized" if failure == "not_initialized" else "initialized",
                "orb_fused": failure != "not_initialized"}

    def reconstruct(run: Path, backend: str = "orb", **kwargs) -> dict:
        if backend == "orb_inertial":
            raise RuntimeError("inertial depth unavailable")
        return fusion_boundary(run, backend, **kwargs)

    monkeypatch.setitem(sys.modules, "experiments.handheld_mapping.inertial", SimpleNamespace(run_inertial_auto=fuse))
    monkeypatch.setitem(sys.modules, "experiments.handheld_mapping.reconstruct", SimpleNamespace(reconstruct=reconstruct))
    session = Session()
    session.start({**config(tmp_path), "inertial_fusion": True})
    session.thread.join(timeout=5)
    result = session.snapshot()
    assert result["state"] == "partial", result
    assert result["dense"]["status"] == "complete"
    assert result["inertial_warning"]
    assert "inertial_dense" not in result
    assert (Path(result["run_dir"]) / "dense/surface.ply").read_text() == "orb"
