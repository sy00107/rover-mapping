# ================================================================
# test_live_session.py
# 설명: 촬영 중 관측 지도와 종료 후 최종 지도·실시간 실패 시 원본 보존을 검증한다.
# 사용법: QT_QPA_PLATFORM=offscreen .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_live_session.py -q
# ================================================================
import time
from pathlib import Path
import pytest

from test_session import wait_state


def test_demo_live_updates_while_recording_and_final_tsdf_is_preserved(tmp_path):
    from experiments.handheld_mapping.controller import Session
    session = Session()
    session.start({'output_root':str(tmp_path),'demo':True,'live_mapping':True})
    try:
        wait_state(session,{'recording'})
        deadline = time.monotonic()+3
        while time.monotonic()<deadline:
            snapshot = session.live_snapshot()
            if snapshot and snapshot['processed']>0: break
            time.sleep(.02)
        assert snapshot['synthetic'] is True
        assert snapshot['observed_cells']>0
        assert session.snapshot()['state']=='recording'
    finally:
        session.stop()
        session.thread.join(timeout=20)
    result=session.snapshot()
    assert result['state']=='complete',result
    assert (Path(result['run_dir'])/'dense/map_0_mesh.ply').is_file()
    assert (Path(result['run_dir'])/'orb_live/coverage_map_0.npz').is_file()


def test_live_start_failure_does_not_discard_raw_or_prevent_final_map(tmp_path,monkeypatch):
    from experiments.handheld_mapping.controller import Session
    from experiments.handheld_mapping import live
    def fail(*args,**kwargs): raise RuntimeError('live startup fixture failure')
    monkeypatch.setattr(live,'LiveMapper',fail)
    session=Session()
    session.start({'output_root':str(tmp_path),'demo':True,'live_mapping':True})
    wait_state(session,{'recording'})
    time.sleep(.2)
    session.stop(); session.thread.join(timeout=20)
    result=session.snapshot()
    assert result['state']=='complete',result
    assert 'live startup fixture failure' in result['live_error']
    assert list((Path(result['run_dir'])/'raw/depth').glob('*.png'))


@pytest.mark.parametrize('native_available', [False, True])
def test_coverage_widget_and_live_option_render_without_camera(tmp_path, monkeypatch, native_available):
    from PyQt5.QtWidgets import QApplication
    from experiments.handheld_mapping.app import MappingWindow
    from experiments.handheld_mapping.controller import Session
    from experiments.handheld_mapping.coverage import CoverageGrid
    from experiments.handheld_mapping import app as mapping_app
    monkeypatch.setattr(mapping_app, 'ROOT', tmp_path)
    if native_available:
        binary = tmp_path/'outputs/handheld_mapping/orb3_comparison_build/rover_orb3_compare'
        binary.parent.mkdir(parents=True)
        binary.touch()
    app=QApplication.instance() or QApplication([])
    window=MappingWindow(Session(),demo=True)
    assert window.point_ready is native_available
    grid=CoverageGrid()
    snapshot=grid.snapshot()
    snapshot.update(tracking_state=2, stale=False,error=None,processed=1,lag_ms=12.,dropped_pairs=0,
                    synthetic=True,finished=False)
    window.coverage.set_snapshot(snapshot)
    window.show(); app.processEvents()
    assert not window.coverage.grab().isNull()
    snapshot.update(error='native stopped',stale=True)
    window.coverage.set_snapshot(snapshot); app.processEvents()
    assert not window.coverage.grab().isNull()
    window.close()


def test_live_finish_error_does_not_prevent_final_map(tmp_path,monkeypatch):
    from experiments.handheld_mapping.controller import Session
    from experiments.handheld_mapping.live import LiveMapper
    original=LiveMapper.finish
    def fail(self):
        original(self)
        raise OSError('live summary disk fixture failure')
    monkeypatch.setattr(LiveMapper,'finish',fail)
    session=Session()
    session.start({'output_root':str(tmp_path),'demo':True,'live_mapping':True})
    wait_state(session,{'recording'})
    time.sleep(.2)
    session.stop(); session.thread.join(timeout=20)
    result=session.snapshot()
    assert result['state']=='complete',result
    assert 'live summary disk fixture failure' in result['live_error']


def test_runtime_live_failure_keeps_recording_and_final_map(tmp_path,monkeypatch):
    import sys
    from experiments.handheld_mapping import live
    from experiments.handheld_mapping.controller import Session
    original=live.LiveMapper
    def factory(run,**kwargs):
        command=[sys.executable,'-c','import sys; sys.stdin.readline(); sys.exit(3)']
        return original(run,command=command,result_timeout=.2)
    monkeypatch.setattr(live,'LiveMapper',factory)
    session=Session()
    session.start({'output_root':str(tmp_path),'demo':True,'live_mapping':True})
    wait_state(session,{'recording'})
    deadline=time.monotonic()+3
    while not session.live_snapshot()['error'] and time.monotonic()<deadline:time.sleep(.01)
    assert session.live_snapshot()['error']
    assert session.snapshot()['state']=='recording'
    session.stop(); session.thread.join(timeout=20)
    result=session.snapshot()
    assert result['state']=='complete',result
    assert result['live']['error']
    assert (Path(result['run_dir'])/'dense/map_0_mesh.ply').is_file()


def test_original_and_overlay_are_both_visible_in_vertical_order():
    from PyQt5.QtWidgets import QApplication
    from experiments.handheld_mapping.app import MappingWindow
    from experiments.handheld_mapping.controller import Session
    app=QApplication.instance() or QApplication([])
    window=MappingWindow(Session(),demo=True)
    window.show(); app.processEvents()
    assert window.preview.isVisible() and window.overlay_preview.isVisible()
    assert window.preview.geometry().bottom() < window.overlay_preview.geometry().top()
    window.close()


def test_overlay_frame_pairing_stale_fallback_and_empty_session_clear():
    import numpy as np
    from types import SimpleNamespace
    from PyQt5.QtWidgets import QApplication
    from experiments.handheld_mapping.app import MappingWindow
    from experiments.handheld_mapping.controller import Session
    from experiments.handheld_mapping.coverage import CoverageGrid
    app=QApplication.instance() or QApplication([])
    session=Session(); session.preview=np.full((8,8),100,np.uint8)
    snapshot=CoverageGrid().snapshot()
    snapshot.update(error=None,stale=False,tracking_state=2,processed=1,finished=False,
                    overlay_original=np.full((8,8),50,np.uint8),overlay=np.full((8,8,3),[53,175,133],np.uint8))
    session.live=SimpleNamespace(snapshot=lambda:snapshot)
    window=MappingWindow(session); window.show(); app.processEvents(); window.refresh()
    def center(label):
        image=label.pixmap().toImage()
        return image.pixelColor(image.width()//2,image.height()//2).getRgb()[:3]
    assert center(window.preview)==(50,50,50)
    assert center(window.overlay_preview)==(53,175,133)
    snapshot['stale']=True; window.refresh()
    assert center(window.preview)==(100,100,100)
    assert center(window.overlay_preview)==(100,100,100)
    session.live=None; session.preview=None; window.refresh()
    assert window.preview.pixmap() is None or window.preview.pixmap().isNull()
    assert window.overlay_preview.pixmap() is None or window.overlay_preview.pixmap().isNull()
    window.close()


def test_recovery_message_retains_previous_coverage_and_uses_current_camera():
    import numpy as np
    from types import SimpleNamespace
    from PyQt5.QtWidgets import QApplication
    from experiments.handheld_mapping.app import MappingWindow
    from experiments.handheld_mapping.controller import Session
    from experiments.handheld_mapping.coverage import CoverageGrid
    app = QApplication.instance() or QApplication([])
    session = Session()
    session.preview = np.full((8,8),100,np.uint8)
    grid = CoverageGrid()
    grid.update_pose(0,np.eye(4))
    snapshot = grid.snapshot()
    snapshot.update(error=None,stale=False,tracking_state=3,processed=12,finished=False,
                    overlay_original=np.full((8,8),50,np.uint8),
                    overlay=np.full((8,8,3),[53,175,133],np.uint8))
    session.live = SimpleNamespace(snapshot=lambda:snapshot)
    session.update(capture_mode='sensors_only')
    window = MappingWindow(session)
    try:
        window.show()
        app.processEvents()
        window.refresh()
        assert '기존 지도 복구 중 · 정지해서 이전 장면 보기' in window.metrics.text()
        assert '기존 지도 복구 중' in window.overlay_note.text()
        assert window.coverage.state['map_id'] == 0
        assert window.coverage.state['path'] == [(0.,0.)]
        image = window.preview.pixmap().toImage()
        assert image.pixelColor(image.width()//2,image.height()//2).red() == 100
    finally:
        window.close()
