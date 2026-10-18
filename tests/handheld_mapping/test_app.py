import pytest
# ================================================================
# test_app.py
# 설명: 실제 Qt 버튼으로 모의 촬영을 시작·종료하고 저장 중 중복 입력을 막는지 검사한다.
# 사용법: QT_QPA_PLATFORM=offscreen .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_app.py -q
# ================================================================
import time


def pump(app, condition, seconds=30):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        app.processEvents()
        if condition():
            return
        time.sleep(.02)
    raise AssertionError("Qt condition timed out")


def test_buttons_drive_capture_and_save(tmp_path):
    from PyQt5.QtWidgets import QApplication
    from experiments.handheld_mapping.app import MappingWindow
    from experiments.handheld_mapping.controller import Session
    app = QApplication.instance() or QApplication([])
    session = Session()
    window = MappingWindow(session, output_root=tmp_path, demo=True)
    window.show()
    window.start_button.click()
    window.refresh()
    assert not window.start_button.isEnabled()
    pump(app, lambda: session.snapshot()["state"] == "recording")
    window.refresh()
    assert window.stop_button.isEnabled()
    time.sleep(.3)
    window.stop_button.click()
    window.refresh()
    assert not window.start_button.isEnabled()
    pump(app, lambda: session.snapshot()["state"] in {"failed", "complete", "partial"})
    assert session.snapshot()["state"] == "complete", session.snapshot()
    pump(app, lambda: not session.thread.is_alive())
    window.refresh()
    assert window.open_button.isEnabled()
    assert window.start_button.isEnabled()
    window.close()


def test_close_while_recording_waits_for_save(tmp_path):
    from PyQt5.QtWidgets import QApplication
    from experiments.handheld_mapping.app import MappingWindow
    from experiments.handheld_mapping.controller import Session
    app = QApplication.instance() or QApplication([])
    session = Session()
    window = MappingWindow(session, output_root=tmp_path, demo=True)
    window.show()
    window.start_button.click()
    pump(app, lambda: session.snapshot()["state"] == "recording")
    time.sleep(.2)
    window.close()
    assert window.isVisible()
    pump(app, lambda: not window.isVisible())
    assert session.snapshot()["state"] == "complete", session.snapshot()


def test_capture_policy_always_records_imu_and_settings_start_collapsed(tmp_path, monkeypatch):
    from PyQt5.QtWidgets import QApplication, QCheckBox
    from experiments.handheld_mapping.app import MappingWindow
    from experiments.handheld_mapping.controller import Session
    app = QApplication.instance() or QApplication([])
    session = Session()
    settings = []
    monkeypatch.setattr(session, "start", settings.append)
    window = MappingWindow(session, output_root=tmp_path)
    try:
        window.show()
        app.processEvents()
        assert not window.settings_panel.isVisible()
        assert not any(box.isVisible() for box in window.findChildren(QCheckBox))
        window.start()
        assert settings[-1]["imu_enabled"] is True
        assert settings[-1]["gravity_enabled"] is True
        assert settings[-1]["sensors_only"] is True
        assert settings[-1]["compare_lines"] is False
        assert settings[-1]["inertial_fusion"] is False
        window.settings_button.click()
        assert window.settings_panel.isVisible()
        session.update(state="recording")
        window.refresh()
        assert not window.imu_calibration.isEnabled()
        assert not window.host.isEnabled()
        window.settings_button.click()
        assert not window.settings_panel.isVisible()
    finally:
        session.update(state="idle")
        window.close()


def test_cli_fusion_flag_reaches_window_without_launching_capture(monkeypatch):
    from experiments.handheld_mapping import app as mapping_app
    settings = []

    def window_after_parse(*args):
        settings.append(args)
        raise RuntimeError("arguments parsed")

    monkeypatch.setattr(mapping_app.sys, "argv", ["handheld_mapping", "--imu-fusion"])
    monkeypatch.setattr(mapping_app, "QApplication", lambda args: None)
    monkeypatch.setattr(mapping_app, "MappingWindow", window_after_parse)
    with pytest.raises(RuntimeError, match="arguments parsed"):
        mapping_app.main()
    assert settings[0][4] is True



def test_window_fusion_argument_enables_imu_recording(tmp_path):
    from PyQt5.QtWidgets import QApplication
    from experiments.handheld_mapping.app import MappingWindow
    from experiments.handheld_mapping.controller import Session
    app = QApplication.instance() or QApplication([])
    window = MappingWindow(Session(), output_root=tmp_path, inertial_fusion=True)
    try:
        assert window.inertial_fusion is True
        assert window.demo is False
        app.processEvents()
    finally:
        window.close()


@pytest.fixture
def saved_viewer_window(tmp_path, monkeypatch):
    from PyQt5.QtWidgets import QApplication
    from experiments.handheld_mapping import app as mapping_app
    from experiments.handheld_mapping.controller import Session
    from test_viewer import maps
    application = QApplication.instance() or QApplication([])
    run = maps(tmp_path/'run')
    maps(run, 'dense_inertial')
    session = Session()
    session.update(run_dir=str(run))
    window = mapping_app.MappingWindow(session, output_root=tmp_path, demo=True)
    launched = []
    monkeypatch.setattr(mapping_app.subprocess, 'Popen', lambda command, **kwargs: launched.append(command))
    try:
        yield window, launched, run
    finally:
        window.close()
        application.processEvents()


def test_view_button_defaults_to_camera_2m_clean_without_building_in_qt(saved_viewer_window):
    window, launched, run = saved_viewer_window
    assert window.view_backend.currentData() == 'auto'
    assert window.view_depth.currentData() == 2.
    assert window.view_cleanup.isChecked()
    assert not window.view_measurements.isChecked()
    window.view_button.click()
    command, = launched
    assert command[command.index('--backend')+1] == 'orb_offline'
    assert command[command.index('--depth-max')+1] == '2'
    assert '--surface-cleanup' in command
    assert '--no-path-measurements' in command
    assert '첫 생성' in window.session.snapshot()['message']
    assert not (run/'depth_variants').exists()


@pytest.mark.parametrize('enabled',[False,True])
def test_distance_angle_toggle_keeps_path_and_2m_clean_map(saved_viewer_window,enabled):
    window,launched,_=saved_viewer_window
    window.view_measurements.setChecked(enabled)
    window.view_button.click()
    command,=launched
    assert ('--no-path-measurements' in command) == (not enabled)
    assert '--no-trajectory' not in command
    assert command[command.index('--depth-max')+1]=='2'
    assert '--surface-cleanup' in command
    assert ('거리·각도 표시' if enabled else '거리·각도 숨김') in window.session.snapshot()['message']


@pytest.mark.parametrize('depth', [1., 2., 3., None])
@pytest.mark.parametrize('cleanup', [False, True])
def test_view_button_honours_distance_and_cleanup_selection(saved_viewer_window, depth, cleanup):
    window, launched, _ = saved_viewer_window
    window.view_depth.setCurrentIndex(window.view_depth.findData(depth))
    window.view_cleanup.setChecked(cleanup)
    assert window.view_cleanup.isEnabled() == (depth is not None)
    window.view_button.click()
    command, = launched
    assert ('--depth-max' in command) == (depth is not None)
    assert ('--surface-cleanup' in command) == (depth is not None and cleanup)
    if depth is not None:
        assert command[command.index('--depth-max')+1] == f'{depth:g}'


def test_view_button_keeps_imu_comparison_explicit(saved_viewer_window):
    window, launched, _ = saved_viewer_window
    window.view_backend.setCurrentIndex(window.view_backend.findData('orb_inertial'))
    window.view_button.click()
    command, = launched
    assert command[command.index('--backend')+1] == 'orb_inertial'


@pytest.mark.parametrize('manifest', ['merge.json', 'depth_variant.json'])
def test_already_derived_maps_open_without_raw_reconstruction(saved_viewer_window, manifest):
    window, launched, run = saved_viewer_window
    (run/manifest).write_text('{}')
    window.view_button.click()
    command, = launched
    assert '--depth-max' not in command
    assert '--surface-cleanup' not in command
    assert '기존 거리' in window.session.snapshot()['message']


def test_unavailable_comparison_does_not_silently_open_camera_map(saved_viewer_window):
    window, launched, _ = saved_viewer_window
    window.view_backend.setCurrentIndex(window.view_backend.findData('orbline'))
    window.view_button.click()
    assert launched == []
    assert '아직 저장된' in window.session.snapshot()['message']


def test_joined_export_button_launches_and_opens_result(tmp_path, monkeypatch):
    from PyQt5.QtWidgets import QApplication
    from PyQt5.QtCore import QProcess
    from experiments.handheld_mapping.app import MappingWindow
    from experiments.handheld_mapping.controller import Session
    from experiments.handheld_mapping import app as module
    app = QApplication.instance() or QApplication([])
    session = Session()
    session.update(run_dir=str(tmp_path))
    window = MappingWindow(session, output_root=tmp_path)
    commands, opened = [], []
    monkeypatch.setattr(QProcess, 'start', lambda self, program, args: commands.append((program, args)))
    monkeypatch.setattr(module.QDesktopServices, 'openUrl', lambda url: opened.append(url.toLocalFile()))
    window.joined_export_button.click()
    assert 'experiments.handheld_mapping.joined_export' in commands[0][1]
    assert str(tmp_path) in commands[0][1]
    assert not window.joined_export_button.isEnabled()
    assert not window.start_button.isEnabled()
    process = window.export_process
    process.finished.emit(0, QProcess.NormalExit)
    assert window.joined_export_button.isEnabled()
    assert len(opened) == 1 and '/exports/planar_' in opened[0]
    window.close()
