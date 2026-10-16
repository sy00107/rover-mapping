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


def test_fusion_checkbox_requires_imu_recording_and_is_opt_in(tmp_path, monkeypatch):
    from PyQt5.QtWidgets import QApplication
    from experiments.handheld_mapping.app import MappingWindow
    from experiments.handheld_mapping.controller import Session
    app = QApplication.instance() or QApplication([])
    session = Session()
    settings = []
    monkeypatch.setattr(session, "start", settings.append)
    window = MappingWindow(session, output_root=tmp_path)
    try:
        assert not window.inertial_fusion.isChecked()
        window.imu_enabled.setChecked(False)
        window.inertial_fusion.setChecked(True)
        assert window.imu_enabled.isChecked()
        window.start()
        assert settings[-1]["inertial_fusion"] is True
        assert settings[-1]["imu_enabled"] is True
        session.update(state="processing")
        window.refresh()
        assert not window.inertial_fusion.isEnabled()
        session.update(state="idle")
        window.refresh()
        window.imu_enabled.setChecked(False)
        assert not window.inertial_fusion.isChecked()
        window.inertial_fusion.setChecked(True)
        window.demo.setChecked(True)
        window.refresh()
        assert not window.inertial_fusion.isChecked()
        assert not window.inertial_fusion.isEnabled()
        app.processEvents()
    finally:
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
        assert window.inertial_fusion.isChecked()
        assert window.imu_enabled.isChecked()
        app.processEvents()
    finally:
        window.close()
