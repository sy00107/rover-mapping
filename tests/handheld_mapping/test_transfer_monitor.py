"""Progress monitor is independent of the transport and uses no robot I/O."""
import io
import json
import subprocess
import sys
from types import SimpleNamespace

from experiments.handheld_mapping import transfer_monitor as monitor


def test_percent_means_bytes_not_final_validation(tmp_path):
    reporter=monitor.TransferReporter(tmp_path)
    reporter.event(dict(kind='transfer_begin',total=100,bytes=0))
    reporter.event(dict(kind='transfer_progress',total=100,bytes=100))
    state=json.loads((tmp_path/'transfer_status.json').read_text())
    assert '100.0%' in monitor.progress_line(state)
    assert state['phase']=='transferring'
    reporter.event(dict(kind='originals_begin'))
    reporter.event(dict(kind='retention_status',message='전체 원본 검증 중'))
    state=json.loads((tmp_path/'transfer_status.json').read_text())
    assert state['phase']=='verifying'
    reporter.event(dict(kind='retention_status',message='보류',result=dict(status='deferred',reason='Pi offline')))
    reporter.event(dict(kind='transfer_complete'))
    state=json.loads((tmp_path/'transfer_status.json').read_text())
    assert state['phase']=='complete' and 'Pi 정리 보류' in state['message']
    assert 'Pi offline' in state['message']
    assert len((tmp_path/'transfer_progress.jsonl').read_text().splitlines())==6
    output=io.StringIO()
    assert monitor.watch(tmp_path,stream=output)=='complete'
    assert '100%는 바이트 수신률' in output.getvalue()
    assert '100.0%' in output.getvalue()


def test_speed_eta_and_reconnect_are_visible():
    text=monitor.progress_line(dict(bytes=25_000_000,total=100_000_000,reconnecting=True),5_000_000)
    assert '25.0%' in text and '25.0 / 100.0 MB' in text
    assert '5.0 MB/s' in text and '00:15' in text and '이어받는 중' in text


def test_closing_monitor_process_does_not_stop_reporter(tmp_path):
    reporter=monitor.TransferReporter(tmp_path)
    reporter.event(dict(kind='transfer_begin',bytes=0,total=100))
    process=subprocess.Popen([sys.executable,'-u','-m','experiments.handheld_mapping.transfer_monitor',
        str(tmp_path),'--no-hold'],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    try:
        assert '전송 모니터' in process.stdout.readline().decode()
        process.terminate();process.wait(timeout=5)
        reporter.event(dict(kind='transfer_progress',bytes=70,total=100))
        assert json.loads((tmp_path/'transfer_status.json').read_text())['bytes']==70
        reporter.event(dict(kind='transfer_progress',bytes=100,total=100))
        reporter.event(dict(kind='transfer_complete'))
        second=subprocess.run([sys.executable,'-m','experiments.handheld_mapping.transfer_monitor',
            str(tmp_path),'--no-hold'],capture_output=True,text=True,timeout=5)
        assert second.returncode==0 and '100.0%' in second.stdout and '전송 완료' in second.stdout
    finally:
        if process.poll() is None:process.kill();process.wait()
        process.stdout.close();process.stderr.close()


def test_monitor_does_not_confuse_dead_sender_with_completion(tmp_path):
    reporter=monitor.TransferReporter(tmp_path)
    reporter.state.update(pid=99999999,start_ticks='x')
    reporter.event(dict(kind='transfer_progress',bytes=10,total=100))
    output=io.StringIO()
    assert monitor.watch(tmp_path,stream=output)=='interrupted'
    assert '완료 여부를 확인할 수 없습니다' in output.getvalue()


def test_telemetry_disk_failure_does_not_fail_transport(tmp_path,monkeypatch):
    from experiments.handheld_mapping import storage
    reporter=monitor.TransferReporter(tmp_path)
    def fail(*args):raise OSError('disk unavailable')
    monkeypatch.setattr(storage,'atomic_json',fail)
    reporter.event(dict(kind='transfer_progress',bytes=50,total=100))
    assert reporter.state['bytes']==50


def test_terminal_launch_uses_separate_window_and_safe_arguments(tmp_path,monkeypatch):
    import shutil
    calls=[]
    monkeypatch.setattr(shutil,'which',lambda name:'/usr/bin/gnome-terminal' if name=='gnome-terminal' else None)
    monkeypatch.setattr(subprocess,'Popen',lambda args,**kwargs:calls.append((args,kwargs)))
    run=tmp_path/'capture with spaces';run.mkdir()
    monitor.launch_terminal(run)
    args,options=calls[0]
    assert args[:2]==['/usr/bin/gnome-terminal','--window']
    assert args[-1]==str(run) and 'experiments.handheld_mapping.transfer_monitor' in args
    assert '--' in args and options['start_new_session'] and 'shell' not in options


def test_gui_auto_opens_once_per_attempt_and_can_reopen(tmp_path,monkeypatch):
    from PyQt5.QtWidgets import QApplication
    from experiments.handheld_mapping.app import MappingWindow
    from experiments.handheld_mapping.controller import Session
    app=QApplication.instance() or QApplication([])
    calls=[]
    monkeypatch.setattr(monitor,'launch_terminal',lambda run:calls.append(run) or SimpleNamespace(poll=lambda:None))
    reporter=monitor.TransferReporter(tmp_path)
    reporter.event(dict(kind='transfer_begin',bytes=0,total=100))
    session=Session();session.update(state='transferring',local_recording=True,run_dir=str(tmp_path))
    window=MappingWindow(session,tmp_path)
    try:
        for _ in range(3):window.refresh()
        assert calls==[tmp_path]
        window.transfer_terminal_button.click()
        assert calls==[tmp_path,tmp_path]
        # A completed very fast transfer must still auto-open on the next GUI poll.
        reporter=monitor.TransferReporter(tmp_path);reporter.event(dict(kind='transfer_complete'))
        session.update(state='complete');window.refresh();app.processEvents()
        assert len(calls)==3
    finally:
        session.update(state='idle');window.close()


def test_live_terminal_overwrites_progress_and_adds_only_notices():
    class Tty(io.StringIO):
        def isatty(self):return True
    output=Tty()
    line=monitor.TerminalLine(output)
    line.update('5%');line.update('10%');line.update('40%')
    assert output.getvalue().count('\n')==0
    line.notice('[연결 불안정] 이어받기 시도')
    line.update('40%');line.update('45%')
    line.notice('[연결 복구] 이어받는 중')
    line.update('100%');line.finish()
    text=output.getvalue()
    assert text.count('\n')==3
    assert text.count('\r\033[2K')==8


def test_terminal_line_does_not_wrap_wide_korean_characters():
    assert monitor.fit_terminal_line('가나다라마바사',10)=='가나다라…'
    assert '\n' not in monitor.fit_terminal_line('hello\nworld',120)
