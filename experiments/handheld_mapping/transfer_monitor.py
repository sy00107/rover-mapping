"""Read-only transfer progress terminal. Closing it never controls capture or transfer."""
import argparse
import json
import os
from pathlib import Path
import sys
import time
import uuid


EVENT_PHASES = {
    'stop_pending': 'waiting_stop', 'stop_ack': 'waiting_stop',
    'capture_stopped': 'finalizing', 'transfer_begin': 'transferring',
    'transfer_progress': 'transferring', 'originals_begin': 'importing',
    'retention_status': 'verifying', 'transfer_complete': 'complete',
    'transport_paused': 'paused', 'capture_failure': 'capture_warning',
}
LABELS = {'recording': '촬영 중 · 전송 대기', 'waiting_stop': 'Pi 촬영 종료 확인 중',
    'finalizing': 'Pi 원본 파일 마감 중', 'transferring': 'Pi → PC 원본 수신',
    'importing': '수신 완료 · 영상 정리 중', 'verifying': '원본 검증 / Pi 정리 중',
    'complete': '원본 전송 완료', 'paused': '보류 · 이어받기 가능',
    'failed': '전송 오류 · 원본 보존', 'capture_warning': '촬영 오류 · 남은 원본 회수 중'}


def process_ticks(pid):
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return None if fields[0] == 'Z' else fields[19]
    except (OSError, IndexError):
        return None


class TransferReporter:
    """Best-effort UI telemetry; a log/disk failure never changes transfer success."""
    def __init__(self, run):
        self.run = Path(run)
        self.state = dict(schema=1, attempt=uuid.uuid4().hex, pid=os.getpid(),
                          start_ticks=process_ticks(os.getpid()), phase='recording',
                          bytes=0, total=0, updated_ns=time.time_ns())

    def event(self, header):
        kind = header.get('kind')
        if kind not in EVENT_PHASES and kind != 'error':
            return
        phase = EVENT_PHASES.get(kind, 'failed')
        self.state.update(phase=phase, updated_ns=time.time_ns())
        for key in ('bytes', 'total', 'reconnecting', 'result'):
            if key in header:
                self.state[key] = header[key]
        if kind != 'transfer_progress':
            self.state.pop('reconnecting', None)
        self.state['message'] = header.get('message', LABELS.get(phase, phase))
        if kind == 'transfer_complete' and self.state.get('result'):
            result = self.state['result']
            self.state['message'] += (' · Pi 정리 보류: '+result.get('reason', '')
                if result.get('status') == 'deferred' else
                f" · Pi 과거 촬영 {len(result.get('deleted', []))}개 정리 · 최신/미검증 원본 보관")
        try:
            from .storage import atomic_json
            atomic_json(self.run/'transfer_status.json', self.state)
            with (self.run/'transfer_progress.jsonl').open('a') as log:
                log.write(json.dumps(self.state, ensure_ascii=False)+'\n')
        except (OSError, ValueError) as error:
            print('전송 모니터 기록 실패: '+str(error), file=sys.stderr, flush=True)


def progress_line(state, speed=0.):
    done, total = max(0, state.get('bytes', 0)), max(0, state.get('total', 0))
    fraction = min(1., done/total) if total else 0.
    bar = '#'*int(fraction*28)+'-'*(28-int(fraction*28))
    percent = f'{fraction*100:5.1f}%' if total else '  대기 '
    text = f'[{bar}] {percent}  {done/1e6:,.1f} / {total/1e6:,.1f} MB'
    if speed > 0 and done < total:
        remaining = int((total-done)/speed)
        text += f'  {speed/1e6:.1f} MB/s  남은 약 {remaining//60:02d}:{remaining%60:02d}'
    if state.get('reconnecting'):
        text += '  연결 복구 후 이어받는 중'
    return text


def fit_terminal_line(text, columns):
    import unicodedata
    used, result = 0, []
    for char in text.replace('\n', ' ').replace('\r', ' '):
        width = 0 if unicodedata.combining(char) else (2 if unicodedata.east_asian_width(char) in 'WF' else 1)
        if used+width > max(0, columns-2):
            return ''.join(result)+'…'
        result.append(char)
        used += width
    return ''.join(result)


class TerminalLine:
    def __init__(self, stream):
        self.stream = stream
        self.live = bool(getattr(stream, 'isatty', lambda: False)())
        self.visible = False

    def update(self, text):
        if self.live:
            import shutil
            text = fit_terminal_line(text, shutil.get_terminal_size((120, 30)).columns)
            self.stream.write('\r\033[2K'+text)
            self.stream.flush()
            self.visible = True
        else:
            print(text, file=self.stream, flush=True)

    def notice(self, text):
        if self.live and self.visible:
            self.stream.write('\r\033[2K')
            self.visible = False
        print(text, file=self.stream, flush=True)

    def finish(self):
        if self.live and self.visible:
            print(file=self.stream, flush=True)
            self.visible = False


def watch(run, poll=.2, stream=None):
    stream = stream or sys.stdout
    run = Path(run)
    print('영상 · IMU 원본 전송 모니터', file=stream, flush=True)
    print(str(run), file=stream, flush=True)
    print('이 창을 닫아도 전송은 계속됩니다. 보류는 매핑 앱에서 하세요.', file=stream, flush=True)
    print('100%는 바이트 수신률입니다. 이후 원본 검증·Pi 정리 결과를 확인하세요.\n', file=stream, flush=True)
    previous = None
    sample = None
    speed = 0.
    display = TerminalLine(stream)
    last_attempt = None
    reconnecting = False
    while True:
        try:
            state = json.loads((run/'transfer_status.json').read_text())
        except (OSError, ValueError):
            if previous is None:
                display.update('전송 상태 대기 중…')
                previous = 'waiting'
            time.sleep(poll)
            continue
        signature = (state.get('attempt'), state.get('phase'), state.get('bytes'),
                     state.get('total'), state.get('message'), state.get('reconnecting'))
        now = time.monotonic()
        if last_attempt != state.get('attempt'):
            reconnecting = False
            speed = 0.
            last_attempt = state.get('attempt')
        if sample and sample[1] > 0 and sample[0] == state.get('attempt') and state['bytes'] > sample[1] and now > sample[2]:
            speed = (state['bytes']-sample[1])/(now-sample[2])
        if not sample or sample[0] != state.get('attempt') or state['bytes'] != sample[1]:
            sample = (state.get('attempt'), state['bytes'], now)
        if signature != previous:
            phase = state['phase']
            connection_warning = bool(state.get('reconnecting'))
            if connection_warning and not reconnecting:
                display.notice('[연결 불안정] 전송 연결이 끊기거나 응답이 없습니다. 이어받기를 시도합니다.')
                speed = 0.
            elif reconnecting and not connection_warning and phase == 'transferring':
                display.notice('[연결 복구] 저장된 위치부터 전송을 이어갑니다.')
            reconnecting = connection_warning
            if phase == 'transferring':
                display.update('Pi→PC  '+progress_line(state, speed))
            else:
                display.update(state['message']+' · '+progress_line(state) if phase == 'complete' else state['message'])
            previous = signature
        if state['phase'] in {'complete', 'paused', 'failed'}:
            display.finish()
            return state['phase']
        if (state.get('pid') and process_ticks(state['pid']) != state.get('start_ticks')):
            display.notice('[전송 중단] 프로세스 종료 감지 · 완료 여부를 확인할 수 없습니다. 앱에서 같은 촬영을 이어받으세요.')
            return 'interrupted'
        time.sleep(poll)


def launch_terminal(run):
    import shutil
    import subprocess
    run = Path(run).resolve()
    command = [sys.executable, '-m', 'experiments.handheld_mapping.transfer_monitor', str(run)]
    title = '영상·IMU 전송 — '+run.name
    if terminal := shutil.which('gnome-terminal'):
        command = [terminal, '--window', '--title='+title, '--', *command]
    elif terminal := shutil.which('xterm'):
        command = [terminal, '-T', title, '-e', *command]
    elif terminal := shutil.which('x-terminal-emulator'):
        command = [terminal, '-T', title, '-e', *command]
    else:
        raise OSError('터미널 프로그램이 없습니다. 앱의 전송률과 transfer_progress.jsonl을 확인하세요.')
    environment = dict(os.environ, PYTHONNOUSERSITE='1')
    with (run/'transfer_terminal.log').open('ab') as log:
        return subprocess.Popen(command, cwd=Path(__file__).resolve().parents[2],
            env=environment, stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--no-hold', action='store_true')
    args = parser.parse_args()
    try:
        watch(args.run)
        if not args.no_hold and sys.stdin.isatty():
            input('\nEnter를 누르면 이 창을 닫습니다. ')
    except (KeyboardInterrupt, EOFError):
        pass


if __name__ == '__main__':
    main()
