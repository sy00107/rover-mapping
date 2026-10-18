"""Single-owner Pi bridge supervisor; SSH only starts it or follows its log.

This module is deployed as a standalone standard-library script. The existing
UDP stale-command clamp and MCU watchdog are unchanged. Ctrl+C in the SSH log
viewer detaches; explicit `stop` asks the bridge to exit with zero commands.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time

ROOT = Path('/dev/shm/rover_mapping_imu_bridge_service')


def identity(pid):
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return None if fields[0] == 'Z' else dict(pid=int(pid), start_ticks=fields[19])
    except (OSError, IndexError):
        return None


def alive(state):
    actual = identity(state['pid']) if state.get('pid') else None
    return actual is not None and actual == {k: state[k] for k in ('pid', 'start_ticks')}


def read_state(root):
    path = root/'state.json'
    return json.loads(path.read_text()) if path.exists() else {}


def save(root, state):
    temporary = root/'state.json.tmp'
    temporary.write_text(json.dumps(state, ensure_ascii=False))
    temporary.replace(root/'state.json')


def bridge_command(root):
    source = Path('/home/pi/ksrc/pi/common')
    if subprocess.run(['fuser', '-s', '/dev/spidev0.0'], capture_output=True).returncode == 0:
        raise RuntimeError('SPI가 다른 프로그램에서 사용 중입니다. 기존 브리지를 정상 종료하세요.')
    run = Path(tempfile.mkdtemp(prefix='code_', dir=root))
    common = run/'pi/common'; common.mkdir(parents=True)
    for name in ('udp_spi_bridge.py', 'nucleo_link.py'):
        shutil.copyfile(source/name, common/name)
    text = (common/'udp_spi_bridge.py').read_text()
    if 'from telemetry_tee import' in text and '--telemetry-socket' in text:
        shutil.copyfile(source/'telemetry_tee.py', common/'telemetry_tee.py')
    else:
        subprocess.run(['patch', '--batch', '--fuzz=0', '--forward', '-p1', '-i', str(root/'tee.patch')],
                       cwd=run, check=True)
    return [sys.executable, '-u', str(common/'udp_spi_bridge.py'), '--hz', '50',
            '--max-lin', '0.45', '--max-ang', '0.8',
            '--telemetry-socket', '/dev/shm/rover-mapping-imu.sock']


def supervise(root):
    root = Path(root)
    with (root/'owner.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return
        stop = False
        def request_stop(*_):
            nonlocal stop
            stop = True
        signal.signal(signal.SIGTERM, request_stop)
        signal.signal(signal.SIGINT, request_stop)
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
        child = None
        state = {**identity(os.getpid()), 'phase': 'starting'}
        try:
            config = json.loads((root/'config.json').read_text())
            command = config.get('test_command') or bridge_command(root)
            with (root/'bridge.log').open('ab') as log:
                child = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log,
                                         stderr=log, start_new_session=True)
            # Catch immediate import/device failures before advertising readiness.
            time.sleep(.3)
            if child.poll() is not None:
                raise RuntimeError('IMU 브리지 시작 실패 · bridge.log를 확인하세요')
            state.update(phase='running', child=identity(child.pid))
            save(root, state)
            while not stop and child.poll() is None:
                time.sleep(.1)
            if not stop:
                raise RuntimeError('IMU 브리지가 비정상 종료됐습니다')
        except Exception as error:
            state.update(phase='failed', error=str(error))
        finally:
            if child and child.poll() is None:
                # SIGINT reaches the existing bridge's KeyboardInterrupt/finally zero-command path.
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    child.terminate()
                    try:
                        child.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        child.kill(); child.wait()
            if state['phase'] != 'failed':
                state['phase'] = 'stopped'
            save(root, state)


def start_service(root=ROOT, command=None):
    root = Path(root); root.mkdir(parents=True, exist_ok=True)
    with (root/'launch.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        existing = read_state(root)
        if existing.get('phase') == 'running' and alive(existing):
            return existing
        save(root, dict(phase='starting'))
        (root/'config.json').write_text(json.dumps({'test_command': command} if command else {}))
        with (root/'supervisor.log').open('ab') as log:
            process = subprocess.Popen([sys.executable, '-u', str(Path(__file__).resolve()),
                'serve', '--root', str(root)], stdin=subprocess.DEVNULL, stdout=log,
                stderr=log, start_new_session=True)
        deadline = time.monotonic()+10
        while time.monotonic() < deadline:
            state = read_state(root)
            if state.get('phase') == 'running' and alive(state):
                return state
            if state.get('phase') == 'failed' or process.poll() is not None:
                raise RuntimeError(state.get('error', 'IMU supervisor 시작 실패'))
            time.sleep(.05)
        if process.poll() is None:
            process.terminate()
        raise TimeoutError('IMU 브리지 시작 시간 초과')


def stop_service(root=ROOT):
    root = Path(root)
    state = read_state(root)
    if alive(state):
        os.kill(state['pid'], signal.SIGTERM)
        deadline = time.monotonic()+8
        while alive(state) and time.monotonic() < deadline:
            time.sleep(.05)
        if alive(state):
            raise TimeoutError('IMU supervisor 종료 확인 실패')
    return read_state(root)


def follow(root):
    root = Path(root)
    offset = max(0, (root/'bridge.log').stat().st_size-4096) if (root/'bridge.log').exists() else 0
    while True:
        if (root/'bridge.log').exists():
            with (root/'bridge.log').open('rb') as stream:
                stream.seek(offset)
                data = stream.read(65536); offset = stream.tell()
            if data:
                sys.stdout.buffer.write(data); sys.stdout.buffer.flush()
        state = read_state(root)
        if state.get('phase') != 'running' or not alive(state):
            return
        time.sleep(.2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['start','serve','follow','stop','status'])
    parser.add_argument('--root', type=Path, default=ROOT)
    args = parser.parse_args()
    if args.action == 'serve':
        supervise(args.root)
    elif args.action == 'start':
        print(json.dumps(start_service(args.root), ensure_ascii=False), flush=True)
    elif args.action == 'stop':
        print(json.dumps(stop_service(args.root), ensure_ascii=False), flush=True)
    elif args.action == 'follow':
        follow(args.root)
    else:
        print(json.dumps(read_state(args.root), ensure_ascii=False))


if __name__ == '__main__':
    main()
