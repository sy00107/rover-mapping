# ================================================================
# test_imu_capture.py
# 설명: IMU 보정·시계 분류·subprocess 정리와 UNIX 구독 경계를 검증한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_imu_capture.py -q
# ================================================================
import json
from pathlib import Path
import socket
import subprocess
import sys
import time

import pytest

AXES = [[0, -1], [1, -1], [2, 1]]
CALIBRATION = {
    'version': 1, 'sensor': 'ISM330DHCX', 'frame': 'rover_body',
    'chip_to_body': AXES, 'acc_unit': 'g', 'gyro_unit': 'deg/s',
    'gyro_bias_dps': [-.5, .4, .6], 'acc_offset_g': [.01, .02, .03],
    'acc_scale': [1.01, .99, 1.02],
}
CAPTURES = []


@pytest.fixture(autouse=True)
def cleanup_captures():
    yield
    while CAPTURES:
        CAPTURES.pop().stop()


def packet(t_ms: int, tick: int = 0, flags: int = 16) -> dict:
    return {'kind': 'telemetry', 'schema': 1, 'run_id': '',
            'host_receipt_ns': time.time_ns(), 'host_monotonic_ns': time.monotonic_ns(),
            'tee_drops': 0, 'telemetry': {'flags': flags, 'tick': tick, 't_ms': t_ms,
            'gyro_dps': [.5, 1.4, 1.6], 'acc_g': [.01, .02, 1.03],
            'imu_raw': [1, 2, 3, 4, 5, 6], 'imu_n': 4}}


def capture(tmp_path: Path, events: list, *, wait: bool = True):
    from experiments.handheld_mapping.imu import ImuCapture
    cal = tmp_path / 'source.json'
    cal.write_text(json.dumps(CALIBRATION))
    urdf = tmp_path / 'source.urdf'
    urdf.write_text('<robot name="test"/>')
    code = ('import json,sys,time\n' +
            'events=json.loads(' + repr(json.dumps(events)) + ')\n' +
            'for event in events:\n print(json.dumps(event),flush=True)\n time.sleep(.01)\n' +
            ('sys.stdin.readline()\n' if wait else ''))
    result = ImuCapture('pi@rover', tmp_path / 'run', cal, urdf,
                        command=[sys.executable, '-u', '-c', code])
    CAPTURES.append(result)
    return result


def ready(axes=AXES) -> dict:
    return {'kind': 'ready', 'chip_to_body': axes}


def wait_recorded(imu, n: int) -> dict:
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        state = imu.snapshot()
        if state['recorded'] >= n:
            return state
        time.sleep(.01)
    raise AssertionError(imu.snapshot())


def test_capture_corrects_body_values_and_keeps_raw_duplicate_startup(tmp_path: Path):
    imu = capture(tmp_path, [ready(), packet(269700), packet(20, 1), packet(20, 2), packet(40, 3)])
    imu.start(timeout=2)
    state = wait_recorded(imu, 4)
    assert state['unique'] == 2 and state['duplicates'] == 1
    assert state['valid'] == 4 and state['startup_discarded'] == 1
    assert state['gyro_dps'] == pytest.approx([1., 1., 1.])
    assert state['acc_g'] == pytest.approx([0., 0., 1.02])
    assert state['acc_norm_g'] == pytest.approx(1.02)
    assert state['roll_deg'] == pytest.approx(0)
    assert state['pitch_deg'] == pytest.approx(0)
    state['gyro_dps'][0] = 999
    assert imu.snapshot()['gyro_dps'][0] == pytest.approx(1)
    stopped = imu.stop()
    assert stopped['state'] == 'stopped'
    assert imu.process.poll() is not None
    rows = [json.loads(line) for line in (tmp_path / 'run/imu.jsonl').read_text().splitlines()]
    assert len(rows) == 4
    assert rows[0]['acceptance'] == 'startup_discard'
    assert rows[2]['acceptance'] == 'duplicate'
    assert rows[1]['raw']['telemetry']['gyro_dps'] == [.5, 1.4, 1.6]
    assert rows[1]['calibrated']['gyro_dps'] == pytest.approx([1, 1, 1])
    assert rows[1]['clock']['imu_clock_verified'] is False
    assert rows[1]['clock']['laptop_receipt_monotonic_ns'] > 0
    manifest = json.loads((tmp_path / 'run/imu_manifest.json').read_text())
    assert manifest['orb_fused'] is False and manifest['extrinsic_provisional'] is True
    assert manifest['calibration_sha256'] and manifest['urdf_sha256']
    assert (tmp_path / 'run/imu_summary.json').is_file()
    assert (tmp_path / 'run/imu_calibration.json').is_file()
    assert (tmp_path / 'run/robot.urdf').is_file()
    assert imu.stop() == stopped


@pytest.mark.parametrize('event,error', [
    (ready([[0, 1], [1, -1], [2, 1]]), 'chip_to_body'),
    ({'kind': 'ready'}, 'chip_to_body'),
    ({'kind': 'unexpected'}, 'ready'),
])
def test_bad_ready_fails_and_cleans_owned_process(tmp_path: Path, event: dict, error: str):
    imu = capture(tmp_path, [event, packet(10), packet(20)])
    with pytest.raises((ValueError, RuntimeError), match=error):
        imu.start(timeout=.5)
    assert imu.snapshot()['state'] == 'failed'
    assert imu.process.poll() is not None
    assert imu.stop()['state'] == 'failed'


def test_nonfinite_measurement_fails_and_no_first_sample_timeout(tmp_path: Path):
    event = packet(20)
    event['telemetry']['gyro_dps'][0] = float('nan')
    imu = capture(tmp_path, [ready(), event])
    with pytest.raises((ValueError, RuntimeError), match='finite|JSON'):
        imu.start(timeout=.5)
    assert imu.process.poll() is not None
    other = tmp_path / 'other'
    other.mkdir()
    imu = capture(other, [ready(), packet(10)])
    with pytest.raises((TimeoutError, RuntimeError), match='fresh|timeout'):
        imu.start(timeout=.15)
    assert imu.process.poll() is not None
    assert imu.snapshot()['stale'] is True


def test_reset_discards_boundary_and_duplicate_does_not_keep_stream_fresh(tmp_path: Path):
    imu = capture(tmp_path, [ready(), packet(10), packet(20), packet(40), packet(5), packet(5), packet(25)])
    imu.start(timeout=2)
    state = wait_recorded(imu, 6)
    assert state['unique'] == 3 and state['resets'] == 1
    assert state['duplicates'] == 1
    time.sleep(.55)
    assert imu.snapshot()['stale'] is True
    imu.stop()


def test_existing_artifacts_are_preserved(tmp_path: Path):
    imu = capture(tmp_path, [ready(), packet(10), packet(20)])
    imu.run.mkdir()
    target = imu.run / 'imu.jsonl'
    target.write_text('existing')
    with pytest.raises(FileExistsError):
        imu.start(timeout=.5)
    assert target.read_text() == 'existing'


def test_invalid_configuration_rejected_before_process(tmp_path: Path):
    imu = capture(tmp_path, [ready(), packet(10), packet(20)])
    cal = dict(CALIBRATION, frame='chip')
    imu.calibration_path.write_text(json.dumps(cal))
    with pytest.raises(ValueError, match='frame'):
        imu.start(timeout=.5)
    assert imu.process is None


def test_host_validation_and_quoted_run_path(tmp_path: Path):
    from experiments.handheld_mapping.imu import ImuCapture
    for host in ['-oProxyCommand=bad', 'pi; touch /tmp/no']:
        with pytest.raises(ValueError, match='host|SSH'):
            ImuCapture(host, tmp_path, tmp_path / 'cal', tmp_path / 'urdf')
    imu = ImuCapture('pi@rover', tmp_path / "run ' with space", tmp_path / 'cal', tmp_path / 'urdf')
    assert imu.command[0] == 'ssh'
    import shlex
    outer = shlex.split(imu.command[-1])
    assert outer[:2] == ['bash', '-c']
    assert '/usr/bin/python3 -u -m experiments.handheld_mapping.imu_subscriber' in outer[2]


def test_real_calibration_formula_has_no_second_axis_transform():
    from experiments.handheld_mapping.imu import load_calibration, correct_measurement
    source = Path('/home/su/Downloads/imu_calibration.json')
    if not source.is_file():
        pytest.skip('사용자 보정 원본이 없음')
    cal = load_calibration(source)
    corrected = correct_measurement(cal, cal['gyro_bias_dps'],
                                    [cal['acc_offset_g'][i] + (1 / cal['acc_scale'][i] if i == 2 else 0) for i in range(3)])
    assert corrected['gyro_dps'] == pytest.approx([0, 0, 0])
    assert corrected['acc_g'] == pytest.approx([0, 0, 1])


def test_subscriber_refuses_existing_socket_without_unlink(tmp_path: Path):
    from experiments.handheld_mapping.imu_subscriber import subscribe
    target = tmp_path / 'existing.sock'
    existing = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    existing.bind(str(target))
    try:
        with pytest.raises(FileExistsError):
            subscribe(str(target), AXES)
        assert target.exists()
    finally:
        existing.close()
        target.unlink()


def test_subscriber_unix_stream_and_cleanup_without_spi(tmp_path: Path):
    target = tmp_path / 'imu.sock'
    code = ('from experiments.handheld_mapping.imu_subscriber import subscribe; '
            f'subscribe({str(target)!r}, {AXES!r})')
    process = subprocess.Popen([sys.executable, '-u', '-c', code], stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    sender = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        event = json.loads(process.stdout.readline())
        assert event['kind'] == 'ready' and event['chip_to_body'] == AXES
        sender.sendto(b'not json', str(target))
        sender.sendto(json.dumps(packet(20)).encode(), str(target))
        malformed = json.loads(process.stdout.readline())
        assert malformed['kind'] == 'malformed' and malformed['count'] == 1
        received = json.loads(process.stdout.readline())
        assert received['kind'] == 'telemetry' and received['telemetry']['t_ms'] == 20
        process.stdin.write('stop\n')
        process.stdin.flush()
        assert process.wait(timeout=2) == 0
        assert not target.exists()
    finally:
        sender.close()
        if process.poll() is None:
            process.kill()
            process.wait()
        for handle in (process.stdin, process.stdout, process.stderr):
            handle.close()


def test_drop_baseline_is_per_run_and_bounded_stats(tmp_path: Path):
    events = [ready()] + [packet(i * 20, i % 256) for i in range(400)]
    for i, event in enumerate(events[1:]):
        event['tee_drops'] = 9000 + i // 100
    imu = capture(tmp_path, events)
    imu.command[-1] = imu.command[-1].replace('time.sleep(.01)', 'time.sleep(0)')
    imu.start(timeout=2)
    state = wait_recorded(imu, 400)
    assert state['tee_drops'] == 3
    assert state['tee_drops_baseline'] == 9000
    assert state['tee_drops_total'] == 9003
    assert len(imu._recent) <= 256
    imu.stop()


def test_parse_error_is_saved_and_subprocess_cleaned(tmp_path: Path):
    imu = capture(tmp_path, [])
    imu.command[-1] = 'import sys; print("not-json",flush=True); sys.stdin.readline()'
    with pytest.raises((ValueError, RuntimeError)):
        imu.start(timeout=1)
    assert imu.process.poll() is not None
    assert imu.snapshot()['state'] == 'failed'
    row = json.loads((tmp_path / 'run/imu.jsonl').read_text())
    assert row['acceptance'] == 'invalid_json'


def test_unavailable_imu_packets_are_retained_without_acceptance(tmp_path: Path):
    unavailable = packet(15, flags=0)
    imu = capture(tmp_path, [ready(), packet(10), unavailable, packet(20)])
    imu.start(timeout=1)
    assert wait_recorded(imu, 3)['valid'] == 2
    assert imu.snapshot()['unique'] == 1
    imu.stop()
    rows = [json.loads(line) for line in (tmp_path / 'run/imu.jsonl').read_text().splitlines()]
    assert rows[1]['acceptance'] == 'invalid_flag'
    assert rows[1]['raw'] == unavailable


def test_subscriber_eof_and_signal_remove_only_owned_socket(tmp_path: Path):
    for action in ('eof', 'signal', 'replace'):
        target = tmp_path / (action + '.sock')
        code = ('from experiments.handheld_mapping.imu_subscriber import subscribe; '
                f'subscribe({str(target)!r}, {AXES!r})')
        process = subprocess.Popen([sys.executable, '-u', '-c', code], stdin=subprocess.PIPE,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        replacement = None
        try:
            assert json.loads(process.stdout.readline())['kind'] == 'ready'
            if action == 'replace':
                target.unlink()
                replacement = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
                replacement.bind(str(target))
            if action == 'signal':
                process.terminate()
            else:
                process.stdin.close()
            assert process.wait(timeout=2) == 0
            assert target.exists() is (action == 'replace')
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
            for handle in (process.stdin, process.stdout, process.stderr):
                handle.close()
            if replacement:
                replacement.close()
                target.unlink()


def test_metadata_import_does_not_construct_spilink(tmp_path: Path, monkeypatch):
    from experiments.handheld_mapping import imu_subscriber
    fake = tmp_path / 'nucleo_link.py'
    fake.write_text('IMU_CHIP_TO_BODY=((0,-1),(1,-1),(2,1))\n'
                    'class SpiLink:\n'
                    ' def __init__(self): raise AssertionError("SPI must not open")\n')
    original = imu_subscriber.importlib.util.spec_from_file_location
    def local_spec(name: str, path: Path):
        assert str(path) == '/home/pi/ksrc/pi/common/nucleo_link.py'
        return original(name, fake)
    monkeypatch.setattr(imu_subscriber.importlib.util, 'spec_from_file_location', local_spec)
    assert imu_subscriber.actual_chip_to_body() == AXES


def test_uint32_wrap_is_unique_and_mcu_jump_is_not_accepted(tmp_path: Path):
    imu = capture(tmp_path, [ready(), packet(2**32 - 60), packet(2**32 - 40),
                             packet(2**32 - 20), packet(0), packet(500000), packet(20), packet(40)])
    imu.start(timeout=1)
    state = wait_recorded(imu, 7)
    assert state['clock_jumps'] == 1
    assert state['resets'] == 1
    assert state['unique'] == 4
    assert state['imu_clock_verified'] is False
    imu.stop()


def test_reset_boundary_immediately_marks_tilt_stale(tmp_path: Path):
    imu = capture(tmp_path, [ready(), packet(10), packet(20), packet(5)])
    imu.start(timeout=1)
    state = wait_recorded(imu, 3)
    assert state['resets'] == 1
    assert state['stale'] is True
    imu.stop()


def test_subscriber_partial_stdin_still_handles_sigterm(tmp_path: Path):
    target = tmp_path / 'partial.sock'
    code = ('from experiments.handheld_mapping.imu_subscriber import subscribe; '
            f'subscribe({str(target)!r}, {AXES!r})')
    process = subprocess.Popen([sys.executable, '-u', '-c', code], stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        assert json.loads(process.stdout.readline())['kind'] == 'ready'
        process.stdin.write('partial')
        process.stdin.flush()
        time.sleep(.3)
        process.terminate()
        assert process.wait(timeout=1) == 0
        assert not target.exists()
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        for handle in (process.stdin, process.stdout, process.stderr):
            handle.close()


def test_hz_uses_same_pi_receipt_clock_even_when_ssh_batches(tmp_path: Path):
    baseline = time.monotonic_ns()
    events = [ready()] + [packet(i * 25, i) for i in range(10)]
    for i, event in enumerate(events[1:]):
        event['host_monotonic_ns'] = baseline + i * 25_000_000
    imu = capture(tmp_path, events)
    imu.command[-1] = imu.command[-1].replace('time.sleep(.01)', 'time.sleep(0)')
    imu.start(timeout=1)
    state = wait_recorded(imu, 10)
    assert state['hz'] == pytest.approx(40.)
    assert state['hz_basis'] == 'pi_receipt_monotonic_unique_packets'
    assert state['imu_clock_verified'] is False
    imu.stop()


def test_bridge_counter_reset_fails_and_retains_offending_raw_packet(tmp_path: Path):
    first, second, reset = packet(10), packet(20), packet(40)
    first['tee_drops'] = second['tee_drops'] = 9000
    reset['tee_drops'] = 0
    imu = capture(tmp_path, [ready(), first, second, reset])
    imu.start(timeout=1)
    deadline = time.monotonic() + 1
    while imu.snapshot()['state'] != 'failed' and time.monotonic() < deadline:
        time.sleep(.01)
    assert imu.stop()['state'] == 'failed'
    rows = [json.loads(line) for line in (tmp_path / 'run/imu.jsonl').read_text().splitlines()]
    assert len(rows) == 3
    assert rows[-1]['raw'] == reset
    assert rows[-1]['acceptance'] == 'invalid_measurement'


def test_timeout_explains_receiver_connected_but_bridge_silent(tmp_path):
    imu=capture(tmp_path,[ready()])
    with pytest.raises(TimeoutError,match='데이터가 0건'):
        imu.start(timeout=.15)
    assert imu.process.poll() is not None


def test_timeout_distinguishes_invalid_flag_from_missing_bridge(tmp_path):
    imu=capture(tmp_path,[ready(),packet(10,flags=0),packet(20,flags=0)])
    with pytest.raises(TimeoutError,match='IMU_VALID'):
        imu.start(timeout=.15)
    assert imu.snapshot()['recorded']==2


def test_bridge_preflight_matches_exact_sender_arguments_only(tmp_path):
    from experiments.handheld_mapping.imu_health import bridge_processes
    for pid,args in [('101',['python3','bridge.py','--telemetry-socket','/tmp/imu.sock']),
                     ('102',['python3','subscriber.py','--socket','/tmp/imu.sock']),
                     ('103',['bash','-c','bridge.py --telemetry-socket /tmp/imu.sock']),
                     ('104',['python3','bridge.py','--telemetry-socket=/tmp/imu.sock'])]:
        folder=tmp_path/pid;folder.mkdir();(folder/'cmdline').write_bytes(('\0'.join(args)+'\0').encode())
    assert bridge_processes('/tmp/imu.sock',tmp_path)==[101,104]
    assert bridge_processes('/tmp/other.sock',tmp_path)==[]


def test_startup_failure_keeps_remote_bridge_diagnostic(tmp_path):
    imu=capture(tmp_path,[])
    imu.command=[sys.executable,'-u','-c',
        "import sys;print('IMU 브리지가 실행 중이 아닙니다',file=sys.stderr,flush=True);sys.exit(2)"]
    with pytest.raises(RuntimeError,match='IMU 브리지가 실행 중이 아닙니다'):
        imu.start(timeout=1)
    assert imu.process.poll() is not None
