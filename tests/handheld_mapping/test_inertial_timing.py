# ================================================================
# test_inertial_timing.py
# 설명: IMU 평균 읽기 시각과 Pi clock domain을 보존한 offline 시간 정렬을 검사한다.
# 사용법: PYTHONNOUSERSITE=1 .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_inertial_timing.py -q
# ================================================================
import csv
import json
from pathlib import Path

import numpy as np
import pytest


EPOCH = 1_790_000_000_000_000_000
RAW_ORIGIN = 5_000_000_000_000


def signal(t: np.ndarray) -> np.ndarray:
    return .15 + .08*np.sin(.7*t+.025*t*t) + .04*np.sin(2.9*t) + .02*np.cos(5.3*t)


def make_run(path: Path, *, stationary: bool = False, legacy: bool = False) -> Path:
    path.mkdir()
    (path/'orb_live').mkdir()
    imu = []
    for i in range(2500):
        t = i*.02
        receipt_ns = round((t+.02)*1e9)
        telemetry = {'imu_n': 4}
        if not legacy:
            telemetry.update(imu_timing_valid=True, imu_mean_read_offset_us=-10000,
                             imu_time_source='mcu_mean_sample_read')
        imu.append({'acceptance': 'accepted', 'clock': {
            'mcu_t_ms': 100000+i*20, 'segment': 0,
            'pi_receipt_monotonic_ns': RAW_ORIGIN+800_000_000_000+round((t+.02)*1.00005e9),
            'pi_receipt_realtime_ns': EPOCH+receipt_ns},
            'raw': {'host_monotonic_raw_ns': RAW_ORIGIN+receipt_ns, 'telemetry': telemetry},
            'calibrated': {'gyro_dps': [0., 0., 0. if stationary else float(np.degrees(signal(t-.01)))],
                           'acc_g': [0., 0., 1.]}})
    write_rows(path/'imu.jsonl', imu)
    frames = []
    dense = np.arange(0., 50., .0005)
    integral = np.r_[0., np.cumsum((signal(dense[:-1])+signal(dense[1:]))*.5*np.diff(dense))]
    fields = ['stamp_ns', 'state', 'map_id']+[f'm{i}{j}' for i in range(3) for j in range(3)]
    with (path/'orb_live/live_poses.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for t in np.arange(1., 49., 1/15):
            ns, receipt_ns = round(t*1e9), round((t+.04)*1e9)
            stamp = EPOCH+ns
            frames.append({'stream': 'left', 'stamp_ns': stamp,
                           'pi_clock_sample_span_ns': 2000,
                           'pi_receipt_monotonic_ns': RAW_ORIGIN+800_000_000_000+round((t+.04)*1.00005e9),
                           'pi_receipt_monotonic_raw_ns': RAW_ORIGIN+receipt_ns,
                           'pi_receipt_realtime_ns': EPOCH+receipt_ns})
            angle = 0. if stationary else np.interp(t, dense, integral)
            c, s = np.cos(angle), np.sin(angle)
            rotation = [[c, -s, 0.], [s, c, 0.], [0., 0., 1.]]
            writer.writerow({'stamp_ns': stamp, 'state': 2, 'map_id': 0,
                             **{f'm{i}{j}': rotation[i][j] for i in range(3) for j in range(3)}})
    write_rows(path/'frames.jsonl', frames)
    (path/'imu_summary.json').write_text(json.dumps({'state': 'stopped'}))
    return path


def write_rows(path: Path, rows: list[dict]) -> None:
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows))


def rows(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text().splitlines()]


def test_manual_correction_preserves_mean_offset_and_uses_raw_pairs(tmp_path: Path) -> None:
    from experiments.handheld_mapping.inertial_timing import aligned_inputs
    run = make_run(tmp_path/'run')
    before = {p: p.read_bytes() for p in run.rglob('*') if p.is_file()}
    data = aligned_inputs(run, correction_s=.025)
    assert data['camera_origin_ns'] == EPOCH+1_000_000_000
    assert data['imu_times_s'][0] == pytest.approx(-.965, abs=1e-7)
    assert data['camera_times_s'][int(data['camera_stamps_ns'][0])] == 0.
    assert data['camera_times_s'][int(data['camera_stamps_ns'][-1])] == pytest.approx(47.933333333, abs=1e-8)
    assert data['gyro'].shape == (2500, 3)
    assert data['acc'][0] == pytest.approx([0., 0., 9.80665])
    assert data['report']['clock_domain'] == 'CLOCK_MONOTONIC_RAW'
    assert data['report']['correction_source'] == 'experimental_manual'
    assert data['report']['clock_verified'] is False
    assert data['report']['imu_time_source'] == 'mcu_mean_sample_read'
    assert data['report']['input_sha256']['imu.jsonl']
    assert all(p.read_bytes() == value for p, value in before.items())


def test_auto_estimate_removes_receipt_delay_without_absorbing_mean_offset(tmp_path: Path) -> None:
    from experiments.handheld_mapping.inertial_timing import aligned_inputs
    data = aligned_inputs(make_run(tmp_path/'run'))
    assert data['report']['imu_time_correction_s'] == pytest.approx(-.02, abs=.006)
    assert data['imu_times_s'][0] == pytest.approx(-1.01, abs=.006)
    assert data['report']['correction_source'] == 'visual_rotation_correlation'
    assert data['report']['clock_verified'] is False


@pytest.mark.parametrize('missing', ['imu', 'camera'])
def test_incomplete_raw_pairs_fall_back_consistently(tmp_path: Path, missing: str) -> None:
    from experiments.handheld_mapping.inertial_timing import aligned_inputs
    run = make_run(tmp_path/'run')
    path = run/('imu.jsonl' if missing == 'imu' else 'frames.jsonl')
    values = rows(path)
    if missing == 'imu':
        values[0]['raw'].pop('host_monotonic_raw_ns')
    else:
        values[0].pop('pi_receipt_monotonic_raw_ns')
    write_rows(path, values)
    data = aligned_inputs(run, correction_s=0.)
    assert data['report']['clock_domain'] == 'CLOCK_MONOTONIC'
    assert data['report']['clock_fit']['scale'] == pytest.approx(1.00005)
    assert data['imu_times_s'][0] == pytest.approx(-.9900515, abs=2e-7)


def test_legacy_requires_opt_in_and_reports_control_time_confidence(tmp_path: Path) -> None:
    from experiments.handheld_mapping.inertial_timing import aligned_inputs
    run = make_run(tmp_path/'run', legacy=True)
    with pytest.raises(ValueError, match='legacy'):
        aligned_inputs(run, correction_s=0.)
    data = aligned_inputs(run, correction_s=0., allow_legacy=True)
    assert data['report']['imu_time_source'] == 'legacy_mcu_control_tick'
    assert data['report']['confidence'] == 'legacy_control_time_unverified'
    assert data['imu_times_s'][0] == pytest.approx(-.98, abs=1e-7)


@pytest.mark.parametrize('change', ['invalid_flag', 'missing_offset', 'wrong_source', 'offset_range', 'bool_offset', 'partial_new'])
def test_new_invalid_timing_cannot_be_hidden_by_legacy_flag(tmp_path: Path, change: str) -> None:
    from experiments.handheld_mapping.inertial_timing import aligned_inputs
    run = make_run(tmp_path/'run')
    values = rows(run/'imu.jsonl')
    item = values[0]['raw']['telemetry']
    if change == 'invalid_flag': item['imu_timing_valid'] = False
    elif change == 'missing_offset': item.pop('imu_mean_read_offset_us')
    elif change == 'wrong_source': item['imu_time_source'] = 'unknown'
    elif change == 'offset_range': item['imu_mean_read_offset_us'] = -32769
    elif change == 'bool_offset': item['imu_mean_read_offset_us'] = True
    elif change == 'partial_new': item.clear()
    write_rows(run/'imu.jsonl', values)
    with pytest.raises(ValueError, match='timing'):
        aligned_inputs(run, correction_s=0., allow_legacy=True)


def test_stationary_requires_explicit_correction_for_pipeline_smoke(tmp_path: Path) -> None:
    from experiments.handheld_mapping.inertial_timing import aligned_inputs
    run = make_run(tmp_path/'run', stationary=True)
    with pytest.raises(ValueError, match='insufficient_motion'):
        aligned_inputs(run)
    assert aligned_inputs(run, correction_s=0.)['report']['clock_verified'] is False


@pytest.mark.parametrize('change', ['reset', 'segment', 'duplicate', 'nonmonotonic_receipt', 'gap', 'clock_jump', 'unstable', 'nonfinite'])
def test_bad_clock_or_measurement_rejected_even_with_manual_correction(tmp_path: Path, change: str) -> None:
    from experiments.handheld_mapping.inertial_timing import aligned_inputs
    run = make_run(tmp_path/'run')
    values = rows(run/'imu.jsonl')
    if change == 'reset': values[30]['acceptance'] = 'reset_boundary_discard'
    elif change == 'segment': values[30]['clock']['segment'] = 1
    elif change == 'duplicate': values[30]['clock']['mcu_t_ms'] = values[29]['clock']['mcu_t_ms']
    elif change == 'nonmonotonic_receipt': values[30]['raw']['host_monotonic_raw_ns'] = 1
    elif change == 'gap': values = values[:30]+values[40:]
    elif change == 'clock_jump': values[-1]['clock']['pi_receipt_realtime_ns'] += 10_000_000
    elif change == 'unstable':
        for i, row in enumerate(values):
            row['raw']['host_monotonic_raw_ns'] += round(.06*np.sin(2*np.pi*i/2500)*1e9)
    elif change == 'nonfinite': values[0]['calibrated']['gyro_dps'][0] = float('nan')
    write_rows(run/'imu.jsonl', values)
    with pytest.raises(ValueError):
        aligned_inputs(run, correction_s=0.)


@pytest.mark.parametrize('correction', [float('nan'), float('inf'), True, 1.])
def test_invalid_manual_correction_is_rejected(tmp_path: Path, correction: float) -> None:
    from experiments.handheld_mapping.inertial_timing import aligned_inputs
    with pytest.raises(ValueError, match='correction'):
        aligned_inputs(make_run(tmp_path/'run'), correction_s=correction)


def test_camera_nonmonotonic_stamps_and_epoch_jump_are_rejected(tmp_path: Path) -> None:
    from experiments.handheld_mapping.inertial_timing import aligned_inputs
    run = make_run(tmp_path/'run')
    values = rows(run/'frames.jsonl')
    values[1]['stamp_ns'] = values[0]['stamp_ns']
    write_rows(run/'frames.jsonl', values)
    with pytest.raises(ValueError):
        aligned_inputs(run, correction_s=0.)


def test_uint32_wrap_is_preserved_but_reset_is_not(tmp_path: Path) -> None:
    from experiments.handheld_mapping.inertial_timing import aligned_inputs
    run = make_run(tmp_path/'run')
    values = rows(run/'imu.jsonl')
    for i, row in enumerate(values): row['clock']['mcu_t_ms'] = (2**32-1000+i*20) % 2**32
    write_rows(run/'imu.jsonl', values)
    assert aligned_inputs(run, correction_s=0.)['imu_times_s'][0] == pytest.approx(-.99, abs=1e-7)


def test_raw_clock_allows_smooth_realtime_slew_without_losing_camera_time(tmp_path: Path) -> None:
    from experiments.handheld_mapping.inertial_timing import aligned_inputs
    run = make_run(tmp_path/'run', stationary=True)
    imu = rows(run/'imu.jsonl')
    for row in imu:
        elapsed_ns = row['raw']['host_monotonic_raw_ns']-RAW_ORIGIN
        row['clock']['pi_receipt_realtime_ns'] = EPOCH+round(elapsed_ns*1.0003)
    write_rows(run/'imu.jsonl', imu)
    frames = rows(run/'frames.jsonl')
    for row in frames:
        elapsed_ns = row['pi_receipt_monotonic_raw_ns']-RAW_ORIGIN
        row['pi_receipt_realtime_ns'] = EPOCH+round(elapsed_ns*1.0003)
        row['stamp_ns'] = EPOCH+round((row['stamp_ns']-EPOCH)*1.0003)
    write_rows(run/'frames.jsonl', frames)
    data = aligned_inputs(run, correction_s=0.)
    assert data['report']['clock_domain'] == 'CLOCK_MONOTONIC_RAW'
    # Callback에서 얻은 realtime−RAW 차를 각 header에 적용하면 300ppm drift가 사라진다.
    assert data['camera_times_s'][int(data['camera_stamps_ns'][-1])] == pytest.approx(47.933333333, abs=2e-8)
    assert data['imu_times_s'][0] == pytest.approx(-.989988, abs=1e-7)


def test_raw_realtime_step_is_rejected_even_when_header_epoch_changes_with_it(tmp_path: Path) -> None:
    from experiments.handheld_mapping.inertial_timing import aligned_inputs
    run = make_run(tmp_path/'run', stationary=True)
    imu = rows(run/'imu.jsonl')
    for row in imu:
        if row['raw']['host_monotonic_raw_ns'] >= RAW_ORIGIN+25_000_000_000:
            row['clock']['pi_receipt_realtime_ns'] += 10_000_000
    write_rows(run/'imu.jsonl', imu)
    frames = rows(run/'frames.jsonl')
    for row in frames:
        if row['pi_receipt_monotonic_raw_ns'] >= RAW_ORIGIN+25_000_000_000:
            row['pi_receipt_realtime_ns'] += 10_000_000
            row['stamp_ns'] += 10_000_000
    write_rows(run/'frames.jsonl', frames)
    with pytest.raises(ValueError, match='clock relation changed'):
        aligned_inputs(run, correction_s=0.)


@pytest.mark.parametrize('segment, sample_count, first_time, last_time', [
    (0, 1250, -.99, 23.99), (1, 1249, 24.03, 48.99)])
def test_explicit_segment_never_integrates_across_an_mcu_reset(
        tmp_path: Path, segment: int, sample_count: int, first_time: float, last_time: float) -> None:
    from experiments.handheld_mapping.inertial_timing import aligned_inputs
    run = make_run(tmp_path/'run')
    imu = rows(run/'imu.jsonl')
    imu[1250]['acceptance'] = 'reset_boundary_discard'
    for i, row in enumerate(imu[1250:], 1250):
        row['clock']['segment'] = 1
        row['clock']['mcu_t_ms'] = 1000+(i-1250)*20
    write_rows(run/'imu.jsonl', imu)
    before = (run/'imu.jsonl').read_bytes()
    with pytest.raises(ValueError, match='reset'):
        aligned_inputs(run, correction_s=0.)
    data = aligned_inputs(run, correction_s=0., segment=segment)
    assert data['report']['selected_segment'] == segment
    assert data['report']['discarded_imu_rows'] == 2500-sample_count
    assert data['report']['imu_selection_discard_counts']['reset_boundary_discard'] == 1
    assert len(data['imu_times_s']) == sample_count
    assert data['imu_times_s'][0] == pytest.approx(first_time, abs=1e-7)
    assert data['imu_times_s'][-1] == pytest.approx(last_time, abs=1e-7)
    assert np.max(np.diff(data['imu_times_s'])) < .021
    assert len(data['camera_stamps_ns']) == 720
    assert (run/'imu.jsonl').read_bytes() == before


@pytest.mark.parametrize('segment', [-1, True, .5, '1'])
def test_segment_must_be_a_nonnegative_integer(tmp_path: Path, segment: object) -> None:
    from experiments.handheld_mapping.inertial_timing import aligned_inputs
    with pytest.raises(ValueError, match='segment'):
        aligned_inputs(make_run(tmp_path/'run'), correction_s=0., segment=segment)


def test_requested_segment_without_accepted_samples_is_rejected(tmp_path: Path) -> None:
    from experiments.handheld_mapping.inertial_timing import aligned_inputs
    with pytest.raises(ValueError, match='samples'):
        aligned_inputs(make_run(tmp_path/'run'), correction_s=0., segment=1)


@pytest.mark.parametrize('window', [(1., 1.), (2., 1.), (float('nan'), 2.),
                                     (0., float('inf')), (True, 2.), (0.,), '0,2'])
def test_correlation_window_requires_two_finite_increasing_values(tmp_path: Path, window: object) -> None:
    from experiments.handheld_mapping.inertial_timing import aligned_inputs
    with pytest.raises(ValueError, match='correlation_window_s'):
        aligned_inputs(make_run(tmp_path/'run'), correction_s=0., correlation_window_s=window)


def test_motion_window_estimates_offset_without_discarding_stationary_imu_padding(tmp_path: Path) -> None:
    from experiments.handheld_mapping.inertial_timing import aligned_inputs
    run = make_run(tmp_path/'run')
    imu = rows(run/'imu.jsonl')
    for row in imu:
        sample_t = (row['clock']['mcu_t_ms']-100000)*.001-.01
        if not 34. <= sample_t < 49.:
            row['calibrated']['gyro_dps'] = [0., 0., 0.]
    write_rows(run/'imu.jsonl', imu)
    dense = np.arange(0., 50., .0005)
    rate = np.where((dense >= 34.) & (dense < 49.), signal(dense), 0.)
    integral = np.r_[0., np.cumsum((rate[:-1]+rate[1:])*.5*np.diff(dense))]
    path = run/'orb_live/live_poses.csv'
    with path.open() as stream:
        reader = csv.DictReader(stream)
        fields, poses = reader.fieldnames, list(reader)
    for row in poses:
        angle = np.interp((int(row['stamp_ns'])-EPOCH)*1e-9, dense, integral)
        c, s = np.cos(angle), np.sin(angle)
        rotation = [[c, -s, 0.], [s, c, 0.], [0., 0., 1.]]
        row.update({f'm{i}{j}': rotation[i][j] for i in range(3) for j in range(3)})
    with path.open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(poses)
    with pytest.raises(ValueError, match='insufficient_half_motion'):
        aligned_inputs(run)
    # camera origin은 t=1s이므로 물리시간 [35,48]s 운동 구간은 상대시간 [34,47]s이다.
    data = aligned_inputs(run, correlation_window_s=(34., 47.))
    assert data['report']['imu_time_correction_s'] == pytest.approx(-.02, abs=.006)
    assert data['report']['correlation_window_s'] == [34., 47.]
    assert data['report']['clock_verified'] is False
    assert len(data['imu_times_s']) == 2500
    assert data['imu_times_s'][0] < -1.
    comparison = data['report']['offset_estimate']['comparison_visual_times_s']
    assert min(comparison) > 34. and max(comparison) < 47.


def test_stationary_correlation_window_does_not_bypass_motion_guard(tmp_path: Path) -> None:
    from experiments.handheld_mapping.inertial_timing import aligned_inputs
    with pytest.raises(ValueError, match='insufficient_motion'):
        aligned_inputs(make_run(tmp_path/'run', stationary=True), correlation_window_s=(10., 30.))
