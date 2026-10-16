"""Exercise host clock recovery against known clocks and invalid support."""
from pathlib import Path

import numpy as np
import pytest

from experiments.handheld_mapping.clock_recovery import recover_imu_raw_clock
from experiments.handheld_mapping.inertial_timing import aligned_inputs
from test_inertial_timing import make_run, rows, write_rows


def remove_imu_raw(run: Path) -> list[dict]:
    values = rows(run/'imu.jsonl')
    for row in values:
        row['raw'].pop('host_monotonic_raw_ns')
    write_rows(run/'imu.jsonl', values)
    return values


def test_recovery_matches_measured_raw_without_extrapolation_or_raw_changes(tmp_path):
    run = make_run(tmp_path/'run')
    measured = aligned_inputs(run, correction_s=0.)
    imu = remove_imu_raw(run)
    frames = rows(run/'frames.jsonl')
    before = {p: p.read_bytes() for p in run.rglob('*') if p.is_file()}
    inside, _, report = recover_imu_raw_clock(imu, frames)
    recovered = aligned_inputs(run, correction_s=0., recover_camera_clock=True)
    assert recovered['imu_times_s'] == pytest.approx(measured['imu_times_s'][inside], abs=2e-8)
    assert recovered['gyro'] == pytest.approx(measured['gyro'][inside])
    assert recovered['camera_times_s'] == measured['camera_times_s']
    assert report['trimmed_imu_samples'] > 0
    assert report['extrapolated_samples'] == 0
    assert recovered['report']['clock_recovery']['imu_raw_clock_measured'] is False
    assert recovered['report']['clock_verified'] is False
    assert recovered['report']['clock_fit']['receipt_residual_trend_suspect'] is False
    automatic = aligned_inputs(run, recover_camera_clock=True)
    assert automatic['report']['imu_time_correction_s'] == pytest.approx(-.02, abs=.006)
    assert automatic['report']['offset_estimate']['correlation'] > .99
    assert all(p.read_bytes() == content for p, content in before.items())


def test_opt_in_does_not_replace_measured_raw(tmp_path):
    data = aligned_inputs(make_run(tmp_path/'run'), correction_s=0., recover_camera_clock=True)
    assert data['report']['clock_recovery'] is None
    assert len(data['imu_times_s']) == 2500


@pytest.mark.parametrize('change', ['raw_missing', 'partial_imu_raw', 'bool_stamp', 'wide_bracket',
                                    'gap', 'reversed', 'step', 'different_pi', 'no_overlap'])
def test_bad_clock_support_is_rejected(tmp_path, change):
    run = make_run(tmp_path/'run')
    imu = remove_imu_raw(run)
    frames = rows(run/'frames.jsonl')
    if change == 'raw_missing': frames[10].pop('pi_receipt_monotonic_raw_ns')
    elif change == 'partial_imu_raw': imu[10]['raw']['host_monotonic_raw_ns'] = 1
    elif change == 'bool_stamp': frames[10]['pi_receipt_monotonic_ns'] = True
    elif change == 'wide_bracket': frames[10]['pi_clock_sample_span_ns'] = 200_000
    elif change == 'gap': frames = frames[:100]+frames[120:]
    elif change == 'reversed': frames[10]['pi_receipt_monotonic_raw_ns'] = frames[9]['pi_receipt_monotonic_raw_ns']
    elif change == 'step':
        for row in frames[100:]: row['pi_receipt_monotonic_raw_ns'] += 10_000_000
    elif change == 'different_pi':
        for row in imu: row['clock']['pi_receipt_realtime_ns'] += 10_000_000
    elif change == 'no_overlap':
        for row in imu:
            row['clock']['pi_receipt_monotonic_ns'] += 60_000_000_000
            row['clock']['pi_receipt_realtime_ns'] += 60_000_000_000
    write_rows(run/'imu.jsonl', imu)
    write_rows(run/'frames.jsonl', frames)
    with pytest.raises(ValueError):
        aligned_inputs(run, correction_s=0., recover_camera_clock=True)


def test_recovery_cannot_hide_unstable_mcu_clock(tmp_path):
    run = make_run(tmp_path/'run')
    imu = remove_imu_raw(run)
    for i, row in enumerate(imu):
        row['clock']['mcu_t_ms'] += round(60*np.sin(2*np.pi*i/len(imu)))
    write_rows(run/'imu.jsonl', imu)
    with pytest.raises(ValueError, match='clock_model_unstable'):
        aligned_inputs(run, correction_s=0., recover_camera_clock=True)


def test_cross_validation_rejects_interpolation_error_even_below_rate_limit():
    origin = 10**12
    frames = []
    for i in range(80):
        mono = origin+i*500_000_000
        frames.append({'pi_receipt_monotonic_ns': mono,
                       'pi_receipt_monotonic_raw_ns': mono-800_000*(i % 2),
                       'pi_receipt_realtime_ns': mono+10**18, 'pi_clock_sample_span_ns': 1000})
    imu = [{'clock': {'pi_receipt_monotonic_ns': origin+i*20_000_000,
                      'pi_receipt_realtime_ns': origin+i*20_000_000+10**18}} for i in range(2000)]
    with pytest.raises(ValueError, match='held-out error'):
        recover_imu_raw_clock(imu, frames)
