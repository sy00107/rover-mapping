# ================================================================
# test_time_alignment.py
# 설명: 알려진 시계 드리프트·시차를 복구하고 정지·모호함·공백을 거부하는지 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_time_alignment.py -q
# ================================================================
import numpy as np
import pytest
import csv
import hashlib
import json


def signal(t: np.ndarray) -> np.ndarray:
    return .15 + .08 * np.sin(.7 * t + .025 * t * t) + .04 * np.sin(2.9 * t) + .02 * np.cos(5.3 * t)


def samples(lag: float = .085) -> tuple:
    t = np.arange(0., 50., .02)
    rate = signal(t - lag)
    start = np.arange(1., 49., 1 / 15)
    end = start + 1 / 15
    # 2kHz independent integration oracle for each visual interval.
    dense = np.linspace(0, 1, 135)
    visual = np.trapz(signal(start[:, None] + (end-start)[:, None]*dense), dense, axis=1)
    return t, rate, start, end, visual


@pytest.mark.parametrize('lag', [.085, -.125])
def test_recovers_known_nonperiodic_time_shift_with_correct_sign(lag: float):
    from experiments.handheld_mapping.time_alignment import estimate_offset
    result = estimate_offset(*samples(lag))
    assert result['status'] == 'estimated', result
    assert result['imu_time_correction_s'] == pytest.approx(-lag, abs=.01)
    assert result['correlation'] > .99
    assert result['imu_clock_verified'] is False


def test_corrects_clock_rate_even_when_pi_receipts_are_late():
    from experiments.handheld_mapping.time_alignment import fit_clock
    mcu = np.arange(100000, 220000, 20, dtype=np.int64)
    physical = (mcu-mcu[0])*.001*.9935
    # Receipt order must remain physical: jitter smaller than the 20ms sampling interval.
    lag = np.random.default_rng(4).uniform(0, .015, len(mcu))
    pi = 3_000_000_000_000 + np.rint((physical+lag)*1e9).astype(np.int64)
    result = fit_clock(mcu, pi)
    assert result['scale'] == pytest.approx(.9935, abs=2e-5)
    assert result['rate_difference_ppm'] == pytest.approx(-6500, abs=20)
    assert result['absolute_receipt_delay_known'] is False
    assert result['receipt_residual_trend_suspect'] is False


def test_clock_fit_flags_slow_residual_trend_instead_of_treating_it_as_jitter():
    from experiments.handheld_mapping.time_alignment import fit_clock
    mcu = 100000 + np.arange(6000, dtype=np.int64)*20
    seconds = (mcu-mcu[0])*.001
    delay = .060*np.sin(2*np.pi*seconds/120)
    jitter = np.random.default_rng(7).uniform(0, .015, len(mcu))
    pi = 3_000_000_000_000+np.rint((seconds+delay+jitter)*1e9).astype(np.int64)
    result = fit_clock(mcu, pi)
    assert result['receipt_residual_trend_suspect'] is True
    assert result['receipt_block_median_range_ms'] > 30


def test_short_clock_fit_does_not_claim_trend_was_checked():
    from experiments.handheld_mapping.time_alignment import fit_clock
    mcu = np.arange(100000, 102000, 20, dtype=np.int64)
    pi = 3_000_000_000_000 + (mcu-mcu[0])*1_000_000
    result = fit_clock(mcu, pi)
    assert result['receipt_residual_trend_suspect'] is None


def test_accepts_uint32_wrap_but_rejects_mcu_reset():
    from experiments.handheld_mapping.time_alignment import fit_clock
    unwrapped = 2**32 - 1000 + np.arange(2000, dtype=np.int64)*20
    pi = 3_000_000_000_000 + np.arange(2000, dtype=np.int64)*20_000_000
    assert fit_clock(unwrapped % 2**32, pi)['scale'] == pytest.approx(1.)
    reset = np.arange(2000, dtype=np.int64)*20 + 100000
    reset[1000:] -= 110000
    with pytest.raises(ValueError, match='reset'):
        fit_clock(reset, pi)


def test_stationary_data_has_no_invented_time_offset():
    from experiments.handheld_mapping.time_alignment import estimate_offset
    t, g, start, end, visual = samples()
    result = estimate_offset(t, np.zeros_like(g), start, end, np.zeros_like(visual))
    assert result['status'] == 'rejected'
    assert 'insufficient_motion' in result['reasons']
    assert result['imu_time_correction_s'] is None


def test_periodic_rotation_reports_ambiguous_alignment():
    from experiments.handheld_mapping.time_alignment import estimate_offset
    t, _, start, end, _ = samples()
    omega = lambda x: .2+.1*np.sin(2*np.pi*x/.3)
    dense = np.linspace(0, 1, 135)
    visual = np.trapz(omega(start[:, None]+(end-start)[:, None]*dense), dense, axis=1)
    result = estimate_offset(t, omega(t-.08), start, end, visual)
    assert result['status'] == 'rejected', result
    assert 'ambiguous_peak' in result['reasons']


@pytest.mark.parametrize('lag', [-.48, .085])
def test_static_half_has_no_invented_peak_for_either_shift_sign(lag: float):
    from experiments.handheld_mapping.time_alignment import estimate_offset
    t, _, start, end, _ = samples()
    moving = lambda x: np.where(x < 24, signal(x), .001)
    dense = np.linspace(0, 1, 135)
    visual = np.trapz(moving(start[:, None]+(end-start)[:, None]*dense), dense, axis=1)
    result = estimate_offset(t, moving(t-lag), start, end, visual)
    assert result['status'] == 'rejected', result
    assert 'insufficient_half_motion' in result['reasons']
    assert result['half_best_shift_s'][1] is None
    assert result['imu_time_correction_s'] is None


def test_no_interpolation_across_large_imu_gaps():
    from experiments.handheld_mapping.time_alignment import estimate_offset
    t, g, start, end, visual = samples()
    valid = (t < 10) | (t > 40)
    result = estimate_offset(t[valid], g[valid], start, end, visual)
    assert result['excluded_gap_intervals'] > 400
    assert result['status'] == 'estimated', result
    assert result['imu_time_correction_s'] == pytest.approx(-.085, abs=.01)


def test_boundary_peak_is_not_accepted():
    from experiments.handheld_mapping.time_alignment import estimate_offset
    result = estimate_offset(*samples(.7))
    assert result['status'] == 'rejected'
    assert result['imu_time_correction_s'] is None


def test_pi_receipt_clock_is_bracketed_and_midpoint_used(monkeypatch):
    from experiments.handheld_mapping import pi_worker
    stamps = iter([1000, 1200])
    monkeypatch.setattr(pi_worker.time, 'monotonic_ns', lambda: next(stamps))
    monkeypatch.setattr(pi_worker.time, 'time_ns', lambda: 9_000_000)
    monkeypatch.setattr(pi_worker.time, 'clock_gettime_ns', lambda _: 8000)
    result = pi_worker.receipt_clock()
    assert result == {'pi_receipt_monotonic_ns': 1100,
                      'pi_receipt_realtime_ns': 9_000_000, 'pi_clock_sample_span_ns': 200,
                      'pi_receipt_monotonic_raw_ns': 8000}


def make_run(path, stationary=False, legacy=False):
    path.mkdir()
    (path/'orb_live').mkdir()
    t, g, _, _, _ = samples()
    epoch, pi_origin = 1_790_000_000_000_000_000, 3_000_000_000_000
    with (path/'imu.jsonl').open('w') as f:
        for seconds, speed in zip(t, g):
            ns = round(seconds*1e9)
            row = {'acceptance':'accepted', 'clock':{'mcu_t_ms':100000+round(seconds*1000),
                   'pi_receipt_monotonic_ns':pi_origin+ns, 'pi_receipt_realtime_ns':epoch+ns, 'segment':0},
                   'calibrated':{'gyro_dps':[0,0,0 if stationary else float(np.degrees(speed))],
                                 'acc_g':[0,0,1]}, 'raw':{'telemetry':{'imu_n':4}}}
            f.write(json.dumps(row)+'\n')
    dense=np.arange(0,50,.0005)
    integral=np.r_[0,np.cumsum((signal(dense[:-1])+signal(dense[1:]))*.5*np.diff(dense))]
    fields=['stamp_ns','state','map_id']+[f'm{i}{j}' for i in range(4) for j in range(4)]
    with (path/'frames.jsonl').open('w') as frames, (path/'orb_live/live_poses.csv').open('w') as poses:
        writer=csv.DictWriter(poses,fieldnames=fields);writer.writeheader()
        for seconds in np.arange(1,49,1/15):
            ns=round(seconds*1e9)
            row={'stream':'left','stamp_ns':epoch+ns,'pi_receipt_monotonic_ns':pi_origin+ns+40_000_000}
            if not legacy: row.update(pi_receipt_realtime_ns=epoch+ns+40_000_000,pi_clock_sample_span_ns=1000)
            frames.write(json.dumps(row)+'\n')
            angle=0 if stationary else np.interp(seconds,dense,integral)
            c,s=np.cos(angle),np.sin(angle)
            matrix=np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]])
            writer.writerow({'stamp_ns':epoch+ns,'state':2,'map_id':0,
                             **{f'm{i}{j}':matrix[i,j] for i in range(4) for j in range(4)}})
    (path/'imu_summary.json').write_text(json.dumps({'state':'stopped'}))
    return path


@pytest.mark.parametrize('legacy',[False,True])
def test_run_analysis_preserves_inputs_and_exports_only_unverified_candidate(tmp_path,legacy):
    from experiments.handheld_mapping.time_alignment import analyze_run
    run=make_run(tmp_path/'run',legacy=legacy)
    hashes={p:hashlib.sha256(p.read_bytes()).hexdigest() for p in run.rglob('*') if p.is_file()}
    result=analyze_run(run)
    assert result['status']=='estimated',result
    assert result['imu_time_correction_s']==pytest.approx(-.085,abs=.01)
    assert result['imu_clock_verified'] is False and result['orb_fused'] is False
    saved=json.loads((run/'imu_time_alignment/summary.json').read_text())
    assert saved['status']=='estimated'
    assert (run/'imu_time_alignment/imu_alignment_candidate.csv').is_file()
    assert (run/'imu_time_alignment/README.md').is_file()
    assert all(hashlib.sha256(p.read_bytes()).hexdigest()==h for p,h in hashes.items())
    with pytest.raises(FileExistsError): analyze_run(run)


def test_static_run_reports_rejection_without_exporting_candidate(tmp_path):
    from experiments.handheld_mapping.time_alignment import analyze_run
    run=make_run(tmp_path/'run',stationary=True)
    result=analyze_run(run)
    assert result['status']=='rejected'
    assert 'insufficient_motion' in result['reasons']
    assert not (run/'imu_time_alignment/imu_alignment_candidate.csv').exists()


def test_changing_clock_relation_blocks_candidate_export(tmp_path):
    from experiments.handheld_mapping.time_alignment import analyze_run
    run = make_run(tmp_path/'run')
    path = run/'imu.jsonl'
    rows = [json.loads(line) for line in path.open()]
    for row in rows:
        clock = row['clock']
        seconds = (clock['mcu_t_ms']-100000)*.001
        clock['mcu_t_ms'] += round(60*np.sin(2*np.pi*seconds/50))
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    result = analyze_run(run)
    assert result['status'] == 'rejected'
    assert 'clock_model_unstable' in result['reasons']
    assert result['imu_time_correction_s'] is None
    assert not (run/'imu_time_alignment/imu_alignment_candidate.csv').exists()


def test_recording_in_progress_is_not_analyzed(tmp_path):
    from experiments.handheld_mapping.time_alignment import analyze_run
    run=make_run(tmp_path/'run')
    (run/'imu_summary.json').unlink()
    with pytest.raises(ValueError,match='stop'): analyze_run(run)
    assert not (run/'imu_time_alignment').exists()


def test_optional_averaging_count_missing_does_not_destroy_report(tmp_path):
    from experiments.handheld_mapping.time_alignment import analyze_run
    run=make_run(tmp_path/'run')
    path=run/'imu.jsonl'
    rows=[json.loads(line) for line in path.open()]
    for row in rows: row['raw']['telemetry']['imu_n']=None
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    result=analyze_run(run)
    assert result['status']=='estimated',result
    assert result['imu_n_median'] is None


def test_camera_clock_jump_is_rejected_without_candidate(tmp_path):
    from experiments.handheld_mapping.time_alignment import analyze_run
    run=make_run(tmp_path/'run')
    path=run/'frames.jsonl'
    rows=[json.loads(line) for line in path.open()]
    rows[-1]['pi_receipt_realtime_ns']+=10_000_000
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    result=analyze_run(run)
    assert result['status']=='rejected'
    assert 'clock relation changed' in result['reasons'][0]
    assert not (run/'imu_time_alignment/imu_alignment_candidate.csv').exists()
