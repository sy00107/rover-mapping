"""Recover a Pi host clock domain from measured camera clock pairs, never sensor time."""
import numpy as np


def _ns(value: object) -> int:
    if type(value) is not int or not 0 <= value < 2**63:
        raise ValueError('invalid camera clock recovery timestamp')
    return value


def recover_imu_raw_clock(imu: list[dict], frames: list[dict]) -> tuple[np.ndarray, np.ndarray, dict]:
    """Interpolate MONOTONIC→RAW only inside dense, stable, same-Pi camera support.

    This recovers a host clock conversion, not an IMU acquisition timestamp. It
    cannot remove MCU drift or receipt delay; the usual fit/correlation guards
    must still run on the recovered input.
    """
    if any(row.get('raw', {}).get('host_monotonic_raw_ns') is not None for row in imu):
        raise ValueError('camera clock recovery requires all IMU RAW timestamps to be absent')
    if len(frames) < 40:
        raise ValueError('insufficient camera clock recovery pairs')
    mono = np.array([_ns(row.get('pi_receipt_monotonic_ns')) for row in frames], dtype=np.int64)
    raw = np.array([_ns(row.get('pi_receipt_monotonic_raw_ns')) for row in frames], dtype=np.int64)
    realtime = np.array([_ns(row.get('pi_receipt_realtime_ns')) for row in frames], dtype=np.int64)
    spans = np.array([_ns(row.get('pi_clock_sample_span_ns')) for row in frames], dtype=np.int64)
    gaps = np.diff(mono)
    if np.any(gaps <= 0) or np.any(np.diff(raw) <= 0) or mono[-1]-mono[0] < 10_000_000_000:
        raise ValueError('nonmonotonic or insufficient camera clock recovery support')
    if max(spans) > 100_000 or max(gaps) > 1_000_000_000:
        raise ValueError('camera clock recovery bracket or gap too large')
    delta = mono-raw
    # Bounds protect interpolation across a discontinuity, not sensor sync accuracy.
    rates = np.abs(np.diff(delta)/gaps)
    if max(np.abs(np.diff(delta))) > 1_000_000 or max(rates) > .002:
        raise ValueError('camera clock recovery relation step or rate too large')
    x = (mono-mono[0])*1e-9
    held = np.arange(1, len(mono)-1, 2)
    anchors = np.arange(0, len(mono), 2)
    residual = np.abs(np.interp(x[held], x[anchors], delta[anchors])-delta[held])
    if max(residual) > 250_000:
        raise ValueError('camera clock recovery held-out error exceeds 250us')
    receipt = np.array([_ns(row['clock']['pi_receipt_monotonic_ns']) for row in imu], dtype=np.int64)
    imu_realtime = np.array([_ns(row['clock']['pi_receipt_realtime_ns']) for row in imu], dtype=np.int64)
    if np.any(np.diff(receipt) <= 0):
        raise ValueError('nonmonotonic IMU clock recovery receipts')
    offsets = np.r_[realtime-mono, imu_realtime-receipt]
    if np.ptp(offsets) > 5_000_000:
        raise ValueError('camera/IMU Pi clock relation incompatible for recovery')
    inside = (receipt >= mono[0]) & (receipt <= mono[-1])
    if np.count_nonzero(inside) < 20:
        raise ValueError('insufficient IMU clock recovery overlap')
    correction = np.interp((receipt[inside]-mono[0])*1e-9, x, delta)
    recovered = receipt[inside]-np.rint(correction).astype(np.int64)
    if np.any(np.diff(recovered) <= 0):
        raise ValueError('nonmonotonic recovered IMU host clock')
    return inside, recovered, {
        'method': 'camera_host_MONOTONIC_to_RAW_interpolation',
        'imu_raw_clock_measured': False, 'sensor_acquisition_time_verified': False,
        'assumption': 'camera and IMU receipt clocks originate on the same Pi boot',
        'extrapolated_samples': 0, 'camera_clock_pairs': len(frames),
        'trimmed_imu_samples': int(np.count_nonzero(~inside)),
        'camera_support_monotonic_ns': [int(mono[0]), int(mono[-1])],
        'max_camera_gap_ms': float(max(gaps)*1e-6),
        'max_clock_bracket_ns': int(max(spans)),
        'max_relation_rate_ppm': float(max(rates)*1e6),
        'held_out_error_p50_p95_max_us': (np.percentile(residual, [50, 95, 100])*1e-3).tolist(),
        'quality_guards_relaxed': False,
    }
