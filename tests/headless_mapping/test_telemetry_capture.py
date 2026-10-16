# ================================================================
# test_telemetry_capture.py
# 설명: 비차단 telemetry 누락과 실제 uint8/uint32 시각 및 IMU 단위 계약을 검증한다.
# 사용법: /usr/bin/python3 -m pytest tests/headless_mapping
# ================================================================
import math
import socket
import time
from pathlib import Path
import pytest
from telemetry_tee import TelemetryTee
from ksrc_telemetry_capture import ImuClockAudit, imu_values


def test_spi_receipt_clock_preserves_raw_and_bracket(monkeypatch):
    import telemetry_tee
    monotonic = iter([1000, 1200])
    monkeypatch.setattr(telemetry_tee.time, 'monotonic_ns', lambda: next(monotonic))
    monkeypatch.setattr(telemetry_tee.time, 'time_ns', lambda: 10**18)
    def raw_clock(domain):
        assert domain == telemetry_tee.time.CLOCK_MONOTONIC_RAW
        return 900
    monkeypatch.setattr(telemetry_tee.time, 'clock_gettime_ns', raw_clock)
    assert telemetry_tee.receipt_clock() == {
        'host_receipt_ns': 10**18, 'host_monotonic_ns': 1100,
        'host_monotonic_raw_ns': 900, 'host_clock_sample_span_ns': 200}

def test_absent_consumer_no_wait(tmp_path):
    tee = TelemetryTee(tmp_path/'missing.sock')
    begin = time.monotonic()
    for _ in range(1000): assert not tee.emit({'schema':1,'telemetry':None})
    assert tee.dropped == 1000 and time.monotonic()-begin < .5
    tee.close()

def test_full_consumer_no_wait(tmp_path):
    path=tmp_path/'full.sock'
    receiver=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
    receiver.bind(str(path))
    tee=TelemetryTee(path)
    for _ in range(2000): tee.emit({'schema':1})
    assert tee.dropped > 0
    tee.close(); receiver.close()

def test_imu_units_and_flag():
    assert imu_values({'flags':0}) is None
    gyro,acc=imu_values({'flags':16,'gyro_dps':[180,-90,0],'acc_g':[0,0,1]})
    assert gyro[0] == pytest.approx(math.pi)
    assert gyro[1] == pytest.approx(-math.pi/2)
    assert acc[2] == pytest.approx(9.80665)

@pytest.mark.parametrize('before,after,status', [((255,100),(0,120),'ok'), ((1,100),(1,100),'duplicate'),
    ((1,100),(4,160),'gap'), ((3,1000),(0,0),'reset_or_reorder'), ((1,2**32-10),(2,10),'ok')])
def test_clock_wrapping(before,after,status):
    audit=ImuClockAudit(); audit.observe(*before)
    assert audit.observe(*after) == status

def test_invalid_imu_is_rejected():
    with pytest.raises(ValueError): imu_values({'flags':16,'gyro_dps':[float('nan'),0,0],'acc_g':[0,0,1]})
