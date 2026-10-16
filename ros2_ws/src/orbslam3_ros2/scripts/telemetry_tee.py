# ================================================================
# telemetry_tee.py
# 설명: SPI owner가 이미 디코딩한 telemetry를 UNIX datagram으로 비차단 분기한다.
# 사용법: tee = TelemetryTee(path); tee.emit(payload); tee.close()
# ================================================================
import json
import socket
import time
from pathlib import Path


def receipt_clock() -> dict:
    """Preserve the SPI receipt's Pi host clocks in one short bracket."""
    before = time.monotonic_ns()
    realtime = time.time_ns()
    raw = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
    after = time.monotonic_ns()
    return {'host_receipt_ns': realtime, 'host_monotonic_ns': (before+after)//2,
            'host_monotonic_raw_ns': raw, 'host_clock_sample_span_ns': after-before}


class TelemetryTee:
    """수신자 부재/포화/잘못된 JSON을 누락으로 집계하고 제어 루프로 즉시 복귀한다."""
    def __init__(self, path: Path | str) -> None:
        self.path = str(path)
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.dropped = 0
        self.last_emit_ns = 0

    def emit(self, payload: dict) -> bool:
        started = time.perf_counter_ns()
        try:
            data = json.dumps(payload, separators=(',', ':'), allow_nan=False).encode()
            self.socket.sendto(data, self.path)
            return True
        except (OSError, ValueError, TypeError):
            self.dropped += 1
            return False
        finally:
            self.last_emit_ns = time.perf_counter_ns() - started

    def close(self) -> None:
        self.socket.close()
