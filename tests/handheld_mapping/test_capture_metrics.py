# ================================================================
# test_capture_metrics.py
# 설명: ROS 없이 Pi 수집·누락·송신 계측과 monotonic 시각의 범위를 검증한다.
# 사용법: PYTHONNOUSERSITE=1 .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_capture_metrics.py -q
# ================================================================
import ast
from collections import deque
from pathlib import Path
import queue
import threading
from types import SimpleNamespace
import zlib

import pytest

from experiments.handheld_mapping import pi_worker
from experiments.handheld_mapping.transport import ByteQueue, FrameBundler


class FakeNode:
    """ROS runtime 없이 CaptureNode 초기화를 수용한다."""

    def __init__(self, name: str) -> None:
        self.name = name

    def create_subscription(self, *args: object) -> None:
        return None


def capture_node(independent_streams=False) -> object:
    assert hasattr(pi_worker, 'CaptureMetrics'), 'recording-only CaptureMetrics is missing'
    source = Path(pi_worker.__file__).read_text()
    definition = next(node for node in ast.walk(ast.parse(source))
                      if isinstance(node, ast.ClassDef) and node.name == 'CaptureNode')
    namespace = {'Node': FakeNode, 'deque': deque, 'ByteQueue': ByteQueue, 'FrameBundler': FrameBundler,
                 'CaptureMetrics': pi_worker.CaptureMetrics, 'queue': queue,
                 'receipt_clock': pi_worker.receipt_clock,
                 'time': pi_worker.time, 'args': SimpleNamespace(queue_mib=4., independent_streams=independent_streams),
                 'stop': threading.Event(), 'qos_profile_sensor_data': object(),
                 'tf2_ros': SimpleNamespace(Buffer=lambda: None, TransformListener=lambda *args: None),
                 **{name: SimpleNamespace for name in
                    ('Image', 'CameraInfo', 'Int32', 'PoseStamped', 'DiagnosticArray')}}
    exec(compile(ast.Module(body=[definition], type_ignores=[]), 'pi_worker_capture', 'exec'), namespace)
    return namespace['CaptureNode']()


def image_message(stamp_ns: int, side='left') -> SimpleNamespace:
    seconds, nanoseconds = divmod(stamp_ns, 1_000_000_000)
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=seconds, nanosec=nanoseconds),
                                                frame_id='camera_left_optical_frame'),
                           width=640, height=480, step=1280 if side == 'depth' else 640,
                           encoding='16UC1' if side == 'depth' else 'mono8',
                           is_bigendian=False, data=b'\0'*(640*480*(2 if side == 'depth' else 1)))


def test_capture_excludes_startup_but_legacy_source_frames_includes_it(monkeypatch: pytest.MonkeyPatch) -> None:
    node = capture_node()
    now = [1_000_000_000]
    monkeypatch.setattr(pi_worker.time, 'monotonic_ns', lambda: now[0])
    node.image('left', image_message(10_000_000_000))
    node.recording = True
    for receipt, stamp in ((2_000_000_000, 10_100_000_000), (2_500_000_000, 10_300_000_000)):
        now[0] = receipt
        for side in ('left', 'right', 'depth'):
            node.image(side, image_message(stamp, side))
    snapshot = node.capture_metrics.snapshot(now_ns=3_000_000_000)
    left = snapshot['streams']['left']
    assert node.source_frames == 3
    assert (left['received'], left['enqueued'], left['dropped']) == (2, 2, 0)
    assert (left['first_stamp_ns'], left['last_stamp_ns']) == (10_100_000_000, 10_300_000_000)
    assert left['max_source_gap_ms'] == pytest.approx(200.)
    assert left['max_receipt_gap_ms'] == pytest.approx(500.)
    headers = [node.pending.get()[0] for _ in range(6)]
    assert [h['stream'] for h in headers] == ['depth', 'left', 'right']*2
    headers = [header for header in headers if header['stream'] == 'left']
    assert [header['stream_seq'] for header in headers] == [1, 2]
    assert headers[0]['pi_receipt_monotonic_ns'] == 2_000_000_000
    assert headers[0]['stamp_ns'] == 10_100_000_000
    assert isinstance(headers[0]['pi_receipt_realtime_ns'], int)
    assert isinstance(headers[0]['pi_receipt_monotonic_raw_ns'], int)
    assert headers[0]['pi_clock_sample_span_ns'] == 0
    assert headers[0]['pi_enqueue_monotonic_ns'] == 2_000_000_000
    assert snapshot['streams']['right']['received'] == 2


def test_camera_clock_pair_preserves_domains_and_records_sampling_uncertainty(monkeypatch):
    ticks = iter((1000, 1200))
    monkeypatch.setattr(pi_worker.time, 'monotonic_ns', lambda: next(ticks))
    monkeypatch.setattr(pi_worker.time, 'time_ns', lambda: 200000)
    def raw(clock_id):
        assert clock_id == pi_worker.time.CLOCK_MONOTONIC_RAW
        return 7000
    monkeypatch.setattr(pi_worker.time, 'clock_gettime_ns', raw)
    assert pi_worker.receipt_clock() == {'pi_receipt_monotonic_ns': 1100,
        'pi_receipt_realtime_ns': 200000, 'pi_receipt_monotonic_raw_ns': 7000,
        'pi_clock_sample_span_ns': 200}


def test_local_capture_enqueues_stereo_without_depth_and_keeps_unpaired_raw():
    node = capture_node(independent_streams=True)
    node.recording = True
    for side in ('left', 'right'):
        node.image(side, image_message(1, side))
    node.image('left', image_message(2))
    node.pending.finish()
    headers = [node.pending.get()[0] for _ in range(3)]
    assert [(h['stream'], h['stamp_ns']) for h in headers] == [('left',1),('right',1),('left',2)]
    assert node.pending.get() is None
    assert all('pi_enqueue_monotonic_ns' in h for h in headers)
    assert node.capture_metrics.snapshot()['streams']['left']['enqueued'] == 2


def test_image_drops_are_separate_from_pose_and_status_drops() -> None:
    node = capture_node()
    node.recording = True
    node.pending = ByteQueue(max_bytes=1)
    node.image('left', image_message(1))
    node.bundler.finish()
    node.enqueue({'kind': 'status'})
    node.enqueue({'kind': 'pose'})
    snapshot = node.capture_metrics.snapshot()
    assert node.dropped == 3
    assert snapshot['streams']['left']['received'] == 1
    assert snapshot['streams']['left']['enqueued'] == 0
    assert snapshot['streams']['left']['dropped'] == 1
    assert snapshot['non_image_dropped'] == {'status': 1, 'pose': 1}


def test_sender_reports_in_progress_and_completed_durations() -> None:
    assert hasattr(pi_worker, 'CaptureMetrics'), 'sender CaptureMetrics is missing'
    metrics = pi_worker.CaptureMetrics()
    metrics.send_started(1_000_000_000, compress_ms=3.)
    busy = metrics.snapshot(now_ns=1_250_000_000)['sender']
    assert busy['sent_packets'] == 0
    assert busy['send_in_progress_age_ms'] == pytest.approx(250.)
    metrics.send_finished(write_ms=300.)
    metrics.send_started(2_000_000_000, compress_ms=2.)
    metrics.send_finished(write_ms=10.)
    done = metrics.snapshot(now_ns=3_000_000_000)['sender']
    assert done['sent_packets'] == 2
    assert done['last_compress_ms'] == pytest.approx(2.)
    assert done['max_compress_ms'] == pytest.approx(3.)
    assert done['last_write_ms'] == pytest.approx(10.)
    assert done['max_write_ms'] == pytest.approx(300.)
    assert done['send_in_progress_age_ms'] == 0.


def test_metric_snapshots_are_detached_and_memory_is_bounded() -> None:
    assert hasattr(pi_worker, 'CaptureMetrics'), 'bounded CaptureMetrics is missing'
    metrics = pi_worker.CaptureMetrics()
    for sequence in range(1000):
        metrics.image_received('left', sequence, sequence)
        metrics.enqueue_result({'kind': 'image', 'stream': 'left'}, enqueued=sequence % 2 == 0)
    snapshot = metrics.snapshot()
    assert len(snapshot['streams']) == 3
    assert snapshot['streams']['left']['received'] == 1000
    assert snapshot['streams']['left']['enqueued'] == 500
    assert snapshot['streams']['left']['dropped'] == 500
    snapshot['streams']['left']['received'] = -1
    snapshot['sender']['sent_packets'] = -1
    snapshot['non_image_dropped']['pose'] = -1
    fresh = metrics.snapshot()
    assert fresh['streams']['left']['received'] == 1000
    assert fresh['sender']['sent_packets'] == 0
    assert fresh['non_image_dropped']['pose'] == 0


def test_transmit_attaches_queue_and_compression_times_before_write(monkeypatch: pytest.MonkeyPatch) -> None:
    assert hasattr(pi_worker, 'CaptureMetrics'), 'sender CaptureMetrics is missing'
    definition = next(node for node in ast.walk(ast.parse(Path(pi_worker.__file__).read_text()))
                      if isinstance(node, ast.FunctionDef) and node.name == 'transmit')
    pending = ByteQueue()
    pending.put_nowait(({'kind': 'image', 'stream': 'left', 'pi_enqueue_monotonic_ns': 1_000_000_000}, b'hello'))
    pending.finish()
    node = SimpleNamespace(pending=pending, capture_metrics=pi_worker.CaptureMetrics(), error=None, sent=0)
    packets = []
    namespace = {'node': node, 'time': pi_worker.time, 'zlib': zlib,
                 'send': lambda header, payload: packets.append((dict(header), payload)),
                 'stop': threading.Event()}
    ticks = iter((2_000_000_000, 2_003_000_000, 2_004_000_000, 2_014_000_000))
    monkeypatch.setattr(pi_worker.time, 'monotonic_ns', lambda: next(ticks))
    exec(compile(ast.Module(body=[definition], type_ignores=[]), 'pi_worker_transmit', 'exec'), namespace)
    namespace['transmit']()
    assert node.error is None and node.sent == 1
    header, payload = packets[0]
    assert zlib.decompress(payload) == b'hello'
    assert header['pi_queue_wait_ms'] == pytest.approx(1000.)
    assert header['pi_compress_ms'] == pytest.approx(3.)
    assert header['pi_send_start_monotonic_ns'] == 2_004_000_000
    sender = node.capture_metrics.snapshot(now_ns=3_000_000_000)['sender']
    assert sender['sent_packets'] == 1
    assert sender['last_write_ms'] == pytest.approx(10.)


def test_finished_packet_contains_final_source_frame_counter() -> None:
    tree = ast.parse(Path(pi_worker.__file__).read_text())
    statement = next(node for node in ast.walk(tree)
                     if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call)
                     and isinstance(node.value.func, ast.Name) and node.value.func.id == 'send'
                     and isinstance(node.value.args[0], ast.Dict)
                     and any(isinstance(value, ast.Constant) and value.value == 'finished'
                             for value in node.value.args[0].values))
    packets = []
    namespace = {'tracking': {'status': 'saved'}, 'args': SimpleNamespace(sensors_only=True),
                 'node': SimpleNamespace(source_frames=7, dropped=0, pending=ByteQueue(),
                                         capture_metrics=pi_worker.CaptureMetrics()),
                 'send': packets.append}
    exec(compile(ast.Module(body=[statement], type_ignores=[]), 'pi_worker_finished', 'exec'), namespace)
    assert packets[0].get('source_frames') == 7
    assert packets[0]['capture_metrics']['streams']['left']['received'] == 0
    assert packets[0]['queue']['queued_bytes'] == 0
