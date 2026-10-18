# ================================================================
# pi_worker.py
# 설명: Pi native Jazzy에서 D435 원본을 전송하며 선택적으로 기존 ORB를 실행한다.
# 사용법: /usr/bin/python3 -m experiments.handheld_mapping.pi_worker --run-dir /dev/shm/rover_mapping_ID/run
# ================================================================
import argparse
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import signal
import shutil
import subprocess
import sys
import threading
import time
import zlib
from typing import Callable

from experiments.handheld_mapping.protocol import write_packet
from experiments.handheld_mapping.reconstruct import pose_matrix
from experiments.handheld_mapping.transport import ByteQueue, FrameBundler
from experiments.handheld_mapping.capture_integrity import stale_streams


def recent_camera_rates(receipts: dict, window: int = 15) -> dict:
    """Use a recent full window so startup/reconnection delays can age out."""
    rates = {}
    for side, values in receipts.items():
        samples = list(values)[-window:]
        elapsed = samples[-1]-samples[0] if len(samples) >= window else 0.
        rates[side] = (len(samples)-1)/elapsed if elapsed > 0 else 0.
    return rates


class CaptureMetrics:
    """녹화 중 stream별 수집·큐 누락과 Pi monotonic 송신 시간을 고정 크기로 집계한다."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.streams = {side: {'received': 0, 'enqueued': 0, 'dropped': 0,
                              'first_stamp_ns': None, 'last_stamp_ns': None,
                              'last_receipt_monotonic_ns': None,
                              'max_source_gap_ms': 0., 'max_receipt_gap_ms': 0.}
                        for side in ('left', 'right', 'depth')}
        self.non_image_dropped = {'status': 0, 'pose': 0}
        self.sender = {'sent_packets': 0, 'last_compress_ms': 0., 'max_compress_ms': 0.,
                       'last_write_ms': 0., 'max_write_ms': 0., 'write_ms_total': 0.,
                       'compress_ms_total': 0., 'payload_bytes_sent': 0}
        self.send_start_ns: int | None = None
        self.bundle_provider = None

    def image_received(self, side: str, stamp_ns: int, receipt_ns: int) -> int:
        with self.lock:
            stream = self.streams[side]
            if stream['last_stamp_ns'] is not None:
                stream['max_source_gap_ms'] = max(stream['max_source_gap_ms'],
                                                 (stamp_ns-stream['last_stamp_ns'])/1e6)
                stream['max_receipt_gap_ms'] = max(stream['max_receipt_gap_ms'],
                                                  (receipt_ns-stream['last_receipt_monotonic_ns'])/1e6)
            else:
                stream['first_stamp_ns'] = stamp_ns
            stream['received'] += 1
            stream['last_stamp_ns'] = stamp_ns
            stream['last_receipt_monotonic_ns'] = receipt_ns
            return stream['received']

    def enqueue_result(self, header: dict, enqueued: bool) -> None:
        with self.lock:
            if header['kind'] == 'image':
                self.streams[header['stream']]['enqueued' if enqueued else 'dropped'] += 1
            elif not enqueued and header['kind'] in self.non_image_dropped:
                self.non_image_dropped[header['kind']] += 1

    def send_started(self, start_ns: int, compress_ms: float | None = None) -> None:
        with self.lock:
            self.send_start_ns = start_ns
            if compress_ms is not None:
                self.sender['last_compress_ms'] = compress_ms
                self.sender['max_compress_ms'] = max(self.sender['max_compress_ms'], compress_ms)
                self.sender['compress_ms_total'] += compress_ms

    def send_finished(self, write_ms: float, payload_bytes: int = 0) -> None:
        with self.lock:
            self.sender['sent_packets'] += 1
            self.sender['last_write_ms'] = write_ms
            self.sender['max_write_ms'] = max(self.sender['max_write_ms'], write_ms)
            self.sender['write_ms_total'] += write_ms
            self.sender['payload_bytes_sent'] += payload_bytes
            self.send_start_ns = None

    def snapshot(self, now_ns: int | None = None) -> dict:
        with self.lock:
            now = time.monotonic_ns() if now_ns is None else now_ns
            age = max(0, now-self.send_start_ns)/1e6 if self.send_start_ns is not None else 0.
            return {'streams': {side: dict(stream) for side, stream in self.streams.items()},
                    'non_image_dropped': dict(self.non_image_dropped),
                    'sender': {**self.sender, 'send_in_progress_age_ms': age},
                    'bundles': self.bundle_provider() if self.bundle_provider is not None else None,
                    'scope': 'recording-only stream callbacks and queued sender packets; '
                             'source gaps use camera stamps; other timings use Pi-local monotonic clock, '
                             'not end-to-end latency; '
                             'legacy source_frames includes startup left callbacks'}


def receipt_clock() -> dict:
    """Pi의 realtime·RAW를 monotonic 두 조회로 감싸 원본 header에 보존한다."""
    before = time.monotonic_ns()
    realtime = time.time_ns()
    raw = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
    after = time.monotonic_ns()
    return {'pi_receipt_monotonic_ns': (before+after)//2,
            'pi_receipt_realtime_ns': realtime, 'pi_clock_sample_span_ns': after-before,
            'pi_receipt_monotonic_raw_ns': raw}


def depth_parameters(fetch: Callable[[list[str]], list]) -> dict:
    """필수 projector 상태와 선택 sensor 단위를 별도 조회해 검증한다."""
    emitter = fetch(['depth_module.emitter_enabled'])
    if len(emitter) != 1 or emitter[0].type != 2 or emitter[0].integer_value != 0:
        raise RuntimeError('projector OFF 검증 실패')
    units = fetch(['depth_module.depth_units'])
    scale = None
    if len(units) == 1 and units[0].type == 3:
        value = float(units[0].double_value)
        if math.isfinite(value) and 0 < value < .1:
            scale = value
    return {'emitter_enabled': 0, 'sensor_depth_units_m': scale}


def field_parameter_snapshot(fetch):
    """Record optional camera settings without inventing defaults for unsupported parameters."""
    names=['depth_module.enable_auto_exposure','depth_module.exposure','depth_module.gain',
           'depth_module.emitter_enabled','depth_module.global_time_enabled']
    result={}
    for name in names:
        try:
            values=fetch([name]);value=values[0] if values else None
            result[name]=(getattr(value,{1:'bool_value',2:'integer_value',3:'double_value',4:'string_value'}.get(value.type,''),None)
                          if value is not None else None)
        except Exception:
            result[name]=None
    return result


def camera_metadata_record(side,msg):
    data=msg.json_data
    if len(data)>32768:raise ValueError('Camera metadata exceeds bounded record size')
    return dict(kind='camera_metadata',stream=side,
                stamp_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec,
                frame_id=msg.header.frame_id,json_data=data,**receipt_clock())


def stop_child(process: subprocess.Popen | None, timeout: float = 35.) -> None:
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        raise RuntimeError("정상 종료 시간 초과 — 결과를 저장 완료로 처리하지 않습니다")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--sensors-only", action="store_true", help="Pi에서는 ORB/Atlas 없이 원본만 전송")
    parser.add_argument("--queue-mib", type=float, default=4.)
    parser.add_argument("--field-metadata",action="store_true",help="수집용 카메라 원시 metadata와 설정 기록")
    parser.add_argument('--independent-streams', action='store_true',
                        help='Pi 로컬 기록: depth 누락과 무관하게 각 원본을 보존')
    args = parser.parse_args()
    if not .25 <= args.queue_mib <= 64:
        raise ValueError("invalid queue MiB limit")
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from rcl_interfaces.srv import GetParameters
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import Int32
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import PoseStamped
    import tf2_ros

    run = args.run_dir.resolve()
    if not run.is_relative_to(Path("/dev/shm")):
        raise ValueError("Pi run must reside in /dev/shm")
    run.mkdir(parents=True, exist_ok=False)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGHUP, lambda *_: stop.set())
    threading.Thread(target=lambda: (sys.stdin.readline(), stop.set()), daemon=True).start()
    output_lock = threading.Lock()

    def send(header: dict, payload: bytes = b"") -> None:
        with output_lock:
            write_packet(sys.stdout.buffer, header, payload)

    rclpy.init()

    class CaptureNode(Node):
        """CameraInfo·TF를 검증하고 bounded queue로 원본과 상태를 전달한다."""

        def __init__(self) -> None:
            super().__init__("handheld_capture")
            self.infos = {}
            self.stamps = {side: deque(maxlen=60) for side in ("left", "right", "depth")}
            self.receipt = {side: deque(maxlen=60) for side in self.stamps}
            self.pending = ByteQueue(max_items=192 if args.independent_streams else 24,
                                     max_bytes=int(args.queue_mib*1024*1024))
            self.recording = False
            self.dropped = 0
            self.state = -1
            self.error = None
            self.sent = 0
            self.capture_metrics = CaptureMetrics()
            self.bundler = FrameBundler(self.enqueue_bundle, self.discard_image)
            self.capture_metrics.bundle_provider = self.bundler.snapshot
            # 기존 source_frames는 준비 중 callback도 포함하며 capture_metrics는 녹화 중만 센다.
            self.source_frames = 0
            self.buffer = tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener(self.buffer, self)
            self.subscriptions_owned = []
            self.metadata_available=False
            if getattr(args,'field_metadata',False):
                try:
                    from realsense2_camera_msgs.msg import Metadata
                    self.metadata_available=True
                    self.capture_metrics.non_image_dropped['camera_metadata']=0
                except ImportError:
                    pass
            names = {"left": "infra1", "right": "infra2", "depth": "depth"}
            for side, name in names.items():
                image_topic = f"/camera/camera/{name}/" + ("image_rect_raw" if side != "depth" else "image_rect_raw")
                self.subscriptions_owned.append(self.create_subscription(
                    Image, image_topic, lambda msg, side=side: self.image(side, msg), qos_profile_sensor_data))
                self.subscriptions_owned.append(self.create_subscription(
                    CameraInfo, f"/camera/camera/{name}/camera_info",
                    lambda msg, side=side: self.infos.__setitem__(side, msg), qos_profile_sensor_data))
                if self.metadata_available:
                    self.subscriptions_owned.append(self.create_subscription(
                        Metadata,f"/camera/camera/{name}/metadata",
                        lambda msg,side=side:self.metadata(side,msg),qos_profile_sensor_data))
            self.subscriptions_owned.append(self.create_subscription(Int32, "/orbslam3/tracking_state", self.tracking, 10))
            self.subscriptions_owned.append(self.create_subscription(PoseStamped, "/orbslam3/camera_pose", self.pose, 10))
            self.subscriptions_owned.append(self.create_subscription(DiagnosticArray, "/orbslam3/diagnostics", self.diagnostics, 10))

        def enqueue(self, header: dict, payload: bytes = b"") -> None:
            if not self.recording:
                return
            if header['kind'] == 'image' and not args.independent_streams:
                self.bundler.push((header, payload))
                return
            if header['kind'] == 'image':
                header['pi_enqueue_monotonic_ns'] = time.monotonic_ns()
            try:
                self.pending.put_nowait((header, payload))
            except queue.Full:
                self.dropped += 1
                self.capture_metrics.enqueue_result(header, enqueued=False)
            else:
                self.capture_metrics.enqueue_result(header, enqueued=True)

        def discard_image(self, item) -> None:
            self.dropped += 1
            self.capture_metrics.enqueue_result(item[0], enqueued=False)

        def enqueue_bundle(self, items) -> None:
            now = time.monotonic_ns()
            for header, _ in items:
                header.update(pi_enqueue_monotonic_ns=now,
                              capture_group_stamp_ns=items[0][0]['stamp_ns'], capture_group_size=3,
                              pi_bundle_wait_ms=(now-header['pi_receipt_monotonic_ns'])/1e6)
            try:
                # Persist depth before offering the stereo pair to live ORB on the laptop.
                self.pending.put_many_nowait([items[2], items[0], items[1]])
            except queue.Full:
                for item in items:
                    self.discard_image(item)
            else:
                for header, _ in items:
                    self.capture_metrics.enqueue_result(header, enqueued=True)

        def image(self, side: str, msg: Image) -> None:
            clock = receipt_clock()
            receipt_ns = clock['pi_receipt_monotonic_ns']
            stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            if self.stamps[side] and stamp <= self.stamps[side][-1]:
                self.error = "카메라 시각이 역행하거나 중복됐습니다"
                stop.set()
                return
            self.stamps[side].append(stamp)
            if side == "left":
                self.source_frames += 1
            self.receipt[side].append(receipt_ns/1e9)
            if (msg.width, msg.height) != (640, 480) or msg.encoding != ("16UC1" if side == "depth" else "mono8"):
                self.error = "실제 카메라 profile이 요청한 640x480 mono8/Z16과 다릅니다"
                stop.set()
                return
            header = {"kind": "image", "stream": side, "stamp_ns": stamp, "width": msg.width,
                      "height": msg.height, "step": msg.step, "encoding": msg.encoding,
                      "frame_id": msg.header.frame_id, "is_bigendian": bool(msg.is_bigendian),
                      "compression": "zlib"}
            if self.recording:
                header.update(**clock, stream_seq=self.capture_metrics.image_received(side, stamp, receipt_ns))
            self.enqueue(header, bytes(msg.data))

        def metadata(self,side,msg):
            if not self.recording:return
            try:self.enqueue(camera_metadata_record(side,msg))
            except (ValueError,TypeError):
                self.capture_metrics.non_image_dropped['camera_metadata']+=1

        def tracking(self, msg: Int32) -> None:
            self.state = msg.data

        def pose(self, msg: PoseStamped) -> None:
            p, q = msg.pose.position, msg.pose.orientation
            self.enqueue({"kind": "pose", "stamp_ns": msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec,
                          "frame_id": msg.header.frame_id, "position": [p.x, p.y, p.z],
                          "quaternion_xyzw": [q.x, q.y, q.z, q.w]})

        def diagnostics(self, msg: DiagnosticArray) -> None:
            values = {value.key: value.value for status in msg.status for value in status.values}
            self.enqueue({"kind": "status", "tracking_state": self.state, "dropped": self.dropped,
                          "source_frames": self.source_frames, "diagnostics": values,
                          "queue": self.pending.snapshot(), "capture_metrics": self.capture_metrics.snapshot()})

    node = CaptureNode()
    camera = orb = None
    handles = []
    sender = None

    def wait_until(predicate, seconds: float) -> None:
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline and not predicate():
            if stop.is_set():
                raise RuntimeError(node.error or "시작 중 종료 요청")
            if camera is not None and camera.poll() is not None:
                raise RuntimeError("D435 실행 실패 — camera.log 확인")
            if orb is not None and orb.poll() is not None:
                raise RuntimeError("ORB 실행 실패 — orb.log 확인")
            rclpy.spin_once(node, timeout_sec=.05)
        if not predicate():
            raise RuntimeError("카메라/TF/ORB 준비 시간 초과")

    def spawn(command: list[str], name: str) -> subprocess.Popen:
        log = (run / name).open("wb")
        handles.append(log)
        return subprocess.Popen(command, cwd=run, stdout=log, stderr=log, start_new_session=True)

    try:
        end = time.monotonic()+1.5
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        if node.count_publishers("/camera/camera/infra1/image_rect_raw") or node.count_publishers("/orbslam3/camera_pose"):
            raise RuntimeError("기존 카메라/ORB가 실행 중입니다. 기존 실행을 종료한 뒤 시작하세요")
        camera = spawn(["ros2", "run", "realsense2_camera", "realsense2_camera_node", "--ros-args",
                        "-r", "__ns:=/camera", "-r", "__node:=camera",
                        "-p", "enable_color:=false", "-p", "enable_depth:=true",
                        "-p", "enable_infra1:=true", "-p", "enable_infra2:=true",
                        "-p", "depth_module.infra_profile:=640,480,15", "-p", "depth_module.depth_profile:=640,480,15",
                        "-p", "depth_module.infra1_format:=Y8", "-p", "depth_module.infra2_format:=Y8",
                        "-p", "depth_module.emitter_enabled:=0", "-p", "enable_sync:=true"], "camera.log")
        def camera_ready():
            return (len(node.infos) == 3 and min(len(values) for values in node.stamps.values()) >= 20
                    and all(12. <= rate <= 18. for rate in recent_camera_rates(node.receipt).values()))
        try:
            wait_until(camera_ready, 30)
        except RuntimeError as error:
            if str(error) != '카메라/TF/ORB 준비 시간 초과':
                raise
            rates = ', '.join(f'{side} {rate:.1f} FPS' for side, rate in recent_camera_rates(node.receipt).items())
            raise RuntimeError('카메라 시작 검사 실패 · 안정적인 15 FPS 영상이 오지 않습니다 ('+
                               rates+'). USB 연결과 camera.log를 확인하세요.') from error
        def info_dict(msg) -> dict:
            return {"width": msg.width, "height": msg.height, "k": list(msg.k), "p": list(msg.p),
                    "r": list(msg.r), "d": list(msg.d), "distortion_model": msg.distortion_model,
                    "binning_x": msg.binning_x, "binning_y": msg.binning_y,
                    "roi": {key: getattr(msg.roi, key) for key in ("x_offset", "y_offset", "width", "height")}}
        # helpers는 배포된 code/helpers 아래에 있으며 run의 sibling이다.
        sys.path.insert(0, str(run.parent / "code/helpers"))
        from prepare_mapping_run import calibration_from_probe, prepare_run, sha256
        stats = {}
        rates = recent_camera_rates(node.receipt)
        for side in ("left", "right"):
            stats[side] = {"encoding": "mono8", "width": 640, "height": 480,
                           "receipt_rate_hz": rates[side]}
        probe = {"camera_info": {side: info_dict(node.infos[side]) for side in ("left", "right")},
                 "stats": stats, "exact_pair_candidates": len(set(node.stamps["left"]) & set(node.stamps["right"]))}
        calibration = calibration_from_probe(probe, None, "640x480x15:Y8", True)
        vocabulary = Path("/home/pi/handheld_mapping/runtime/ORBvoc.txt")
        if not args.sensors_only:
            calibration["vocabulary_sha256"] = sha256(vocabulary)
        left_frame, depth_frame = node.infos["left"].header.frame_id, node.infos["depth"].header.frame_id
        wait_until(lambda: node.buffer.can_transform(left_frame, depth_frame, Time()), 10)
        tf = node.buffer.lookup_transform(left_frame, depth_frame, Time()).transform
        left_depth = pose_matrix({"x": tf.translation.x, "y": tf.translation.y, "z": tf.translation.z,
                                  "qx": tf.rotation.x, "qy": tf.rotation.y, "qz": tf.rotation.z, "qw": tf.rotation.w})
        client = node.create_client(GetParameters, "/camera/camera/get_parameters")
        wait_until(lambda: client.service_is_ready(), 5)
        def fetch_parameters(names: list[str]) -> list:
            future = client.call_async(GetParameters.Request(names=names))
            wait_until(future.done, 5)
            return future.result().values
        parameters = depth_parameters(fetch_parameters)
        field_settings=field_parameter_snapshot(fetch_parameters) if args.field_metadata else None
        depth = {**info_dict(node.infos["depth"]), "depth_scale_m": .001,
                 "depth_scale_source": "RealSense ROS fix_depth_scale: Z16 millimetres",
                 "sensor_depth_units_m": parameters['sensor_depth_units_m'], "T_left_depth": left_depth.tolist(),
                 "depth_frame": depth_frame, "left_frame": left_frame, "transform_source": "live_tf2"}
        if any(abs(value) > 1e-8 for value in depth["d"]):
            raise RuntimeError("TSDF에는 rectified depth 보정값이 필요합니다")
        # prepare_run은 새 폴더를 요구하므로 별도 RAM subdirectory에 ORB 산출물을 둔다.
        prepared = run / "orb_run"
        if not args.sensors_only:
            prepare_run(prepared, calibration, None)
        send({"kind": "calibration", "source": "D435_Pi", "depth": depth, "stereo": calibration,
              "emitter_enabled": 0, "sensor_profile": "640x480x15",
              "capture_mode": "sensors_only" if args.sensors_only else "pi_orb",
              "field_camera_settings":field_settings,"camera_metadata_subscription":node.metadata_available})
        def transmit() -> None:
            try:
                while True:
                    item = node.pending.get()
                    if item is None:
                        return
                    header, payload = item
                    dequeued_ns = time.monotonic_ns()
                    compress_ms = None
                    if header["kind"] == "image":
                        payload = zlib.compress(payload, 1)
                        compress_ms = (time.monotonic_ns()-dequeued_ns)/1e6
                    send_start_ns = time.monotonic_ns()
                    if header['kind'] == 'image':
                        header.update(pi_queue_wait_ms=(dequeued_ns-header['pi_enqueue_monotonic_ns'])/1e6,
                                      pi_compress_ms=compress_ms, pi_send_start_monotonic_ns=send_start_ns)
                    node.capture_metrics.send_started(send_start_ns, compress_ms)
                    send(header, payload)
                    node.capture_metrics.send_finished((time.monotonic_ns()-send_start_ns)/1e6, len(payload))
                    node.sent += 1
            except Exception as error:
                node.error = str(error)
                stop.set()
        sender = threading.Thread(target=transmit, daemon=True)
        sender.start()
        node.recording = True
        binary = "/home/pi/handheld_mapping/ros2_ws/install/orbslam3_ros2/lib/orbslam3_ros2/orbslam3_stereo_inertial"
        command = [binary, "--ros-args", "-p", "sensor_mode:=stereo", "-p", f"vocabulary_path:={vocabulary}",
                   "-p", f"settings_path:={prepared/'orb.yaml'}", "-p", f"trajectory_dir:={prepared}",
                   "-p", "require_prepared_run:=true", "-p", "use_sim_time:=false",
                   "-p", "left_topic:=/camera/camera/infra1/image_rect_raw",
                   "-p", "right_topic:=/camera/camera/infra2/image_rect_raw"]
        if not args.sensors_only:
            log = (run / "orb.log").open("wb")
            handles.append(log)
            orb = subprocess.Popen(command, cwd=prepared, stdout=log, stderr=log, start_new_session=True)
            wait_until(lambda: node.state >= 0, 45)
        send({"kind": "ready", "message": "D435 센서 전송 · 종료 후 노트북 ORB" if args.sensors_only else "D435 원본 저장·ORB 촬영 중"})
        last_status = time.monotonic()
        while not stop.is_set():
            rclpy.spin_once(node, timeout_sec=.05)
            stale = stale_streams(node.receipt, time.monotonic())
            if camera.poll() is not None or (orb is not None and orb.poll() is not None):
                raise RuntimeError("카메라/ORB 종료")
            if 'left' in stale or 'right' in stale:
                raise RuntimeError('좌우 카메라 영상 수신 중단: '+', '.join(stale))
            if time.monotonic()-last_status >= 1.:
                free = shutil.disk_usage('/dev/shm').free
                if free < 32*1024*1024:
                    raise RuntimeError("Pi RAM disk 여유 부족 — 현재 촬영을 종료합니다")
                node.enqueue({"kind": "status", "tracking_state": node.state, "dropped": node.dropped,
                              "stream_warning": '깊이 영상 수신 중단 · 좌우 원본은 계속 기록합니다' if 'depth' in stale else None,
                              "source_frames": node.source_frames, "queue": node.pending.snapshot(),
                              "capture_metrics": node.capture_metrics.snapshot(), "ram_disk_free_bytes": free})
                last_status = time.monotonic()
        node.recording = False
        node.bundler.finish()
        stop_child(orb)
        stop_child(camera, 10)
        node.pending.finish(timeout=10)
        sender.join(timeout=15)
        if sender.is_alive() or node.error:
            raise RuntimeError(node.error or "원본 전송 마감 시간 초과")
        tracking = ({"status": "saved" if node.source_frames else "incomplete"} if args.sensors_only
                    else json.loads((prepared / "tracking.json").read_text()))
        for path in sorted(prepared.iterdir()) if not args.sensors_only else []:
            if path.is_file() and path.suffix in (".csv", ".json", ".yaml", ".osa", ".ply", ".tum", ".log"):
                payload = path.read_bytes()
                send({"kind": "artifact", "name": path.name, "sha256": hashlib.sha256(payload).hexdigest()}, payload)
        for name in (("camera.log",) if args.sensors_only else ("camera.log", "orb.log")):
            payload = (run / name).read_bytes()
            send({"kind": "artifact", "name": name, "sha256": hashlib.sha256(payload).hexdigest()}, payload)
        send({"kind": "finished", "status": tracking["status"], "dropped": node.dropped, "source": "D435_Pi",
              "source_frames": node.source_frames,
              "capture_mode": "sensors_only" if args.sensors_only else "pi_orb", "queue": node.pending.snapshot(),
              "capture_metrics": node.capture_metrics.snapshot()})
    except Exception as error:
        node.recording = False
        try:
            send({"kind": "error", "message": str(error)})
        except (BrokenPipeError, OSError):
            pass
        raise
    finally:
        node.recording = False
        for process in (orb, camera):
            try:
                stop_child(process, 10)
            except Exception as error:
                print(str(error), file=sys.stderr)
        for handle in handles:
            handle.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
