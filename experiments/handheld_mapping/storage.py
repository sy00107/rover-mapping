# ================================================================
# storage.py
# 설명: 전달받은 IR·depth 원본과 metadata를 새 노트북 폴더에만 저장한다.
# 사용법: from experiments.handheld_mapping.storage import RunWriter
# ================================================================
import hashlib
import json
from pathlib import Path
import re
import time
import zlib

import cv2
import numpy as np


def atomic_json(path: Path, data: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def decode_image(header: dict, payload: bytes) -> np.ndarray:
    width, height, step = (int(header[key]) for key in ("width", "height", "step"))
    encoding = header["encoding"]
    if encoding not in ("mono8", "16UC1"):
        raise ValueError("unsupported image encoding")
    pixel = 1 if encoding == "mono8" else 2
    if not (0 < width <= 4096 and 0 < height <= 2160 and width * pixel <= step <= width * pixel + 4096):
        raise ValueError("invalid image dimensions/stride")
    size = height * step
    if size > 32 * 1024 * 1024:
        raise ValueError("invalid image size/compression")
    if header.get('compression') == 'jpeg' and header.get('preview') is True and encoding == 'mono8':
        if len(payload) > size*4:
            raise ValueError('oversized preview JPEG')
        image = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_GRAYSCALE)
        if image is None or image.shape != (height, width):
            raise ValueError('invalid preview JPEG')
        return image
    if header.get('compression') != 'zlib':
        raise ValueError('invalid image size/compression')
    decoder = zlib.decompressobj()
    raw = decoder.decompress(payload, size + 1)
    if len(raw) != size or not decoder.eof or decoder.unused_data:
        raise ValueError("invalid image payload length")
    dtype = np.dtype(">u2" if header.get("is_bigendian") else "<u2") if pixel == 2 else np.dtype("u1")
    return np.ndarray((height, width), dtype=dtype, buffer=raw, strides=(step, pixel)).astype(
        np.uint16 if pixel == 2 else np.uint8, copy=True)


class RunWriter:
    """노트북 원본과 최종 ORB 산출물을 기록하고 경로·checksum을 검증한다."""

    def __init__(self, path: Path, resume: bool = False) -> None:
        self.path = Path(path)
        self.path.mkdir(parents=True, exist_ok=resume)
        (self.path / "orb").mkdir(exist_ok=resume)
        self.resuming = resume
        self.counts = {key: 0 for key in ("left", "right", "depth")}
        self.last_stamp: dict[str, int] = {}
        self.saved_stamps = {side: set() for side in self.counts}
        if resume and (self.path/'frames.jsonl').exists():
            with (self.path/'frames.jsonl').open('r+b') as stream:
                while True:
                    position = stream.tell()
                    line = stream.readline()
                    if not line:
                        break
                    try:
                        if not line.endswith(b'\n'):
                            raise ValueError('unfinished index row')
                        row = json.loads(line)
                    except (ValueError, UnicodeDecodeError):
                        if stream.read(1):
                            raise ValueError('corrupt interior frame index')
                        stream.truncate(position)
                        break
                    side, stamp = row['stream'], row['stamp_ns']
                    if (side not in self.counts or not isinstance(stamp, int)
                            or stamp <= self.last_stamp.get(side, -1)
                            or row['path'] != f'raw/{side}/{stamp}.png'):
                        raise ValueError('invalid saved frame index')
                    self.saved_stamps[side].add(stamp)
                    self.last_stamp[side] = stamp
                    self.counts[side] += 1
        self.index = (self.path / "frames.jsonl").open("a" if resume else "w", buffering=1)
        self.events = (self.path / "events.jsonl").open("a" if resume else "w", buffering=1)
        self.image_timings = {side: {"images": 0, "decode_ms_total": 0., "decode_ms_max": 0.,
                                    "png_write_ms_total": 0., "png_write_ms_max": 0.}
                              for side in self.counts}

    def accept(self, header: dict, payload: bytes) -> np.ndarray | None:
        kind = header["kind"]
        if kind == "image":
            side, stamp = header["stream"], header["stamp_ns"]
            duplicate = self.resuming and side in self.saved_stamps and stamp in self.saved_stamps[side]
            if (side not in self.counts or not isinstance(stamp, int)
                    or (stamp <= self.last_stamp.get(side, -1) and not duplicate)):
                raise ValueError("invalid/non-increasing image stream or stamp")
            decode_start = time.monotonic_ns()
            image = decode_image(header, payload)
            decode_ms = (time.monotonic_ns()-decode_start)/1e6
            if (side == "depth") != (image.dtype == np.uint16):
                raise ValueError("image encoding disagrees with stream")
            relative = f"raw/{side}/{stamp}.png"
            target = self.path / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            if duplicate:
                saved = cv2.imread(str(target), cv2.IMREAD_UNCHANGED) if target.exists() else None
                if saved is not None and not np.array_equal(saved, image):
                    raise ValueError('재개할 영상이 검증된 Pi 원본과 다릅니다')
                if saved is None and not cv2.imwrite(str(target), image, [cv2.IMWRITE_PNG_COMPRESSION, 1]):
                    raise OSError('중단된 원본 영상 복구 실패')
                return None
            png_start = time.monotonic_ns()
            if not cv2.imwrite(str(target), image, [cv2.IMWRITE_PNG_COMPRESSION, 1]):
                raise OSError("원본 영상 저장 실패")
            saved_ns = time.monotonic_ns()
            png_ms = (saved_ns-png_start)/1e6
            self.index.write(json.dumps({**header, "path": relative, "laptop_decode_ms": decode_ms,
                                        "laptop_png_write_ms": png_ms,
                                        "laptop_image_saved_monotonic_ns": saved_ns}) + "\n")
            stats = self.image_timings[side]
            stats["images"] += 1
            for key, value in (("decode_ms", decode_ms), ("png_write_ms", png_ms)):
                stats[key+"_total"] += value
                stats[key+"_max"] = max(stats[key+"_max"], value)
            self.counts[side] += 1
            self.last_stamp[side] = stamp
            return image
        if kind == "calibration":
            target = self.path / "capture_calibration.json"
            if target.exists():
                if self.resuming:
                    def authoritative(data):
                        return {k: v for k, v in data.items()
                                if not k.startswith('laptop_') and k != 'delivery_mode'}
                    if authoritative(json.loads(target.read_text())) != authoritative(header):
                        raise ValueError('재개할 카메라 보정값이 Pi 원본과 다릅니다')
                    return None
                raise ValueError("calibration changed during capture")
            atomic_json(target, header)
        elif kind == "artifact":
            name = header["name"]
            if not re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_.-]*\.(csv|json|yaml|osa|ply|tum|log)", name):
                raise ValueError("invalid artifact path")
            if hashlib.sha256(payload).hexdigest() != header.get("sha256"):
                raise ValueError("artifact checksum mismatch")
            target = self.path / "orb" / name
            if target.exists():
                if self.resuming and hashlib.sha256(target.read_bytes()).hexdigest() == header.get('sha256'):
                    return None
                raise ValueError("duplicate artifact")
            temporary = target.with_suffix(target.suffix + ".tmp")
            temporary.write_bytes(payload)
            temporary.replace(target)
        self.events.write(json.dumps(header, allow_nan=False) + "\n")
        return None

    def close(self) -> None:
        self.index.close()
        self.events.close()
