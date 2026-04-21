#!/usr/bin/env python3
"""
LibreYOLO Inference Performance Benchmark
Fully configurable via benchmark_config.yaml

Supported model families:
  - YOLOX           (LibreYOLOXn / t / s / m / l / x  —  .pt)
  - YOLOv9          (LibreYOLO9t / s / m / c           —  .pt)
  - RF-DETR det.    (LibreRFDETRn / s / m / l          —  .pth)
  - RF-DETR seg.    (LibreRFDETRn/s/m/l-seg            —  .pth)
    → https://huggingface.co/LibreYOLO/LibreRFDETRn-seg

Supported backends (auto-detected from file extension / path):
  - PyTorch    (.pt / .pth)            — default
  - TensorRT   (.engine)               — pip install libreyolo[tensorrt]
  - ONNX       (.onnx)                 — pip install libreyolo[onnx]
  - OpenVINO   (directory with .xml)   — pip install libreyolo[openvino]

Input sources (controlled via benchmark_config.yaml):
  - fake    — loop tools.jpeg as a synthetic video stream
  - image   — single image file (repeated num_frames times for benchmarking)
  - video   — video file
  - webcam  — USB / built-in camera by device index

pip install (core):
  pip install libreyolo opencv-python psutil numpy torch torchvision pyyaml

pip install (optional backends):
  pip install libreyolo[rfdetr]      # RF-DETR detection + segmentation
  pip install libreyolo[tensorrt]    # TensorRT .engine inference
  pip install libreyolo[onnx]        # ONNX Runtime inference
  pip install libreyolo[openvino]    # OpenVINO inference
"""

import sys
import os
import time
import cv2
import numpy as np
import psutil
import traceback
import argparse
from pathlib import Path
from datetime import datetime
from collections import deque
from typing import Optional, Generator, Tuple

# ── YAML ──────────────────────────────────────────────────────────
try:
    import yaml
except ImportError:
    print("ERROR: pyyaml not installed.  pip install pyyaml")
    sys.exit(1)

# ── PyTorch ───────────────────────────────────────────────────────
try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False

# ── LibreYOLO ─────────────────────────────────────────────────────
try:
    from libreyolo import LibreYOLO
    LIBREYOLO_AVAILABLE = True
except ImportError:
    print("ERROR: libreyolo not installed.  pip install libreyolo")
    sys.exit(1)

# ── Optional backends / extras ────────────────────────────────────
RFDETR_AVAILABLE = False
# True when rfdetr ≥ 1.4.x (new class-based API: RFDETRNano, RFDETRSegMedium, …)
# False when only the old rfdetr.main.Model API is present (≤ 1.3.x, incompatible with
# libreyolo's internal adapter but usable via NativeRFDETRWrapper below)
RFDETR_NEW_API = False
TENSORRT_AVAILABLE = False
ONNX_AVAILABLE = False
OPENVINO_AVAILABLE = False

try:
    import rfdetr          # noqa: F401
    RFDETR_AVAILABLE = True
    # Check which API generation is present
    try:
        from rfdetr import RFDETRNano  # noqa: F401  — exists only in ≥ 1.4.x
        RFDETR_NEW_API = True
    except ImportError:
        pass
except ImportError:
    pass

try:
    import tensorrt        # noqa: F401
    TENSORRT_AVAILABLE = True
except ImportError:
    pass

try:
    import onnxruntime     # noqa: F401
    ONNX_AVAILABLE = True
except ImportError:
    pass

try:
    import openvino        # noqa: F401
    OPENVINO_AVAILABLE = True
except ImportError:
    pass


# ══════════════════════════════════════════════════════════════════
# Default configuration  (overridden by benchmark_config.yaml)
# ══════════════════════════════════════════════════════════════════

DEFAULT_CONFIG: dict = {
    "model": {
        # Path to weights.  Any of:
        #   LibreYOLOXs.pt  /  LibreRFDETRn.pth  /  LibreRFDETRn-seg.pth
        #   model.engine  /  model.onnx  /  model_openvino/
        "path": "LibreYOLOXs.pt",
        # Override number of classes (None = auto-detect from weights)
        "nb_classes": None,
        # "auto" | "cpu" | "cuda" | "0" | "1"
        "device": "auto",
        # RF-DETR size override (auto-detected from filename when null).
        # Values: "n" | "s" | "m" | "l" | "xl"
        # Only used when falling back to the native rfdetr API.
        "rfdetr_size": None,
    },
    "inference": {
        "conf": 0.25,
        "iou": 0.45,
        # null = model native resolution
        "imgsz": None,
        # null = all classes  |  list of int IDs to keep
        "classes": None,
        "max_det": 300,
        # tiled inference for large images (YOLOX / YOLOv9 only)
        "tiling": False,
        "overlap_ratio": 0.2,
    },
    "source": {
        # "fake"   — loop image_path as synthetic video
        # "image"  — static image file (runs num_frames inferences)
        # "video"  — video file (video_path)
        # "webcam" — USB / built-in camera (webcam_index)
        "type": "fake",
        # Used by "fake" and "image" modes
        "image_path": "tools.jpeg",
        # Used by "video" mode
        "video_path": None,
        # USB camera device index (0 = first camera)
        "webcam_index": 0,
        # How many frames to process (0 = unlimited for webcam)
        "num_frames": 100,
        # Pre-resize frames to [width, height] before inference.
        # Crucial for large static images: tools.jpeg is 4032x3024; without
        # resizing, the preprocessing alone costs ~100 ms/frame on Thor.
        # Set to the model's native res for a pure-inference benchmark.
        # null = no resize (raw frame size sent to model).
        "pre_resize": None,
    },
    "output": {
        # Display a live OpenCV window while running
        "show_stream": False,
        "window_name": "LibreYOLO Benchmark",
        # Save annotated frames to disk
        "save_frames": False,
        "save_dir": "runs/benchmark",
        # Maximum annotated frames to write (saves I/O time)
        "save_max_frames": 5,
    },
    "benchmark": {
        # Warm-up frames excluded from timing statistics
        "warmup_frames": 5,
        # Rolling window for avg/min/max/p95 computation
        "window_size": 30,
    },
    "segmentation": {
        # Render instance masks when the model produces them
        "enabled": True,
        # Mask overlay transparency  (0 = invisible, 1 = opaque)
        "mask_alpha": 0.4,
    },
}


# ══════════════════════════════════════════════════════════════════
# Config helpers
# ══════════════════════════════════════════════════════════════════

def _deep_merge(base: dict, override: dict) -> dict:
    result = dict(base)
    for k, v in override.items():
        if k in result and isinstance(result[k], dict) and isinstance(v, dict):
            result[k] = _deep_merge(result[k], v)
        else:
            result[k] = v
    return result


def load_config(config_path: Optional[str] = None) -> dict:
    cfg = DEFAULT_CONFIG

    if config_path is None:
        candidate = Path(__file__).parent / "benchmark_config.yaml"
        if candidate.exists():
            config_path = str(candidate)

    if config_path and Path(config_path).exists():
        print(f"Config : {config_path}")
        with open(config_path) as fh:
            user = yaml.safe_load(fh) or {}
        cfg = _deep_merge(DEFAULT_CONFIG, user)
    else:
        print("Config : built-in defaults (no benchmark_config.yaml found)")

    return cfg


# ══════════════════════════════════════════════════════════════════
# Performance monitor
# ══════════════════════════════════════════════════════════════════

class PerformanceMonitor:
    def __init__(self, window_size: int = 30):
        self.inference_times: deque = deque(maxlen=window_size)
        self.ram_samples: deque = deque(maxlen=window_size)
        self.gpu_mem_samples: deque = deque(maxlen=window_size)
        self._process = psutil.Process(os.getpid())
        self.start_time = time.time()

    def record_inference(self, ms: float):
        self.inference_times.append(ms)

    def sample_memory(self) -> float:
        try:
            rss_mb = self._process.memory_info().rss / (1024 * 1024)
            self.ram_samples.append(rss_mb)
        except Exception:
            rss_mb = 0.0
        if TORCH_AVAILABLE and torch.cuda.is_available():
            gpu_mb = torch.cuda.memory_allocated() / (1024 * 1024)
            self.gpu_mem_samples.append(gpu_mb)
        return rss_mb

    def get_stats(self) -> dict:
        s: dict = {}
        if self.inference_times:
            arr = np.asarray(self.inference_times)
            avg = float(np.mean(arr))
            s["avg_inference_ms"] = avg
            s["fps"] = 1000.0 / avg if avg > 0 else 0.0
            s["min_inference_ms"] = float(np.min(arr))
            s["max_inference_ms"] = float(np.max(arr))
            s["p95_inference_ms"] = float(np.percentile(arr, 95))
        else:
            s.update(avg_inference_ms=0, fps=0, min_inference_ms=0,
                     max_inference_ms=0, p95_inference_ms=0)
        if self.ram_samples:
            s["avg_ram_mb"] = float(np.mean(self.ram_samples))
            s["peak_ram_mb"] = float(np.max(self.ram_samples))
        else:
            s.update(avg_ram_mb=0, peak_ram_mb=0)
        if self.gpu_mem_samples:
            s["avg_gpu_mb"] = float(np.mean(self.gpu_mem_samples))
            s["peak_gpu_mb"] = float(np.max(self.gpu_mem_samples))
        try:
            s["cpu_percent"] = self._process.cpu_percent(interval=None)
        except Exception:
            s["cpu_percent"] = 0.0
        return s

    def print_summary(self, num_frames: int):
        s = self.get_stats()
        elapsed = time.time() - self.start_time
        print("\n" + "=" * 70)
        print("LIBREYOLO INFERENCE BENCHMARK SUMMARY")
        print("=" * 70)
        print(f"Timestamp : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"Frames    : {num_frames}  |  Elapsed: {elapsed:.2f}s")
        print()
        print("INFERENCE SPEED:")
        print(f"  Avg  : {s['avg_inference_ms']:8.2f} ms  ({s['fps']:.2f} FPS)")
        print(f"  Min  : {s['min_inference_ms']:8.2f} ms")
        print(f"  Max  : {s['max_inference_ms']:8.2f} ms")
        print(f"  P95  : {s['p95_inference_ms']:8.2f} ms")
        print()
        print("MEMORY USAGE:")
        print(f"  Avg RAM  : {s['avg_ram_mb']:7.1f} MB")
        print(f"  Peak RAM : {s['peak_ram_mb']:7.1f} MB")
        if "avg_gpu_mb" in s:
            print(f"  Avg GPU  : {s['avg_gpu_mb']:7.1f} MB")
            print(f"  Peak GPU : {s['peak_gpu_mb']:7.1f} MB")
        print()
        print(f"CPU       : {s['cpu_percent']:.1f}%")
        print("=" * 70 + "\n")
        return s


# ══════════════════════════════════════════════════════════════════
# Colour palette  (deterministic, COCO-style)
# ══════════════════════════════════════════════════════════════════

_RNG = np.random.default_rng(42)
_PALETTE: np.ndarray = _RNG.integers(60, 230, size=(256, 3), dtype=np.uint8)


def _color(cls_id: int) -> Tuple[int, int, int]:
    c = _PALETTE[int(cls_id) % len(_PALETTE)]
    return int(c[0]), int(c[1]), int(c[2])


# ══════════════════════════════════════════════════════════════════
# Visualisation helpers
# ══════════════════════════════════════════════════════════════════

def draw_detections(frame: np.ndarray, result, names: dict) -> np.ndarray:
    """Draw bounding boxes + labels on *frame* (in-place copy)."""
    out = frame.copy()
    try:
        if len(result) == 0:
            return out
        boxes = result.boxes
        xyxy  = boxes.xyxy.cpu().numpy() if hasattr(boxes.xyxy, "cpu") else np.asarray(boxes.xyxy)
        confs = boxes.conf.cpu().numpy() if hasattr(boxes.conf, "cpu") else np.asarray(boxes.conf)
        clss  = boxes.cls.cpu().numpy()  if hasattr(boxes.cls,  "cpu") else np.asarray(boxes.cls)
        for box, conf, cls_id in zip(xyxy, confs, clss):
            x1, y1, x2, y2 = map(int, box)
            color = _color(cls_id)
            cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
            label = f"{names.get(int(cls_id), str(int(cls_id)))}: {conf:.2f}"
            (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1)
            cv2.rectangle(out, (x1, y1 - lh - 6), (x1 + lw + 2, y1), color, -1)
            cv2.putText(out, label, (x1 + 1, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)
    except Exception as exc:
        print(f"  [draw_detections] {exc}")
    return out


def draw_segmentation(frame: np.ndarray, result, seg_cfg: dict) -> np.ndarray:
    """
    Overlay instance segmentation masks when the model produces them.
    Compatible with RF-DETR-seg results that expose result.masks.data
    as a (N, H, W) binary tensor.
    """
    alpha = float(seg_cfg.get("mask_alpha", 0.4))
    try:
        masks_obj = getattr(result, "masks", None)
        if masks_obj is None:
            return frame

        # masks.data  →  (N, H, W) tensor / array
        masks_data = getattr(masks_obj, "data", masks_obj)
        if hasattr(masks_data, "cpu"):
            masks_data = masks_data.cpu().numpy()
        masks_data = np.asarray(masks_data)

        if masks_data.ndim != 3 or masks_data.shape[0] == 0:
            return frame

        # Class IDs for per-instance colouring (fall back to index)
        try:
            clss = result.boxes.cls.cpu().numpy() if hasattr(result.boxes.cls, "cpu") \
                else np.asarray(result.boxes.cls)
        except Exception:
            clss = np.arange(masks_data.shape[0])

        overlay = frame.copy()
        for i, mask in enumerate(masks_data):
            cls_id = int(clss[i]) if i < len(clss) else i
            color = _color(cls_id)
            # Resize mask to match frame if necessary
            if mask.shape[:2] != frame.shape[:2]:
                mask = cv2.resize(
                    mask.astype(np.uint8),
                    (frame.shape[1], frame.shape[0]),
                    interpolation=cv2.INTER_NEAREST,
                )
            overlay[mask.astype(bool)] = color

        return cv2.addWeighted(frame, 1.0 - alpha, overlay, alpha, 0)
    except Exception as exc:
        print(f"  [draw_segmentation] {exc}")
        return frame


def add_hud(frame: np.ndarray, frame_idx: int, total: int,
            stats: dict, num_det: int, seg_active: bool) -> np.ndarray:
    """Overlay a minimal heads-up display on *frame*."""
    out = frame.copy()
    lines = [
        f"Frame : {frame_idx}/{total or '?'}",
        f"FPS   : {stats.get('fps', 0):.1f}",
        f"Infer : {stats.get('avg_inference_ms', 0):.1f} ms (p95 {stats.get('p95_inference_ms', 0):.1f})",
        f"RAM   : {stats.get('avg_ram_mb', 0):.0f} MB",
        f"Det   : {num_det}",
    ]
    if seg_active:
        lines.append("SEG   : ON")
    if "avg_gpu_mb" in stats:
        lines.append(f"GPU   : {stats['avg_gpu_mb']:.0f} MB")

    for i, text in enumerate(lines):
        y = 20 + i * 20
        cv2.putText(out, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.52, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.52, (0, 245, 180), 1, cv2.LINE_AA)
    return out


# ══════════════════════════════════════════════════════════════════
# Frame source generators
# ══════════════════════════════════════════════════════════════════

def _make_synthetic_frame() -> np.ndarray:
    """Create a labelled synthetic BGR frame as fallback."""
    f = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(f, (80, 80), (320, 340), (0, 200, 80), 3)
    cv2.circle(f, (480, 240), 70, (200, 80, 0), -1)
    cv2.putText(f, "Synthetic Test Frame", (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 180, 255), 2)
    return f


def get_frame_source(
    cfg: dict,
) -> Tuple[Generator, int]:
    """
    Return (generator_of_frames, total_frame_count).

    Each iteration yields a BGR numpy array ready for inference.
    total_frame_count == 0 means unlimited (webcam with num_frames=0).
    """
    src = cfg["source"]
    src_type: str = src.get("type", "fake")
    num_frames: int = int(src.get("num_frames", 100))
    script_dir = Path(__file__).parent

    # ── fake / image ──────────────────────────────────────────────
    if src_type in ("fake", "image"):
        img_path = src.get("image_path", "tools.jpeg")
        # Resolve relative path from script directory
        full_path = Path(img_path)
        if not full_path.is_absolute():
            candidate = script_dir / img_path
            if candidate.exists():
                full_path = candidate

        base_frame: Optional[np.ndarray] = None
        if full_path.exists():
            base_frame = cv2.imread(str(full_path))
            if base_frame is None:
                print(f"  Warning: cv2.imread failed for '{full_path}'")

        if base_frame is None:
            print(f"  Warning: '{full_path}' not loadable — using synthetic frame")
            base_frame = _make_synthetic_frame()

        _raw_w, _raw_h = base_frame.shape[1], base_frame.shape[0]
        pre_resize = src.get("pre_resize") or None
        if pre_resize:
            rw, rh = int(pre_resize[0]), int(pre_resize[1])
            base_frame = cv2.resize(base_frame, (rw, rh), interpolation=cv2.INTER_LINEAR)
            print(f"  Source : '{full_path}' (raw {_raw_w}x{_raw_h} → pre-resized to {rw}x{rh})"
                  f" — repeating {num_frames}x")
        else:
            print(f"  Source : '{full_path}' ({_raw_w}x{_raw_h} — no pre_resize, may be slow)"
                  f" — repeating {num_frames}x")

        def _image_gen() -> Generator:
            for i in range(num_frames):
                f = base_frame.copy()
                cv2.putText(f, f"{i+1}/{num_frames}",
                            (base_frame.shape[1] - 120, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
                yield f
        return _image_gen(), num_frames

    # ── webcam ────────────────────────────────────────────────────
    elif src_type == "webcam":
        idx: int = int(src.get("webcam_index", 0))
        print(f"  Source : webcam index {idx}")
        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            print(f"ERROR: Cannot open webcam at index {idx}")
            sys.exit(1)
        # Request a common high-quality resolution; camera may cap it
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"  Camera : {w}x{h}  (index {idx})")

        def _webcam_gen() -> Generator:
            i = 0
            _pre = src.get("pre_resize") or None
            _resize_to = (int(_pre[0]), int(_pre[1])) if _pre else None
            try:
                while num_frames <= 0 or i < num_frames:
                    ret, frame = cap.read()
                    if not ret:
                        print("  Warning: webcam read failed — stopping")
                        break
                    if _resize_to:
                        frame = cv2.resize(frame, _resize_to, interpolation=cv2.INTER_LINEAR)
                    i += 1
                    yield frame
            finally:
                cap.release()
        return _webcam_gen(), num_frames

    # ── video file ────────────────────────────────────────────────
    elif src_type == "video":
        video_path: str = src.get("video_path", "") or ""
        if not video_path or not Path(video_path).exists():
            print(f"ERROR: video_path '{video_path}' not found")
            sys.exit(1)
        print(f"  Source : video '{video_path}'")
        cap = cv2.VideoCapture(video_path)
        file_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        total = min(file_frames, num_frames) if num_frames > 0 else file_frames

        def _video_gen() -> Generator:
            i = 0
            try:
                while num_frames <= 0 or i < num_frames:
                    ret, frame = cap.read()
                    if not ret:
                        break
                    i += 1
                    yield frame
            finally:
                cap.release()
        return _video_gen(), total

    else:
        print(f"ERROR: Unknown source type '{src_type}'. Use: fake | image | video | webcam")
        sys.exit(1)


# ══════════════════════════════════════════════════════════════════
# Native RF-DETR wrapper  (rfdetr ≥ 1.4.x / 1.6.x class-based API)
# ══════════════════════════════════════════════════════════════════
# libreyolo's internal rfdetr adapter uses `from rfdetr.main import Model`
# which was removed in rfdetr 1.4.x.  When that ImportError is raised we
# fall back to calling rfdetr's public classes (RFDETRNano, RFDETRSegMedium…)
# directly and wrap their supervision.Detections results in thin proxies.

class _NativeBoxes:
    """Bounding-box proxy compatible with draw_detections()."""
    def __init__(self, xyxy, conf, cls_ids):
        if TORCH_AVAILABLE:
            to = lambda a, dt: torch.from_numpy(np.asarray(a, dtype=dt)) if len(a) else torch.zeros(0)
            self.xyxy = torch.from_numpy(np.asarray(xyxy, dtype=np.float32)) if len(xyxy) else torch.zeros((0, 4))
            self.conf = to(conf, np.float32)
            self.cls  = to(cls_ids, np.float32)
        else:
            self.xyxy = np.asarray(xyxy,    dtype=np.float32)
            self.conf = np.asarray(conf,    dtype=np.float32)
            self.cls  = np.asarray(cls_ids, dtype=np.float32)
    def cpu(self):   return self
    def numpy(self): return self
    def __len__(self): return len(self.conf)


class _NativeMasks:
    """Mask proxy compatible with draw_segmentation()."""
    def __init__(self, mask_array):
        # supervision stores masks as (N, H, W) bool numpy array
        arr = mask_array.astype(np.float32)
        self.data = torch.from_numpy(arr) if TORCH_AVAILABLE else arr


class _NativeResult:
    """Wraps supervision.Detections to look like a LibreYOLO Results object."""
    def __init__(self, detections, names: dict):
        xyxy    = detections.xyxy         if detections.xyxy         is not None else np.zeros((0, 4))
        conf    = detections.confidence   if detections.confidence   is not None else np.zeros(0)
        cls_ids = detections.class_id.astype(float) if detections.class_id is not None else np.zeros(0)
        self.boxes = _NativeBoxes(xyxy, conf, cls_ids)
        mask_arr   = getattr(detections, "mask", None)
        self.masks = _NativeMasks(mask_arr) if (mask_arr is not None and mask_arr.shape[0] > 0) else None
        self.names = names
    def cpu(self): return self
    def __len__(self): return len(self.boxes)


class NativeRFDETRWrapper:
    """
    Inference wrapper for rfdetr ≥ 1.4.x (the new class-based API).

    The model size is auto-inferred from the path name (LibreRFDETRl-seg → 'l')
    or can be overridden via cfg['model']['rfdetr_size'].

    Size codes: 'n' (Nano)  's' (Small)  'm' (Medium)  'l' (Large)  'xl' (XLarge)
    """

    _DET_MAP = {
        'n': 'RFDETRNano',   's': 'RFDETRSmall',
        'm': 'RFDETRMedium', 'l': 'RFDETRLarge',
        'xl': 'RFDETRXLarge',
    }
    _SEG_MAP = {
        'n': 'RFDETRSegNano',   's': 'RFDETRSegSmall',
        'm': 'RFDETRSegMedium', 'l': 'RFDETRSegLarge',
        'xl': 'RFDETRSegXLarge',
    }

    @staticmethod
    def _infer_size(path: str) -> str:
        stem = Path(path.lower()).stem.replace('-seg', '').replace('_seg', '')
        for code in ('xl', 'l', 'm', 's', 'n'):
            if stem.endswith(code):
                return code
        return 'm'

    def __init__(self, path: str, is_seg: bool, device: str = 'auto',
                 rfdetr_size: Optional[str] = None):
        import rfdetr as rfdetr_pkg
        try:
            from PIL import Image as _PILImage
        except ImportError:
            print("ERROR: Pillow not installed.  pip install pillow")
            sys.exit(1)
        self._PIL = _PILImage

        size     = (rfdetr_size or self._infer_size(path)).lower()
        cls_map  = self._SEG_MAP if is_seg else self._DET_MAP
        cls_name = cls_map.get(size, cls_map['m'])

        ModelCls = getattr(rfdetr_pkg, cls_name, None)
        if ModelCls is None:
            raise ImportError(
                f"rfdetr class '{cls_name}' not found in rfdetr {getattr(rfdetr_pkg, '__version__', '?')}.\n"
                f"Upgrade rfdetr: pip install --upgrade rfdetr"
            )

        print(f"  Native rfdetr class : {cls_name}  (size='{size}', seg={is_seg})")
        self._model = ModelCls()
        self._is_seg = is_seg

        try:
            from rfdetr.assets.coco_classes import COCO_CLASSES
            self.names = {i: n for i, n in enumerate(COCO_CLASSES)}
        except Exception:
            self.names = {}

    def __call__(self, frame: np.ndarray, *, conf: float = 0.25, **_ignored):
        rgb     = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pil_img = self._PIL.fromarray(rgb)
        dets    = self._model.predict(pil_img, threshold=conf)
        return _NativeResult(dets, self.names)


# ══════════════════════════════════════════════════════════════════
# Model loader
# ══════════════════════════════════════════════════════════════════

def load_model(cfg: dict):
    """
    Load a model.  Returns (model, is_rfdetr: bool, is_seg: bool).

    Priority:
      1. Try LibreYOLO() factory (handles YOLOX / YOLOv9 / RF-DETR via its own adapters,
         plus .engine / .onnx / OpenVINO backends).
      2. If LibreYOLO raises a ModuleNotFoundError for 'rfdetr.main' (API mismatch
         between libreyolo and rfdetr ≥ 1.4.x), fall back to NativeRFDETRWrapper
         which calls rfdetr's public class API directly.
    """
    mcfg = cfg["model"]
    path: str        = mcfg["path"]
    device: str      = mcfg.get("device", "auto")
    nb_classes       = mcfg.get("nb_classes") or None
    rfdetr_size: str = mcfg.get("rfdetr_size") or None

    pl = path.lower()
    is_engine   = pl.endswith(".engine")
    is_onnx     = pl.endswith(".onnx")
    is_openvino = Path(path).is_dir() or pl.endswith("_openvino")
    is_rfdetr   = "rfdetr" in pl
    is_seg      = "seg" in pl

    print()
    print(f"Model  : {path}")
    print(f"Device : {device}")

    if is_rfdetr:
        tag = "RF-DETR segmentation" if is_seg else "RF-DETR detection"
        print(f"Family : {tag}")
        if not RFDETR_AVAILABLE:
            print("  [!] rfdetr not installed — pip install libreyolo[rfdetr]")

    if is_engine:
        print("Backend: TensorRT (.engine)")
        if not TENSORRT_AVAILABLE:
            print("  [!] TensorRT not available — pip install libreyolo[tensorrt]")
    elif is_onnx:
        print("Backend: ONNX Runtime (.onnx)")
        if not ONNX_AVAILABLE:
            print("  [!] onnxruntime not installed — pip install libreyolo[onnx]")
    elif is_openvino:
        print("Backend: OpenVINO")
        if not OPENVINO_AVAILABLE:
            print("  [!] openvino not installed — pip install libreyolo[openvino]")
    else:
        print("Backend: PyTorch")

    kwargs: dict = {}
    if nb_classes is not None:
        kwargs["nb_classes"] = nb_classes
    if not is_openvino:
        kwargs["device"] = device

    # ── Attempt 1: LibreYOLO factory ────────────────────────────
    try:
        model = LibreYOLO(path, **kwargs)
        print("  Model loaded OK  [LibreYOLO]")
        return model, is_rfdetr, is_seg

    except (ModuleNotFoundError, ImportError) as exc:
        # libreyolo's rfdetr adapter uses the removed `rfdetr.main.Model` API.
        # Detect this specific error and fall back to calling rfdetr directly.
        if is_rfdetr and RFDETR_NEW_API and "rfdetr.main" in str(exc):
            print(f"  [!] libreyolo↔rfdetr API mismatch  ({exc})")
            print("  → rfdetr ≥ 1.4.x detected — bypassing libreyolo adapter...")
            model = NativeRFDETRWrapper(
                path, is_seg=is_seg, device=device, rfdetr_size=rfdetr_size
            )
            print("  Model loaded OK  [native rfdetr API]")
            return model, is_rfdetr, is_seg
        # Any other ImportError (missing optional dep, wrong model name, etc.) — re-raise
        raise


# ══════════════════════════════════════════════════════════════════
# Main benchmark loop
# ══════════════════════════════════════════════════════════════════

def run_benchmark(config_path: Optional[str] = None) -> dict:
    cfg = load_config(config_path)

    # ── Banner ────────────────────────────────────────────────────
    print("\n" + "=" * 70)
    print("LIBREYOLO INFERENCE BENCHMARK")
    print("=" * 70)
    if TORCH_AVAILABLE:
        cuda_info = (f"YES — {torch.cuda.get_device_name(0)}"
                     if torch.cuda.is_available() else "no")
        print(f"PyTorch      : {torch.__version__}")
        print(f"CUDA         : {cuda_info}")
    if RFDETR_AVAILABLE:
        api_tag = "new API ≥ 1.4.x (native fallback available)" if RFDETR_NEW_API \
                  else "old API ≤ 1.3.x (libreyolo adapter only)"
        print(f"RF-DETR      : installed — {api_tag}")
    else:
        print("RF-DETR      : not installed  (pip install libreyolo[rfdetr])")
    print(f"TensorRT     : {'installed' if TENSORRT_AVAILABLE else 'not installed  (pip install libreyolo[tensorrt])'}")
    print(f"ONNX Runtime : {'installed' if ONNX_AVAILABLE else 'not installed  (pip install libreyolo[onnx])'}")
    print(f"OpenVINO     : {'installed' if OPENVINO_AVAILABLE else 'not installed  (pip install libreyolo[openvino])'}")
    print("=" * 70)

    # ── Load model ────────────────────────────────────────────────
    model, is_rfdetr, is_seg = load_model(cfg)

    # Class names (best-effort)
    try:
        names: dict = model.names if (hasattr(model, "names") and model.names) else {}
    except Exception:
        names = {}

    # ── Inference kwargs ──────────────────────────────────────────
    inf = cfg["inference"]
    infer_kwargs: dict = {
        "conf": inf.get("conf", 0.25),
        "iou":  inf.get("iou",  0.45),
        "max_det": inf.get("max_det", 300),
        "color_format": "bgr",
    }
    if inf.get("imgsz"):
        infer_kwargs["imgsz"] = inf["imgsz"]
    if inf.get("classes"):
        infer_kwargs["classes"] = inf["classes"]
    if inf.get("tiling"):
        infer_kwargs["tiling"] = True
        infer_kwargs["overlap_ratio"] = inf.get("overlap_ratio", 0.2)

    # ── Setup ─────────────────────────────────────────────────────
    bcfg = cfg["benchmark"]
    monitor = PerformanceMonitor(window_size=int(bcfg.get("window_size", 30)))
    warmup: int = int(bcfg.get("warmup_frames", 5))

    frame_gen, total_frames = get_frame_source(cfg)

    ocfg = cfg["output"]
    show_stream: bool = bool(ocfg.get("show_stream", False))
    save_frames: bool = bool(ocfg.get("save_frames", False))
    save_dir    = Path(ocfg.get("save_dir", "runs/benchmark"))
    save_max    = int(ocfg.get("save_max_frames", 5))
    win_name: str = ocfg.get("window_name", "LibreYOLO Benchmark")

    seg_cfg = cfg["segmentation"]
    do_seg: bool = bool(seg_cfg.get("enabled", True))

    if save_frames:
        save_dir.mkdir(parents=True, exist_ok=True)
    if show_stream:
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
        print(f"\nLive view ON  —  press  q  to quit early")

    print(f"\nSource  : {cfg['source']['type']}")
    print(f"Frames  : {total_frames}  (warmup: {warmup})")
    print(f"Stream  : {'ON' if show_stream else 'off'}")
    print(f"Seg     : {'ON' if (do_seg and is_seg) else ('off' if not is_seg else 'OFF (disabled in cfg)')}")
    print()

    # ── Inference loop ────────────────────────────────────────────
    frame_idx   = 0   # raw counter (includes warmup)
    counted_idx = 0   # frames that count toward benchmark stats
    det_log: list = []
    seg_detected = False

    try:
        for frame in frame_gen:
            frame_idx += 1
            is_warmup = frame_idx <= warmup

            ram_mb = monitor.sample_memory()

            # CUDA sync before timing
            if TORCH_AVAILABLE and torch.cuda.is_available():
                torch.cuda.synchronize()

            t0 = time.perf_counter()
            try:
                result = model(frame, **infer_kwargs)
            except Exception as exc:
                print(f"  ERROR frame {frame_idx}: {exc}")
                traceback.print_exc()
                continue

            if TORCH_AVAILABLE and torch.cuda.is_available():
                torch.cuda.synchronize()

            elapsed_ms = (time.perf_counter() - t0) * 1000.0

            if not is_warmup:
                monitor.record_inference(elapsed_ms)
                counted_idx += 1

            # Normalise result to single Results object
            r = result[0] if isinstance(result, list) else result
            num_det = len(r) if r is not None else 0

            # Check for segmentation masks
            has_masks = (
                do_seg and is_seg
                and r is not None
                and getattr(r, "masks", None) is not None
            )
            if has_masks and not seg_detected:
                seg_detected = True
                print(f"  Segmentation masks detected  (frame {frame_idx})")

            if not is_warmup:
                det_log.append({
                    "frame": frame_idx,
                    "num_detections": num_det,
                    "inference_ms": elapsed_ms,
                    "ram_mb": ram_mb,
                })

            # ── Visualisation ─────────────────────────────────────
            if (show_stream or (save_frames and counted_idx <= save_max)):
                vis = frame.copy()
                if has_masks:
                    vis = draw_segmentation(vis, r, seg_cfg)
                if r is not None:
                    vis = draw_detections(vis, r, names)
                if not is_warmup:
                    vis = add_hud(vis, counted_idx, max(0, total_frames - warmup),
                                  monitor.get_stats(), num_det, has_masks)
                else:
                    cv2.putText(vis, f"WARMUP {frame_idx}/{warmup}", (8, 22),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 100, 255), 2)

                if show_stream:
                    cv2.imshow(win_name, vis)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        print("  Stream closed by user (q)")
                        break

                if save_frames and not is_warmup and counted_idx <= save_max:
                    out_path = save_dir / f"frame_{frame_idx:05d}.jpg"
                    cv2.imwrite(str(out_path), vis)

            # ── Console progress (every ~10 % of benchmark frames) ─
            benchmark_total = max(1, total_frames - warmup)
            log_every = max(1, benchmark_total // 10)
            if not is_warmup and (counted_idx % log_every == 0 or counted_idx == 1):
                s = monitor.get_stats()
                print(f"Frame {frame_idx:5d}/{total_frames or '?'} | "
                      f"Det: {num_det:3d} | "
                      f"Infer: {elapsed_ms:7.2f} ms | "
                      f"FPS: {s.get('fps', 0):6.2f} | "
                      f"RAM: {ram_mb:7.1f} MB")
            elif is_warmup:
                print(f"  [warmup {frame_idx}/{warmup}]  {elapsed_ms:.1f} ms")

    except KeyboardInterrupt:
        print("\n  Interrupted by user")
    finally:
        if show_stream:
            cv2.destroyAllWindows()

    # ── Final summary ─────────────────────────────────────────────
    if det_log:
        monitor.print_summary(counted_idx)
        det_counts = [d["num_detections"] for d in det_log]
        print("DETECTION SUMMARY:")
        print(f"  Total detections : {sum(det_counts)}")
        print(f"  Avg per frame    : {np.mean(det_counts):.2f}")
        print(f"  Max in frame     : {max(det_counts)}")
        print(f"  Segmentation     : {'YES — masks rendered' if seg_detected else 'NO (detection-only model)'}")
        if is_rfdetr:
            print(f"  RF-DETR model    : {'seg' if is_seg else 'det'}")
        print()
    else:
        print("No frames were processed successfully.")

    return monitor.get_stats()


# ══════════════════════════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="LibreYOLO Inference Benchmark — YAML-configurable",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python libreyolo_inference_benchmark.py
  python libreyolo_inference_benchmark.py --config benchmark_config.yaml
  python libreyolo_inference_benchmark.py --config my_rfdetr_seg.yaml
        """,
    )
    parser.add_argument(
        "--config", "-c",
        type=str,
        default=None,
        help="Path to a YAML config file (default: benchmark_config.yaml next to this script)",
    )
    args = parser.parse_args()
    run_benchmark(args.config)
