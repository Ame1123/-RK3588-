"""
RKNNPool — Python equivalent of the C++ rknnPool from
https://github.com/JA-cmd-wq/yolov8-helmet-rk3588-multithread

Architecture (the part that fixes the camera-dropoff problem on RK3588):

    capture thread          rknn_pool                        consumer
    ┌──────────────┐    put(frame, ts)                ┌─────────────────┐
    │ cv2.Video... │ ──────────────► ThreadPool ────► │ get() blocks    │
    │  grab/retrve │                 ├ worker0 (NPU0) │ on oldest fut   │
    └──────────────┘                 ├ worker1 (NPU1) └─────────────────┘
                                     └ worker2 (NPU2)

Each worker owns ONE RKNNLite session pinned to one of the three NPU cores.
Frames are submitted asynchronously; the consumer drains results in FIFO
order. The capture loop never blocks on inference — that is what was
killing the USB camera in x3.py.

3 workers is the sweet spot on RK3588 (one per core). 4+ workers contend
for cores and slow down. The helmet project's "12 workers" only helps for
very heavy postprocessing — overkill here.
"""
from __future__ import annotations

import os
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor
from queue import Empty, Queue
from typing import Callable, Optional

import numpy as np
from rknnlite.api import RKNNLite

# RK3588 has 3 NPU cores
_CORE_MASKS = [RKNNLite.NPU_CORE_0, RKNNLite.NPU_CORE_1, RKNNLite.NPU_CORE_2]


class _RKNNWorker:
    """One RKNNLite session bound to one NPU core."""

    def __init__(self, model_path: str, core_index: int):
        self.core_index = core_index
        self.rknn = RKNNLite()
        if self.rknn.load_rknn(model_path) != 0:
            raise RuntimeError(f"load_rknn({model_path}) failed for core {core_index}")
        core_mask = _CORE_MASKS[core_index % len(_CORE_MASKS)]
        if self.rknn.init_runtime(core_mask=core_mask) != 0:
            raise RuntimeError(f"init_runtime core_mask={core_mask} failed")

    def infer(self, blob: np.ndarray) -> list[np.ndarray]:
        # blob must be NHWC uint8 (RKNN does mean/std conversion internally)
        return self.rknn.inference(inputs=[blob], data_format=["nhwc"])

    def release(self) -> None:
        try:
            self.rknn.release()
        except Exception:
            pass


class RKNNPool:
    """
    FIFO inference pool. Use `put(frame, meta)` to submit, `get()` to
    drain results in submission order.

    `meta` is anything you want to ride along with the frame (timestamp,
    raw frame copy, letterbox params, …) — it comes back unchanged
    alongside the inference outputs.
    """

    def __init__(self, model_path: str, num_workers: int = 3,
                 preprocess: Optional[Callable[[np.ndarray], tuple]] = None):
        if not os.path.exists(model_path):
            raise FileNotFoundError(
                f"RKNN model not found: {model_path}\n"
                "Run rk_convert/ to generate best.rknn and copy it to models/best.rknn"
            )
        self.num_workers = max(1, num_workers)
        self.preprocess = preprocess
        self.workers = [_RKNNWorker(model_path, i) for i in range(self.num_workers)]
        self.executor = ThreadPoolExecutor(max_workers=self.num_workers)
        self._round_robin = 0
        self._lock = threading.Lock()
        self._futs: Queue[tuple[Future, object]] = Queue()
        self._closed = False

    def _run(self, worker: _RKNNWorker, frame: np.ndarray, meta) -> tuple[list[np.ndarray], object]:
        if self.preprocess is not None:
            blob, meta_extra = self.preprocess(frame)
            full_meta = (meta, meta_extra)
        else:
            blob = frame
            full_meta = meta
        outs = worker.infer(blob)
        return outs, full_meta

    def put(self, frame: np.ndarray, meta=None) -> bool:
        if self._closed:
            return False
        with self._lock:
            worker = self.workers[self._round_robin % self.num_workers]
            self._round_robin += 1
        fut = self.executor.submit(self._run, worker, frame, meta)
        self._futs.put((fut, meta))
        return True

    def get(self, timeout: float = 1.0):
        """Returns (outputs, meta) for the oldest submitted frame, or None on timeout."""
        try:
            fut, _ = self._futs.get(timeout=timeout)
        except Empty:
            return None
        try:
            return fut.result(timeout=timeout * 4)
        except Exception as e:
            print(f"[RKNNPool] inference failed: {e}")
            return None

    def qsize(self) -> int:
        return self._futs.qsize()

    def release(self) -> None:
        self._closed = True
        self.executor.shutdown(wait=True)
        for w in self.workers:
            w.release()


# --------------------------- self-test ---------------------------
if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("usage: python3 rknn_pool.py <best.rknn>")
        sys.exit(1)

    pool = RKNNPool(sys.argv[1], num_workers=3)
    # 1×3×640×640 inference takes a NHWC uint8 blob: 1×640×640×3
    blob = np.random.randint(0, 255, (1, 640, 640, 3), dtype=np.uint8)
    t0 = time.time()
    N = 60
    for i in range(N):
        pool.put(blob, meta=i)
    for _ in range(N):
        out = pool.get()
        assert out is not None
    dt = time.time() - t0
    print(f"{N} frames in {dt:.2f}s -> {N/dt:.1f} FPS (random input)")
    pool.release()
