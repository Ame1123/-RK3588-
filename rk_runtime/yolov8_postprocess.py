"""
YOLOv8 utilities: letterbox preprocess + standard-output postprocess + draw.

Designed for the RKNN-exported model that keeps YOLOv8's default output
shape [1, 4+nc, 8400] (xywh + per-class sigmoid scores, three detection
heads already concatenated).

If you later re-export with `format=rknn` (9-branch output) for maximum
speed, replace `decode_yolov8` accordingly — the rest of the pipeline
(preprocess / NMS / draw) stays the same.
"""
from __future__ import annotations

import cv2
import numpy as np

INPUT_SIZE = 640  # must match what export_onnx_fixed.py used


def letterbox(image: np.ndarray, new_size: int = INPUT_SIZE,
              color: tuple[int, int, int] = (114, 114, 114)) -> tuple[np.ndarray, float, int, int]:
    """Resize-with-pad to a square. Returns (padded_image, scale, pad_x, pad_y)."""
    h, w = image.shape[:2]
    scale = min(new_size / h, new_size / w)
    nh, nw = int(round(h * scale)), int(round(w * scale))
    resized = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_LINEAR)
    pad_x = (new_size - nw) // 2
    pad_y = (new_size - nh) // 2
    out = np.full((new_size, new_size, 3), color, dtype=np.uint8)
    out[pad_y:pad_y + nh, pad_x:pad_x + nw] = resized
    return out, scale, pad_x, pad_y


def decode_yolov8(raw: np.ndarray, conf_thres: float,
                  num_classes: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    raw: [1, 4+nc, 8400]  or  [4+nc, 8400]  (RKNN may strip batch dim)
    Returns boxes_xywh [N,4], scores [N], class_ids [N], all in 640-input coords.
    """
    a = np.squeeze(raw)
    if a.shape[0] != 4 + num_classes:
        # RKNN sometimes emits [8400, 4+nc] instead — auto-fix
        a = a.T
    xywh = a[:4, :].T               # [8400, 4]
    cls_scores = a[4:, :].T         # [8400, nc]
    # Already-sigmoid scores (YOLOv8 default export). Pick best class per anchor.
    class_ids = np.argmax(cls_scores, axis=1)
    scores = cls_scores[np.arange(cls_scores.shape[0]), class_ids]
    keep = scores > conf_thres
    return xywh[keep], scores[keep], class_ids[keep]


def nms_per_class(boxes_xywh: np.ndarray, scores: np.ndarray,
                  class_ids: np.ndarray, iou_thres: float) -> list[int]:
    """cv2.dnn.NMSBoxesBatched does class-aware NMS quickly. Falls back to per-class loop if unavailable."""
    if boxes_xywh.shape[0] == 0:
        return []
    # convert xywh (centre) -> xywh (top-left) for OpenCV NMS
    xs = boxes_xywh[:, 0] - boxes_xywh[:, 2] / 2
    ys = boxes_xywh[:, 1] - boxes_xywh[:, 3] / 2
    boxes_tl = np.stack([xs, ys, boxes_xywh[:, 2], boxes_xywh[:, 3]], axis=1)
    try:
        idx = cv2.dnn.NMSBoxesBatched(boxes_tl.tolist(),
                                      scores.tolist(),
                                      class_ids.tolist(),
                                      score_threshold=0.0,
                                      nms_threshold=iou_thres)
    except AttributeError:
        # Older OpenCV: do it per-class manually
        idx = []
        for c in np.unique(class_ids):
            mask = np.where(class_ids == c)[0]
            sub = cv2.dnn.NMSBoxes(boxes_tl[mask].tolist(),
                                   scores[mask].tolist(),
                                   score_threshold=0.0,
                                   nms_threshold=iou_thres)
            if len(sub):
                idx.extend(mask[np.array(sub).flatten()].tolist())
    return list(np.array(idx).flatten())


def scale_back(boxes_xywh: np.ndarray, scale: float, pad_x: int, pad_y: int) -> np.ndarray:
    """Undo letterbox: convert 640-space xywh into original-image xyxy."""
    if boxes_xywh.shape[0] == 0:
        return boxes_xywh.reshape(0, 4)
    cx, cy, w, h = boxes_xywh[:, 0], boxes_xywh[:, 1], boxes_xywh[:, 2], boxes_xywh[:, 3]
    x1 = (cx - w / 2 - pad_x) / scale
    y1 = (cy - h / 2 - pad_y) / scale
    x2 = (cx + w / 2 - pad_x) / scale
    y2 = (cy + h / 2 - pad_y) / scale
    return np.stack([x1, y1, x2, y2], axis=1)


def draw_detections(image: np.ndarray, boxes_xyxy: np.ndarray, scores: np.ndarray,
                    class_ids: np.ndarray, names: dict[int, str],
                    ice_class: str = "jiebing") -> np.ndarray:
    """Draw rectangles + labels on a copy of `image`."""
    if boxes_xyxy.shape[0] == 0:
        return image
    out = image.copy()
    h, w = out.shape[:2]
    for box, score, cid in zip(boxes_xyxy, scores, class_ids):
        x1, y1, x2, y2 = (int(max(0, min(v, lim - 1))) for v, lim in zip(box, (w, h, w, h)))
        name = names.get(int(cid), str(cid))
        color = (0, 0, 255) if name == ice_class else (0, 255, 0)
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        cv2.putText(out, f"{name} {score:.2f}", (x1, max(y1 - 5, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    return out
