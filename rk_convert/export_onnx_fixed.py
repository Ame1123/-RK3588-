#!/usr/bin/env python3
"""
Step 1: Re-export YOLOv8 .pt to ONNX with a FIXED input shape.

Run this on ANY machine that has `ultralytics` installed (Jetson, x86 PC,
or even the RK3588 itself — this step does not use the NPU).

Output:
    best_640.onnx  (next to this script)

Why we re-export:
    The existing best.onnx in models/ is dynamic-shape, which rknn_toolkit2
    refuses. RKNN requires a fully static input.
"""
from pathlib import Path
from ultralytics import YOLO

PT_PATH = Path(__file__).resolve().parent.parent / "model" / "best.pt"
OUT_DIR = Path(__file__).resolve().parent
IMGSZ = 640  # must match training; YOLOv8 default is 640

print(f"Loading {PT_PATH} ...")
model = YOLO(str(PT_PATH))
print("Exporting ONNX (fixed shape, opset=12) ...")
onnx_path = model.export(
    format="onnx",
    imgsz=IMGSZ,
    opset=12,
    dynamic=False,
    simplify=True,
)
target = OUT_DIR / f"best_{IMGSZ}.onnx"
Path(onnx_path).replace(target)
print(f"Done: {target}")
