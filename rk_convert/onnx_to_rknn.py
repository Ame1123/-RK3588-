#!/usr/bin/env python3
"""
Step 2: Convert best_640.onnx -> best.rknn for RK3588.

RUN THIS ON AN x86 LINUX PC, NOT ON THE RK3588.

Requirements (x86 PC, Python 3.8 / 3.10 / 3.11):
    pip install rknn-toolkit2==2.3.2

    The version MUST match the librknnrt.so on the RK3588 board.
    Confirmed on the target board: 2.3.2.
    Pip index:
        https://pypi.org/project/rknn-toolkit2/

Inputs:
    best_640.onnx       — produced by export_onnx_fixed.py
    calib_images/*.jpg  — (optional) calibration images for INT8 quantization

Outputs:
    best_fp.rknn   — FP16, no calibration needed     (this is what we use first)
    best_i8.rknn   — INT8, needs >=20 calibration images (optional later step)

Usage:
    python3 onnx_to_rknn.py            # build FP16 only
    python3 onnx_to_rknn.py --int8     # also build INT8 (needs calib_images/)
"""
import argparse
import sys
from pathlib import Path

from rknn.api import RKNN

HERE = Path(__file__).resolve().parent
ONNX_PATH = HERE / "best_640.onnx"
CALIB_DIR = HERE / "calib_images"
CALIB_LIST = HERE / "calib_list.txt"
FP_OUT = HERE / "best_fp.rknn"
I8_OUT = HERE / "best_i8.rknn"
TARGET = "rk3588"


def build(quantize: bool, out_path: Path) -> None:
    print(f"\n=== Building {'INT8' if quantize else 'FP16'} -> {out_path.name} ===")
    rknn = RKNN(verbose=True)

    # YOLOv8 inputs are normalized to 0..1, channel order RGB.
    # ultralytics' export uses no mean/std baked in, so we tell RKNN to do it.
    rknn.config(
        mean_values=[[0, 0, 0]],
        std_values=[[255, 255, 255]],
        target_platform=TARGET,
        optimization_level=3,
    )

    print(f"Loading ONNX: {ONNX_PATH}")
    if rknn.load_onnx(model=str(ONNX_PATH)) != 0:
        sys.exit("load_onnx failed")

    if quantize:
        if not CALIB_LIST.exists():
            sys.exit(f"INT8 build needs {CALIB_LIST} (one image path per line)")
        if rknn.build(do_quantization=True, dataset=str(CALIB_LIST)) != 0:
            sys.exit("INT8 build failed")
    else:
        if rknn.build(do_quantization=False) != 0:
            sys.exit("FP16 build failed")

    if rknn.export_rknn(str(out_path)) != 0:
        sys.exit("export_rknn failed")

    print(f"OK -> {out_path}")
    rknn.release()


def write_calib_list() -> bool:
    imgs = sorted([p for p in CALIB_DIR.glob("*") if p.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp"}])
    if not imgs:
        return False
    CALIB_LIST.write_text("\n".join(str(p) for p in imgs) + "\n")
    print(f"Wrote {CALIB_LIST} ({len(imgs)} images)")
    return True


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--int8", action="store_true", help="also build INT8 (needs calib_images/)")
    args = ap.parse_args()

    if not ONNX_PATH.exists():
        sys.exit(f"Missing {ONNX_PATH}. Run export_onnx_fixed.py first.")

    build(quantize=False, out_path=FP_OUT)

    if args.int8:
        if write_calib_list():
            build(quantize=True, out_path=I8_OUT)
        else:
            print(f"\nSkip INT8: drop >=20 .jpg files into {CALIB_DIR}/ and re-run with --int8.")


if __name__ == "__main__":
    main()
