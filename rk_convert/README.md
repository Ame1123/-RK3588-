# ONNX → RKNN 转换说明（在 x86 PC 上跑）

> 这一步**不能在 RK3588 板子上完成**。板子上只装了 `rknn-toolkit-lite2`（推理 API），
> 真正负责把 ONNX 转成 RKNN 的 `rknn-toolkit2` 只发布了 x86_64 Linux 版本。

## 1. 把这个目录拷到 PC

整个 `rk_convert/` 拷到一台 **x86_64 Linux**（Ubuntu 20.04 / 22.04 推荐）或 Windows 上的 WSL2。

也需要带上模型文件：
```
rk_convert/
├── export_onnx_fixed.py
├── onnx_to_rknn.py
├── README.md          # 本文件
└── calib_images/      # （仅 INT8 需要，可后补）
../model/best.pt       # 一起拷过去
```

## 2. PC 上安装环境

```bash
# Python 3.8 / 3.10 / 3.11 任选一个，建议用 venv 隔离
python3 -m venv rk_env
source rk_env/bin/activate

pip install ultralytics                # 用来重新导出 ONNX
pip install rknn-toolkit2==2.3.2       # ★ 版本必须是 2.3.2，跟板子上的 librknnrt.so 配套
```

如果 pip 装不上，去 Rockchip 官方仓库直接拉 wheel：
<https://github.com/airockchip/rknn-toolkit2/tree/v2.3.2/rknn-toolkit2/packages>
选对应 Python 版本的 `rknn_toolkit2-2.3.2-cp310-cp310-linux_x86_64.whl` 这种文件，`pip install <wheel>`。

## 3. 重新导出固定 shape 的 ONNX

```bash
cd rk_convert
python3 export_onnx_fixed.py
```

会生成 `best_640.onnx`（input 形状固定为 `1×3×640×640`）。

> 现有的 `models/best.onnx` 是动态 shape，rknn_toolkit2 会拒绝，所以必须重导一份。

## 4. 转 RKNN（FP16，最简，先用这个）

```bash
python3 onnx_to_rknn.py
```

生成 `best_fp.rknn`。把它拷回 RK3588，放到工程的 `models/best.rknn`（重命名一下）。

## 5. （可选）INT8 量化，速度更快

INT8 比 FP16 大约再快一倍，但需要校准图，且类别失真会比 FP16 大一点。

**校准图准备**：
- 收集 **20~100 张**实拍场景图（最好就是现场摄像头同角度拍的），有冰、无冰各占一半
- 任意尺寸都行（脚本会让 rknn-toolkit2 自动 resize），但建议接近 640×640
- 全部丢到 `rk_convert/calib_images/` 目录

然后：
```bash
python3 onnx_to_rknn.py --int8
```

会再生成一个 `best_i8.rknn`。

## 6. 拷回板子

把 `best_fp.rknn`（或 `best_i8.rknn`）拷到板子上：
```bash
# 在板子上
cp best_fp.rknn /home/elf/Desktop/ai-tieluchubing/models/best.rknn
```

然后就可以跑新的 `x4.py` 了（在下一步我会写）。

## 故障排查

| 错误 | 解决 |
|---|---|
| `version mismatch` | toolkit2 版本和板子 librknnrt.so 不一致，必须 2.3.2 |
| `Cannot find op type: Mul` 之类 | ONNX opset 太新，用 `opset=12` 重导（脚本已设置） |
| `load_onnx failed` 后跟 dynamic shape 报错 | 没用脚本重导 ONNX，best.onnx 是动态的 |
| INT8 精度暴跌（一张都检不出） | 校准图选得不好，多放些有目标的图；或先用 FP16 |
