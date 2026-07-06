# RK3588 铁路结冰检测与除冰控制系统

本仓库为比赛作品开源代码仓库，主体任务代码已开源。项目基于 RK3588 NPU、YOLOv8/RKNN、摄像头视觉识别、串口控制和物联网 MQTT 上报，实现铁路结冰检测、状态上报和除冰执行控制。

仓库地址：https://github.com/Ame1123/-RK3588-

## 主体任务代码

- 主程序入口：`x4.py`
- RKNN 推理封装：`rk_runtime/rknn_pool.py`
- YOLOv8 后处理：`rk_runtime/yolov8_postprocess.py`
- 模型转换工具：`rk_convert/`
- RKNN 模型文件：`models/best.rknn`

`x4.py` 是最终运行版本，旧版本和测试脚本保留用于开发记录与对比。

## 主要功能

- 摄像头实时采集铁路/线缆画面
- RK3588 NPU 加速运行 RKNN 模型
- YOLOv8 目标检测与结冰/无冰识别
- 串口发送 JSON 指令控制 STM32、舵机和继电器
- MQTT 上报检测结果并接收云端下发指令
- RTP 视频流上传

## 目录结构

```text
.
├── x4.py                    # 最终运行主程序
├── models/                  # 模型文件，包含 best.rknn
├── rk_runtime/              # RKNN 推理池与 YOLO 后处理代码
├── rk_convert/              # ONNX/RKNN 模型转换脚本
├── static/                  # Web 静态资源
├── templates/               # Web 页面模板
├── test/                    # 硬件测试脚本
├── docs/                    # 比赛资料、设计文件、演示视频说明
└── README.md
```

## 运行环境

- 硬件平台：RK3588 / Elf2
- 外设：摄像头、STM32、舵机、继电器
- Python 依赖：OpenCV、NumPy、PySerial、Paho MQTT、RKNN Lite Runtime
- 模型：`models/best.rknn`

## 运行方式

在 RK3588 设备上进入项目目录，配置私有 MQTT 密码后运行：

```bash
export MQTT_PASSWORD="your_huaweicloud_mqtt_password"
export CAMERA_DEVICE=21
sudo python3 x4.py
```

Windows 调试时可使用：

```powershell
$env:MQTT_PASSWORD="your_huaweicloud_mqtt_password"
$env:CAMERA_DEVICE="21"
python x4.py
```

私有密钥不提交到仓库，请参考 `.env.example` 配置本地环境变量。

## 设备连接与调试

串口连接：将 STM32 的 PA9 连接到 Elf2 的 10 号引脚，PA10 连接到 8 号引脚，并接地。

设备端启动流程示例：

```bash
cd Desktop/
source tieluchubing/bin/activate
cd ai-tieluchubing/
sudo python3 x4.py
```

串口调试工具：

```bash
sudo apt-get install cutecom
sudo cutecom
```

## STM32 控制指令示例

通过串口发送 JSON 格式数据控制舵机角度或模式：

```json
{"servo1": 90, "servo2": 0, "servo3": 0, "servo4": 0}
```

单个舵机测试：

```json
{"servo1": 0}
{"servo1": 180}
{"servo3": 120}
```

切换自动/手动模式：

```json
{"auto": 0}
{"auto": 1}
```

启动继电器：

```json
{"switch": 1}
```

## 比赛资料

- 设计文件：可放入 `docs/design/`
- 演示视频：可放入 `docs/demo/`，如果视频文件较大，建议在该目录的说明文件中填写网盘或公开视频链接
- 项目代码：已在本仓库开源

## 开源协议

本项目采用 MIT License 开源，详见 `LICENSE`。

