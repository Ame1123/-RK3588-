# RK3588 Railway Ice Detection

RK3588 NPU based railway ice detection project. The final runtime entry is `x4.py`.

AI视觉识别、嵌入式、YOLOv8、Elf2、物联网项目。

## Run

1. Install the required Python packages for the target device, including OpenCV, NumPy, PySerial, Paho MQTT and RKNN Lite runtime.
2. Make sure `models/best.rknn` exists.
3. Configure private credentials through environment variables:

```powershell
$env:MQTT_PASSWORD="your_huaweicloud_mqtt_password"
$env:CAMERA_DEVICE="21"
```

On Linux/RK3588:

```bash
export MQTT_PASSWORD="your_huaweicloud_mqtt_password"
export CAMERA_DEVICE=21
python3 x4.py
```

## Notes

- `x4.py` is the current main program.
- `rk_convert/` contains model conversion helpers.
- Private credentials are intentionally not committed. See `.env.example` for expected variables.

## Device Setup

1. 连接与启动摄像头

串口连接：将 STM32 的 PA9 连接到 Elf2 的 10 号引脚，PA10 连接到 8 号引脚，并接地。

启动摄像头：

```bash
cd Desktop/
source tieluchubing/bin/activate
cd ai-tieluchubing/
sudo python3 x4.py
```

2. 串口调试命令

```bash
sudo apt-get install cutecom
sudo cutecom
```

3. 控制 STM32 舵机与继电器

通过发送 JSON 格式的数据来控制舵机角度或模式：

```json
{"servo1": 90, "servo2": 0, "servo3": 0, "servo4": 0}
```

测试舵机角度：

```json
{"servo1": 0}
{"servo1": 180}
{"servo3": 120}
```

切换模式：

```json
{"auto": 0}
{"auto": 1}
```

启动继电器：

```json
{"switch": 1}
```
