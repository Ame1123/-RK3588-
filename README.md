# RK3588 Railway Ice Detection

RK3588 NPU based railway ice detection project. The final runtime entry is `x4.py`.

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

