#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
铁路冰雪检测系统 - RK3588 NPU 重构版 (x4.py)

相比 x3.py 的核心变化:
  - 推理: ONNX Runtime CPU  →  RKNNLite × 3 NPU 核 (RKNNPool)
  - 架构: 采集/推理/后处理 三线程完全解耦, 主循环不再被推理阻塞
  - 摄像头: 设备号抽成配置项 CAMERA_DEVICE
  - 业务逻辑(除冰/舵机/MQTT/串口/box): 与 x3.py 完全一致

启动前提:
  models/best.rknn 必须存在 (在 PC 上用 rk_convert/ 转好后拷过来,
  详见 rk_convert/README.md)
"""

import datetime
import os
import queue
import signal
import socket
import struct
import sys
import threading
import time

import json
import serial

os.environ['OPENCV_LOG_LEVEL'] = 'SILENT'
os.environ['OPENCV_VIDEOIO_DEBUG'] = '0'
# RKNN 自己管 NPU 调度, 给 OpenCV/numpy 留两个核就够
os.environ.setdefault('OMP_NUM_THREADS', '2')
os.environ.setdefault('MKL_NUM_THREADS', '2')

import cv2
cv2.setLogLevel(0)
import numpy as np
from paho.mqtt import client as mqtt_client

# 把 rk_runtime 目录加进 sys.path, 不强制要求项目结构
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rk_runtime'))
from rknn_pool import RKNNPool
from yolov8_postprocess import (INPUT_SIZE, decode_yolov8, draw_detections,
                                 letterbox, nms_per_class, scale_back)

# ============= 配置参数 =============
# MQTT配置
MQTT_BROKER = 'f0210a01a6.st1.iotda-device.cn-east-3.myhuaweicloud.com'
MQTT_PORT = 8883
MQTT_TOPIC = "$oc/devices/686c84697d33413cbacc4598_wwb1/sys/properties/report"
MQTT_CLIENT_ID = '686c84697d33413cbacc4598_wwb1_0_0_2025070802'
MQTT_USERNAME = '686c84697d33413cbacc4598_wwb1'
MQTT_PASSWORD = os.environ.get('MQTT_PASSWORD', '')
MQTT_DOWN_TOPIC = "$oc/devices/686c84697d33413cbacc4598_wwb1/sys/messages/down"

# 串口配置
SERIAL_PORT = '/dev/ttyS9'
BAUDRATE = 115200

# RTP配置
RTP_HOST = '120.55.88.126'
RTP_PORT = 5004
RTP_TARGET_FPS = 15           # 上传帧率上限. 推理仍按摄像头实际 FPS 跑, 只是 RTP 节流
RTP_JPEG_QUALITY = 60         # JPG 质量(0-100), 越低越省带宽. x3.py 是 80

# 摄像头配置 - 板子换了以后要重新确认 (ls /dev/video*)
# RK3588 板载 ISP/HDMI-RX 已经占用 video0~video20, USB UVC 通常落在 video21+
CAMERA_DEVICE = int(os.environ.get('CAMERA_DEVICE', 21))
CAMERA_WIDTH = 640   # 摄像头原始采集分辨率, 推理时会 letterbox 到 INPUT_SIZE
CAMERA_HEIGHT = 480

# YOLO/RKNN
RKNN_MODEL_PATH = 'models/best.rknn'
NUM_CLASSES = 2          # 模型有两类: 0=jiebing(冰), 1=wubing(无冰)
CLASS_NAMES = {0: 'jiebing', 1: 'wubing'}
CONF_THRES = 0.25
IOU_THRES = 0.45
NPU_WORKERS = 3          # RK3588 三个 NPU 核, 各分一个 worker

# 结冰检测配置
ICE_DETECTION_CLASS = 'jiebing'
DEICING_COOLDOWN = 10
SERVO2_DEICING_ANGLE = 45
SWITCH_ON_DURATION = 5

# ============= 全局变量 =============
running = True
mqtt_client_instance = None
ser = None
yolo_pool = None         # RKNNPool 实例
rtp_sender = None

current_temp = None
current_humi = None
current_auto = 1
icy_status = 0
switch_status = 0
box_status = 0
last_deicing_time = 0
deicing_lock = threading.Lock()
servo1_auto_rotation = True
deicing_in_progress = False

ice_detection_buffer = []
CONSECUTIVE_CHECKS = 5
ICE_TRIGGER_THRESHOLD = 5
current_servo2_position = 0

# 处理队列 (大小都为 1, 永远只保留最新一帧, 避免堆积)
ice_event_queue = queue.Queue(maxsize=1)
rtp_frame_queue = queue.Queue(maxsize=1)
display_frame_queue = queue.Queue(maxsize=1)  # 本地 imshow 用

# MQTT 节流: 同值最多 1 秒一次, 变化立即上报. 避免 ~50FPS 检测把云端发布限流打满.
MQTT_PUBLISH_MIN_INTERVAL = 1.0
_last_publish = {
    "icy_value": None, "icy_ts": 0.0,
    "switch_value": None, "switch_ts": 0.0,
}


def ice_worker_thread():
    """单线程消费检测结果, 避免每帧起线程造成阻塞堆积"""
    while running:
        try:
            args = ice_event_queue.get(timeout=1)
            handle_ice_detection(*args)
        except queue.Empty:
            continue
        except Exception as e:
            print(f"ice_worker异常: {e}")


def rtp_sender_thread():
    """独立线程做 RTP 编码和发送, 避免阻塞摄像头主循环.
    节流到 RTP_TARGET_FPS — 检测/本地预览仍按真实 FPS, 但上传不会撑爆带宽."""
    min_interval = 1.0 / max(1, RTP_TARGET_FPS)
    last_send = 0.0
    while running:
        try:
            frame = rtp_frame_queue.get(timeout=1)
            now = time.time()
            if now - last_send < min_interval:
                continue  # 丢这一帧, 维持目标 FPS
            last_send = now
            if rtp_sender is not None:
                rtp_sender.send_frame(frame)
        except queue.Empty:
            continue
        except Exception as e:
            print(f"rtp_sender异常: {e}")


# ============= RTP发送器类 =============
class RTPSender:
    def __init__(self, host=RTP_HOST, port=RTP_PORT):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dest_addr = (host, port)
        self.sequence = 0
        self.ssrc = 0x12345678

    def send_frame(self, frame):
        try:
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, RTP_JPEG_QUALITY])
            if not ret:
                return
            header = bytearray(12)
            header[0] = 0x80
            header[1] = 0x60
            struct.pack_into('>H', header, 2, self.sequence % 65536)
            struct.pack_into('>I', header, 4, int(time.time()))
            struct.pack_into('>I', header, 8, self.ssrc)
            max_pkt_size = 60000
            for i in range(0, len(buffer), max_pkt_size):
                chunk = bytes(buffer[i:i + max_pkt_size])
                self.sock.sendto(header + chunk, self.dest_addr)
            self.sequence += 1
        except Exception as e:
            print(f"RTP发送失败: {e}")

    def close(self):
        if self.sock:
            self.sock.close()


# ============= 信号处理 =============
def signal_handler(sig, frame):
    global running, mqtt_client_instance, ser, yolo_pool, rtp_sender
    print('\n程序被中断，正在优雅退出...')
    running = False
    if mqtt_client_instance:
        try:
            mqtt_client_instance.disconnect()
            mqtt_client_instance.loop_stop()
        except Exception:
            pass
    if ser:
        try:
            ser.close()
        except Exception:
            pass
    if yolo_pool:
        try:
            yolo_pool.release()
        except Exception:
            pass
    if rtp_sender:
        rtp_sender.close()
    cv2.destroyAllWindows()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


# ============= 业务逻辑(舵机/MQTT/除冰) - 与 x3.py 保持一致 =============
def control_servo1_rotation(enable_rotation):
    global servo1_auto_rotation
    try:
        if ser is None:
            print("串口未初始化, 无法控制servo1旋转")
            return
        servo1_auto_rotation = enable_rotation
        rotation_data = {"servo1stop": 0 if enable_rotation else 1}
        cjson_str = json.dumps(rotation_data, separators=(',', ':')) + '\n'
        ser.write(cjson_str.encode('utf-8'))
        status = "启用" if enable_rotation else "停止"
        print(f"servo1自动旋转{status}: {rotation_data}")
        if mqtt_client_instance:
            now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            msg = {"services": [{"serviceId": "wwb", "properties": rotation_data, "event_time": now}]}
            json_msg = json.dumps(msg)
            result = tracked_publish(mqtt_client_instance, MQTT_TOPIC, json_msg)
            if result.rc == mqtt_client.MQTT_ERR_SUCCESS:
                print(f"servo1旋转状态云端同步: {msg}")
            else:
                print(f"servo1旋转状态云端同步失败, code: {result.rc}")
    except Exception as e:
        print(f"控制servo1旋转异常: {e}")


def control_servo2_position(angle):
    global current_servo2_position
    try:
        if ser is None:
            print("串口未初始化, 无法控制servo2位置")
            return
        if angle == current_servo2_position:
            return
        servo2_data = {"servo2": angle}
        cjson_str = json.dumps(servo2_data, separators=(',', ':')) + '\n'
        ser.write(cjson_str.encode('utf-8'))
        current_servo2_position = angle
        print(f"servo2位置设置为{angle}度")
        if mqtt_client_instance:
            now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            msg = {"services": [{"serviceId": "wwb", "properties": servo2_data, "event_time": now}]}
            json_msg = json.dumps(msg)
            result = tracked_publish(mqtt_client_instance, MQTT_TOPIC, json_msg)
            if result.rc != mqtt_client.MQTT_ERR_SUCCESS:
                print(f"servo2位置云端同步失败, code: {result.rc}")
    except Exception as e:
        print(f"控制servo2位置异常: {e}")


def publish_icy_status(client, icy_value):
    global icy_status
    try:
        if client is None:
            return
        # 节流: 同值 1 秒内只发一次, 变化立刻发. 避免 50FPS 把云端发布限流打满.
        now_ts = time.time()
        same = (icy_value == _last_publish["icy_value"])
        if same and (now_ts - _last_publish["icy_ts"]) < MQTT_PUBLISH_MIN_INTERVAL:
            icy_status = icy_value
            return
        icy_status = icy_value
        _last_publish["icy_value"] = icy_value
        _last_publish["icy_ts"] = now_ts
        now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        msg = {"services": [{"serviceId": "wwb", "properties": {"icy": icy_value}, "event_time": now}]}
        json_msg = json.dumps(msg)
        result = tracked_publish(client, MQTT_TOPIC, json_msg)
        if result.rc == mqtt_client.MQTT_ERR_SUCCESS:
            print(f"icy状态上报: {icy_value}")
        else:
            print(f"icy状态上报失败, code: {result.rc}")
    except Exception as e:
        print(f"icy状态上报异常: {e}")


def publish_switch_status(client, switch_value):
    global switch_status
    try:
        if client is None:
            return
        now_ts = time.time()
        same = (switch_value == _last_publish["switch_value"])
        if same and (now_ts - _last_publish["switch_ts"]) < MQTT_PUBLISH_MIN_INTERVAL:
            switch_status = switch_value
            return
        switch_status = switch_value
        _last_publish["switch_value"] = switch_value
        _last_publish["switch_ts"] = now_ts
        now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        msg = {"services": [{"serviceId": "wwb", "properties": {"switch": switch_value}, "event_time": now}]}
        json_msg = json.dumps(msg)
        result = tracked_publish(client, MQTT_TOPIC, json_msg)
        if result.rc == mqtt_client.MQTT_ERR_SUCCESS:
            print(f"switch状态上报: {switch_value}")
        else:
            print(f"switch状态上报失败, code: {result.rc}")
    except Exception as e:
        print(f"switch状态上报异常: {e}")


def trigger_deicing():
    global last_deicing_time, deicing_in_progress, current_servo2_position
    try:
        if deicing_in_progress:
            return
        if ser is None:
            print("串口未初始化, 无法执行除冰操作")
            return
        deicing_in_progress = True
        deicing_data = {"switch": 1, "servo2": SERVO2_DEICING_ANGLE}
        cjson_str = json.dumps(deicing_data, separators=(',', ':')) + '\n'
        ser.write(cjson_str.encode('utf-8'))
        current_servo2_position = SERVO2_DEICING_ANGLE
        last_deicing_time = time.time()
        print(f"自动模式触发除冰操作: {deicing_data}")
        publish_switch_status(mqtt_client_instance, 1)
        if mqtt_client_instance:
            now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            msg = {"services": [{"serviceId": "wwb", "properties": {"servo2": SERVO2_DEICING_ANGLE}, "event_time": now}]}
            json_msg = json.dumps(msg)
            tracked_publish(mqtt_client_instance, MQTT_TOPIC, json_msg)

        def close_switch():
            global deicing_in_progress
            try:
                if ser is None:
                    deicing_in_progress = False
                    return
                ser.write((json.dumps({"switch": 0}, separators=(',', ':')) + '\n').encode('utf-8'))
                publish_switch_status(mqtt_client_instance, 0)
                deicing_in_progress = False
                print("除冰操作完成")
            except Exception as e:
                deicing_in_progress = False
                print(f"关闭除冰开关异常: {e}")

        timer = threading.Timer(SWITCH_ON_DURATION, close_switch)
        timer.daemon = True
        timer.start()
    except Exception as e:
        deicing_in_progress = False
        print(f"除冰操作异常: {e}")


def handle_ice_detection(detected_ice, temp, humi, auto_mode):
    """与 x3.py 一致的连续判断+模式分支"""
    global icy_status, servo1_auto_rotation, ice_detection_buffer
    try:
        ice_detection_buffer.append(detected_ice)
        if len(ice_detection_buffer) > CONSECUTIVE_CHECKS:
            ice_detection_buffer.pop(0)
        if len(ice_detection_buffer) < CONSECUTIVE_CHECKS:
            return

        ice_count = sum(ice_detection_buffer)
        mode_name = "手动模式" if auto_mode == 1 else "自动模式"

        if auto_mode == 1:
            if ice_count >= ICE_TRIGGER_THRESHOLD:
                print(f"{mode_name} - 检测到结冰 ({ice_count}/{CONSECUTIVE_CHECKS})")
                publish_icy_status(mqtt_client_instance, 1)
            else:
                publish_icy_status(mqtt_client_instance, 0)
            ice_detection_buffer.clear()
            return

        with deicing_lock:
            if ice_count >= ICE_TRIGGER_THRESHOLD:
                print(f"{mode_name} - 检测到结冰 ({ice_count}/{CONSECUTIVE_CHECKS})")
                if servo1_auto_rotation:
                    control_servo1_rotation(False)
                control_servo2_position(45)
                publish_icy_status(mqtt_client_instance, 1)
                trigger_deicing()
            else:
                if current_servo2_position != 0:
                    control_servo2_position(0)
                    time.sleep(0.1)
                if not servo1_auto_rotation:
                    control_servo1_rotation(True)
                publish_icy_status(mqtt_client_instance, 0)
            ice_detection_buffer.clear()
    except Exception as e:
        print(f"结冰检测处理异常: {e}")


def handle_manual_servo_control(servo_data):
    global current_servo2_position, servo1_auto_rotation, ice_detection_buffer
    try:
        if 'servo1' in servo_data:
            if servo1_auto_rotation:
                servo1_auto_rotation = False
        if 'servo1stop' in servo_data:
            servo1_auto_rotation = not bool(servo_data['servo1stop'])
        if 'servo2' in servo_data:
            current_servo2_position = servo_data['servo2']
        if 'switch' in servo_data:
            publish_switch_status(mqtt_client_instance, servo_data['switch'])
        if ice_detection_buffer:
            ice_detection_buffer.clear()
    except Exception as e:
        print(f"手动模式舵机控制异常: {e}")


# ============= MQTT =============
# 调试用: 统计每秒 publish 调用次数 / 成功率 / 当前连接状态
_mqtt_stats = {"calls": 0, "ok": 0, "fail": 0, "last_print": 0.0}
_mqtt_stats_lock = threading.Lock()


def _record_publish(result_rc: int) -> None:
    with _mqtt_stats_lock:
        _mqtt_stats["calls"] += 1
        if result_rc == 0:
            _mqtt_stats["ok"] += 1
        else:
            _mqtt_stats["fail"] += 1
        now = time.time()
        if now - _mqtt_stats["last_print"] >= 2.0:
            connected = bool(mqtt_client_instance and mqtt_client_instance.is_connected())
            print(f"[mqtt] last 2s: calls={_mqtt_stats['calls']} ok={_mqtt_stats['ok']} "
                  f"fail={_mqtt_stats['fail']} connected={connected}")
            _mqtt_stats["calls"] = 0
            _mqtt_stats["ok"] = 0
            _mqtt_stats["fail"] = 0
            _mqtt_stats["last_print"] = now


def tracked_publish(client, topic, payload):
    """所有 MQTT publish 走这里, 顺手记录调用次数+连接状态."""
    if client is None:
        class _Fake:
            rc = 4
        return _Fake()
    result = client.publish(topic, payload)
    _record_publish(int(result.rc))
    return result


def connect_mqtt():
    def on_connect(client, userdata, flags, reason_code, properties=None):
        if reason_code == 0:
            print("Connected to MQTT Broker!")
        else:
            print(f"Failed to connect, return code {reason_code}")

    def on_disconnect(client, userdata, flags, reason_code, properties=None):
        print("Disconnected from MQTT Broker")

    try:
        client = mqtt_client.Client(client_id=MQTT_CLIENT_ID,
                                    callback_api_version=mqtt_client.CallbackAPIVersion.VERSION2)
    except AttributeError:
        client = mqtt_client.Client(client_id=MQTT_CLIENT_ID)

        def on_connect_legacy(client, userdata, flags, rc):
            print(f"MQTT connect rc={rc}")
        client.on_connect = on_connect_legacy
    else:
        client.on_connect = on_connect
        client.on_disconnect = on_disconnect

    client.username_pw_set(username=MQTT_USERNAME, password=MQTT_PASSWORD)
    client.tls_set()
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
        return client
    except Exception as e:
        print(f"MQTT连接失败: {e}")
        return None


def on_message(client, userdata, msg):
    global current_auto, current_servo2_position, switch_status, box_status, ice_detection_buffer
    try:
        payload = msg.payload.decode('utf-8')
        print(f"收到云端下发: {payload}")
        data = json.loads(payload)

        def extract(d):
            result = {}
            if isinstance(d, dict):
                for k, v in d.items():
                    if k in ['servo1', 'servo2', 'servo3', 'servo4', 'auto', 'switch', 'box']:
                        result[k] = v
                    elif isinstance(v, dict):
                        result.update(extract(v))
                    elif isinstance(v, list):
                        for item in v:
                            result.update(extract(item))
            return result

        sdata = extract(data)
        filtered = {}
        for k in ['servo1', 'servo2', 'servo3', 'servo4']:
            if k in sdata:
                filtered[k] = int(sdata[k])
        if 'auto' in sdata:
            filtered['auto'] = int(sdata['auto'])
            old_auto = current_auto
            current_auto = filtered['auto']
            if old_auto != current_auto:
                mode = "自动模式" if current_auto == 0 else "手动模式"
                print(f"模式切换: {old_auto} -> {current_auto} ({mode})")
                if old_auto == 1 and current_auto == 0:
                    ice_detection_buffer.clear()
        if 'switch' in sdata:
            filtered['switch'] = int(sdata['switch'])
        if 'box' in sdata:
            filtered['box'] = int(sdata['box'])
            box_status = filtered['box']

        if filtered and ser:
            cjson_str = json.dumps(filtered, separators=(',', ':')) + '\n'
            try:
                ser.write(cjson_str.encode('utf-8'))
                print(f"串口下发: {cjson_str.strip()}")
                if 'servo2' in filtered:
                    current_servo2_position = filtered['servo2']
                if current_auto == 1:
                    handle_manual_servo_control(filtered)
                if 'switch' in filtered:
                    switch_status = filtered['switch']
            except Exception as e:
                print(f"串口写入异常: {e}")

            now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            msg_up = {"services": [{"serviceId": "wwb", "properties": filtered, "event_time": now}]}
            tracked_publish(client, MQTT_TOPIC, json.dumps(msg_up))
    except Exception as e:
        print(f"处理下发消息异常: {e}")


# ============= 串口 =============
def open_serial():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        print(f"串口{SERIAL_PORT}已打开")
        return True
    except Exception as e:
        print(f"串口打开失败: {e}")
        ser = None
        return False


def publish_from_serial(client):
    global current_auto, current_servo2_position, switch_status, box_status
    if client is None or ser is None:
        print("MQTT client或串口未初始化")
        return
    while running:
        try:
            line = ser.readline().decode('utf-8').strip()
            if not line:
                continue
            try:
                data = json.loads(line)
            except Exception:
                print(f"串口收到非JSON数据: {line}")
                continue
            now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            msg = None
            if 'temp' in data and 'humi' in data:
                global current_temp, current_humi
                current_temp = data['temp']
                current_humi = data['humi']
                msg = {"services": [{"serviceId": "wwb",
                                     "properties": {"temp": data['temp'], "humi": data['humi']},
                                     "event_time": now}]}
            elif 'su' in data and 'xiang' in data:
                msg = {"services": [{"serviceId": "wwb",
                                     "properties": {"su": data['su'], "xiang": data['xiang']},
                                     "event_time": now}]}
            elif any(k in data for k in ('servo1', 'servo2', 'servo3', 'servo4', 'auto', 'switch', 'box')):
                properties = {}
                for k, v in data.items():
                    if k.startswith('servo'):
                        properties[k] = v
                        if k == 'servo2':
                            current_servo2_position = v
                if 'auto' in data:
                    properties['auto'] = data['auto']
                    current_auto = data['auto']
                if 'switch' in data:
                    properties['switch'] = data['switch']
                    switch_status = data['switch']
                if 'box' in data:
                    properties['box'] = data['box']
                    box_status = data['box']
                if current_auto == 1:
                    handle_manual_servo_control(data)
                msg = {"services": [{"serviceId": "wwb", "properties": properties, "event_time": now}]}

            if msg:
                result = tracked_publish(client, MQTT_TOPIC, json.dumps(msg))
                if result.rc != mqtt_client.MQTT_ERR_SUCCESS:
                    print(f"MQTT发送失败, code: {result.rc}")
        except Exception as e:
            print(f"串口上报异常: {e}")
            time.sleep(1)


# ============= 摄像头采集 (生产者) =============
import subprocess


def _v4l2_warmup(device_idx: int) -> None:
    """
    在 OpenCV 打开摄像头之前, 用 v4l2-ctl 强制配置 UVC 控制项.

    不同 UVC 摄像头对手动曝光的容忍度差别很大: 有的摄像头在
    exposure_auto=1 (manual) 时如果 exposure_absolute 值不合适, 会
    直接不出流(read 返回 False). 所以这里保留自动曝光, 但禁用
    exposure_auto_priority — 后者允许驱动通过降低帧率来延长曝光时间,
    是 USB UVC 在暗光下 FPS 从 30 掉到 15 的常见元凶.
    """
    dev = f"/dev/video{device_idx}"
    cmds = [
        ["v4l2-ctl", "-d", dev, "--set-ctrl=power_line_frequency=0"],   # 关 50Hz 抗闪
        ["v4l2-ctl", "-d", dev, "--set-ctrl=exposure_auto=3"],          # 自动曝光(Aperture Priority)
        ["v4l2-ctl", "-d", dev, "--set-ctrl=exposure_auto_priority=0"], # 关闭"为曝光降帧率"
        ["v4l2-ctl", "-d", dev, "--set-ctrl=backlight_compensation=0"],
        ["v4l2-ctl", "-d", dev,
         f"--set-fmt-video=width={CAMERA_WIDTH},height={CAMERA_HEIGHT},pixelformat=MJPG"],
    ]
    for c in cmds:
        try:
            subprocess.run(c, check=False, capture_output=True, timeout=2)
        except Exception:
            pass


def open_camera():
    _v4l2_warmup(CAMERA_DEVICE)
    # 显式用 V4L2 后端, 避免 OpenCV 默认走 GStreamer/FFmpeg 导致 set() 不生效
    cap = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)
    # 顺序: 先设 FOURCC, 再设分辨率. 反过来 V4L2 会拒收 FOURCC 切换
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 30)
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass
    if not cap.isOpened():
        # 第一次失败: 有些 UVC 设备拒绝 MJPG 切换, 退回默认格式再试一次
        print("[camera] V4L2+MJPG 打开失败, 退回默认后端再试")
        try:
            cap.release()
        except Exception:
            pass
        cap = cv2.VideoCapture(CAMERA_DEVICE)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
    if not cap.isOpened():
        return None
    # 验证: 真去读一帧, 有些 UVC 摄像头 isOpened()==True 但 read() 立刻 False
    ok, _ = cap.read()
    if not ok:
        print("[camera] 打开成功但首帧读取失败, 视为打开失败")
        try:
            cap.release()
        except Exception:
            pass
        return None
    # 把实际生效的参数打印一次, 方便定位带宽问题
    fcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    fcc_str = ''.join(chr((fcc >> (8 * i)) & 0xFF) for i in range(4))
    print(f"[camera] device={CAMERA_DEVICE} "
          f"size={int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} "
          f"fourcc={fcc_str} fps={cap.get(cv2.CAP_PROP_FPS):.1f}")
    return cap


def yolo_preprocess(frame):
    """letterbox 到 INPUT_SIZE×INPUT_SIZE, 返回 NHWC uint8 blob 以及变换参数"""
    padded, scale, pad_x, pad_y = letterbox(frame, INPUT_SIZE)
    # RKNN config 里指定了 mean/std (除以255), 所以这里直接传 uint8 即可
    # 颜色: ultralytics 默认按 BGR 训练? 实际是 RGB, 这里转一下
    rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
    blob = np.expand_dims(rgb, axis=0)
    return blob, (scale, pad_x, pad_y)


def capture_loop():
    """采集 -> 预处理 -> 提交到 NPU pool. 不做任何后处理, 也不画框."""
    global yolo_pool
    cap = open_camera()
    reopen_attempts = 0
    fail_count = 0
    frame_count = 0
    t0 = time.time()

    while running:
        try:
            if cap is None or not cap.isOpened():
                reopen_attempts += 1
                if reopen_attempts == 1 or reopen_attempts % 5 == 0:
                    print(f"摄像头未打开, 尝试重新打开 (第{reopen_attempts}次). 检查: ls /dev/video{CAMERA_DEVICE}")
                try:
                    if cap is not None:
                        cap.release()
                except Exception:
                    pass
                cap = open_camera()
                if cap is None or not cap.isOpened():
                    time.sleep(2)
                    continue
                reopen_attempts = 0

            ret, frame = cap.read()
            if not ret:
                fail_count += 1
                if fail_count >= 3:
                    try:
                        cap.release()
                    except Exception:
                        pass
                    cap = None
                    fail_count = 0
                else:
                    time.sleep(0.05)
                continue
            fail_count = 0

            blob, lb = yolo_preprocess(frame)
            yolo_pool.put(blob, meta=(frame, lb))

            frame_count += 1
            if frame_count % 100 == 0:
                dt = time.time() - t0
                print(f"[capture] {frame_count} frames, ~{frame_count/dt:.1f} FPS, "
                      f"pool backlog={yolo_pool.qsize()}")

        except Exception as e:
            print(f"采集线程异常: {e}")
            time.sleep(0.2)

    if cap is not None:
        try:
            cap.release()
        except Exception:
            pass
    print("采集线程退出")


# ============= 推理结果消费线程 =============
def consume_loop():
    """从 NPU pool 取结果 -> 后处理 -> 画框 -> 丢给 RTP/ice 处理"""
    global yolo_pool
    while running:
        try:
            out = yolo_pool.get(timeout=1.0)
            if out is None:
                continue
            outputs, meta = out
            # capture_loop 里手动做的预处理, 所以 meta 直接就是 (orig_frame, lb)
            orig_frame, (scale, pad_x, pad_y) = meta

            # YOLOv8 后处理: outputs[0] 是 [1, 4+nc, 8400]
            raw = outputs[0] if isinstance(outputs, (list, tuple)) else outputs
            boxes_xywh, scores, class_ids = decode_yolov8(raw, CONF_THRES, NUM_CLASSES)
            keep = nms_per_class(boxes_xywh, scores, class_ids, IOU_THRES)
            if len(keep):
                boxes_xywh = boxes_xywh[keep]
                scores = scores[keep]
                class_ids = class_ids[keep]
            else:
                boxes_xywh = boxes_xywh[:0]
                scores = scores[:0]
                class_ids = class_ids[:0]
            boxes_xyxy = scale_back(boxes_xywh, scale, pad_x, pad_y)

            annotated = draw_detections(orig_frame, boxes_xyxy, scores, class_ids,
                                        CLASS_NAMES, ice_class=ICE_DETECTION_CLASS)

            # 丢给 RTP (队列大小 1, 永远只发最新帧)
            try:
                rtp_frame_queue.put_nowait(annotated)
            except queue.Full:
                try:
                    rtp_frame_queue.get_nowait()
                    rtp_frame_queue.put_nowait(annotated)
                except queue.Empty:
                    pass

            # 同样丢给本地 imshow (主线程的 display_loop 取)
            try:
                display_frame_queue.put_nowait(annotated)
            except queue.Full:
                try:
                    display_frame_queue.get_nowait()
                    display_frame_queue.put_nowait(annotated)
                except queue.Empty:
                    pass

            # 结冰判断 (是否检测到 jiebing 类)
            detected_ice = False
            for cid in class_ids:
                if CLASS_NAMES.get(int(cid)) == ICE_DETECTION_CLASS:
                    detected_ice = True
                    break

            try:
                ice_event_queue.put_nowait((detected_ice, current_temp, current_humi, current_auto))
            except queue.Full:
                pass

        except Exception as e:
            print(f"消费线程异常: {e}")
            time.sleep(0.1)
    print("消费线程退出")


# ============= 本地预览窗口 (必须在主线程运行) =============
WINDOW_NAME = "tieluchubing"


def _have_display():
    """sudo 启动时 DISPLAY/XAUTHORITY 可能丢失, 没有就不开 GUI 窗口."""
    if not os.environ.get('DISPLAY'):
        return False
    # 如果是 sudo 启动, 尝试从 SUDO_USER 的家目录继承 xauth
    if os.environ.get('SUDO_USER') and not os.environ.get('XAUTHORITY'):
        sudo_user = os.environ['SUDO_USER']
        candidate = f"/home/{sudo_user}/.Xauthority"
        if os.path.exists(candidate):
            os.environ['XAUTHORITY'] = candidate
    return True


def display_loop():
    """主线程驱动 cv2.imshow."""
    global running
    if not _have_display():
        print("未检测到图形环境(DISPLAY 未设置), 跳过本地预览窗口")
        print("   提示: 用普通用户跑(`python3 x4.py`)就能弹窗;")
        print("         或 sudo 时加 -E 保留环境: `sudo -E python3 x4.py`")
        while running:
            time.sleep(1)
        return
    try:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
    except cv2.error as e:
        print(f"打开预览窗口失败({e}), 跳过本地预览")
        while running:
            time.sleep(1)
        return

    last_frame = None
    while running:
        try:
            frame = display_frame_queue.get(timeout=0.5)
            last_frame = frame
        except queue.Empty:
            frame = last_frame
        if frame is not None:
            try:
                cv2.imshow(WINDOW_NAME, frame)
            except cv2.error:
                break
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            print("收到键盘退出 (q/ESC)")
            running = False
            break
    cv2.destroyAllWindows()



def main():
    global mqtt_client_instance, yolo_pool, rtp_sender, running
    global current_temp, current_humi, current_auto, icy_status
    global ice_detection_buffer, current_servo2_position, servo1_auto_rotation
    global switch_status, box_status, deicing_in_progress

    print("=== 铁路冰雪检测系统启动 ===")

    # 初始化全局变量
    current_temp = None
    current_humi = None
    current_auto = 1
    icy_status = 0
    switch_status = 0
    box_status = 0
    servo1_auto_rotation = True
    ice_detection_buffer = []
    current_servo2_position = 0
    deicing_in_progress = False

    # 1. 加载 RKNN 模型 -> NPU 池
    print("正在加载YOLO模型...")
    try:
        yolo_pool = RKNNPool(RKNN_MODEL_PATH, num_workers=NPU_WORKERS)
        print("YOLO模型加载成功")
    except FileNotFoundError as e:
        print(str(e))
        print("YOLO模型加载失败，程序退出")
        return
    except Exception as e:
        print(f"YOLO模型加载失败: {e}")
        return

    # 2. 检查摄像头能不能打开 (只是探测, 真正的采集在线程里)
    print("正在初始化摄像头...")
    probe = open_camera()
    if probe is None:
        print("摄像头初始化失败，采集线程会持续重试")
        print(f"      可通过环境变量切换: CAMERA_DEVICE=<n> sudo python3 x4.py")
    else:
        probe.release()
        print("摄像头初始化成功")

    # 3. RTP
    print("正在初始化RTP发送器...")
    try:
        rtp_sender = RTPSender()
        print("RTP发送器初始化成功")
    except Exception as e:
        print(f"RTP发送器初始化失败: {e}")
        return

    # 4. 串口 (可选)
    print("正在打开串口...")
    serial_ok = open_serial()
    if not serial_ok:
        print("串口打开失败，将跳过串口通信功能")

    # 5. MQTT
    print("正在连接MQTT服务器...")
    mqtt_client_instance = connect_mqtt()
    if not mqtt_client_instance:
        print("MQTT连接失败，程序退出")
        return
    mqtt_client_instance.on_message = on_message
    mqtt_client_instance.loop_start()
    mqtt_client_instance.subscribe(MQTT_DOWN_TOPIC)

    # 6. 启动各线程
    print("启动视觉检测线程...")
    threading.Thread(target=capture_loop, daemon=True).start()
    threading.Thread(target=consume_loop, daemon=True).start()
    threading.Thread(target=ice_worker_thread, daemon=True).start()
    threading.Thread(target=rtp_sender_thread, daemon=True).start()
    if serial_ok:
        print("启动串口通信线程...")
        threading.Thread(target=publish_from_serial, args=(mqtt_client_instance,), daemon=True).start()

    print("=== 系统启动完成，按Ctrl+C退出 ===")

    # 主循环负责本地预览窗口 (cv2.imshow 必须在主线程调用)
    try:
        display_loop()
    except KeyboardInterrupt:
        print("收到退出信号")

    # 清理
    print("正在清理资源...")
    running = False
    time.sleep(0.5)
    if mqtt_client_instance:
        mqtt_client_instance.disconnect()
        mqtt_client_instance.loop_stop()
    if ser:
        ser.close()
    if yolo_pool:
        yolo_pool.release()
    if rtp_sender:
        rtp_sender.close()
    print("程序已退出")


if __name__ == '__main__':
    main()
