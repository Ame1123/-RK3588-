#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
铁路冰雪检测系统 - RK3588 NPU 适配版 (x4.py)

业务逻辑(MQTT/串口/除冰/舵机/状态机)严格照搬 Jetson 端验证过的 x3.py
(/home/elf/Desktop/jetson orin nano源代码/ai-tieluchubing/x3.py).

仅替换三处平台相关代码:
  1. 推理后端: YOLO(.pt) ultralytics  ->  RKNNPool(三核 NPU)
  2. 摄像头:   /dev/video0 默认参数    ->  /dev/video21 + v4l2 预热 + 自动重连
  3. 视觉线程: 单线程串行              ->  采集/推理/RTP 三线程解耦
                                          (Jetson GPU 推理 30ms 串行没事,
                                           RK3588 50FPS 时串行会让 USB 掉线)

启动前提:
  models/best.rknn 存在 (rk_convert/ 转出后拷贝到此路径)
"""

import datetime
import time
import signal
import sys
import json
import serial
import os
# 屏蔽OpenCV的打印输出
os.environ['OPENCV_LOG_LEVEL'] = 'SILENT'
os.environ['OPENCV_VIDEOIO_DEBUG'] = '0'
import cv2
# 设置OpenCV日志级别为静默（4.7+ 有此 API，旧版本靠上面的环境变量）
try:
    cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
except AttributeError:
    pass
import socket
import struct
import subprocess
import numpy as np
import queue
import threading
from paho.mqtt import client as mqtt_client

# RK3588 NPU 推理池 + YOLOv8 后处理 (放在 rk_runtime/ 子目录)
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

# 串口配置 (RK3588: /dev/ttyS9; Jetson: /dev/ttyTHS1)
SERIAL_PORT = '/dev/ttyS9'
BAUDRATE = 115200

# RTP配置
RTP_HOST = '120.55.88.126'
RTP_PORT = 5004

# 摄像头配置
# RK3588 平台: 板载 ISP/HDMI-RX 占用 /dev/video0~20, USB UVC 落在 21+
CAMERA_DEVICE = int(os.environ.get('CAMERA_DEVICE', 21))
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# YOLO/RKNN
RKNN_MODEL_PATH = 'models/best.rknn'
NUM_CLASSES = 2
CLASS_NAMES = {0: 'jiebing', 1: 'wubing'}
CONF_THRES = 0.25
IOU_THRES = 0.45
NPU_WORKERS = 3  # RK3588 三个 NPU 核, 各分一个 worker

# 结冰检测配置
ICE_DETECTION_CLASS = 'jiebing'  # YOLO模型中结冰类别名称
DEICING_COOLDOWN = 10  # 除冰操作冷却时间（秒），防止频繁触发
SERVO2_DEICING_ANGLE = 45  # 除冰时舵机2的角度
SWITCH_ON_DURATION = 5  # 除冰开关打开持续时间（秒）

# ============= 全局变量 =============
running = True
mqtt_client_instance = None
ser = None
camera_cap = None
yolo_pool = None  # RKNNPool 实例 (替换 Jetson 端的 yolo_model)
rtp_sender = None

# 结冰检测相关全局变量
current_temp = None  # 当前温度
current_humi = None  # 当前湿度
current_auto = 1  # 当前自动模式状态，默认手动模式（与单片机保持一致）
icy_status = 0  # 结冰状态：0=未结冰，1=结冰
switch_status = 0  # 继电器状态：0=关闭，1=打开
box_status = 0     # box状态：0=初始状态，1=动作状态
last_deicing_time = 0  # 上次除冰时间戳
deicing_lock = threading.Lock()  # 除冰操作锁
servo1_auto_rotation = True  # servo1自动旋转状态：True=旋转，False=停止
deicing_in_progress = False  # 除冰操作进行中标志

# 连续判断相关全局变量
ice_detection_buffer = []  # 检测结果缓冲区
CONSECUTIVE_CHECKS = 5  # 连续判断次数
current_servo2_position = 0  # 当前servo2位置：0=0度，45=45度

# RK3588 平台特性: 50FPS 时如果业务逻辑/RTP 同步执行会让 V4L2 缓冲堆积
# 进而导致 USB 摄像头掉线. 所以把"非推理"工作放到独立线程消费.
# 队列大小 1: 满了丢旧帧, 永远只处理最新一帧.
ice_event_queue = queue.Queue(maxsize=1)
rtp_frame_queue = queue.Queue(maxsize=1)
display_frame_queue = queue.Queue(maxsize=1)  # 本地 imshow

# icy 上报节流: 华为云 IoT 设备发布限流大约 5 msg/s, 超了会被踢连接(reason_code=Unspecified).
# Jetson 上推理慢, icy 上报天然稀疏不会触发; RK3588 50FPS 时每秒上报多次必中.
# 策略: 状态变化立即发, 没变化最多 1 秒发一次保活.
ICY_PUBLISH_MIN_INTERVAL = 1.0
_last_icy_report = {"value": None, "ts": 0.0}

# 每帧 print 节流: 50FPS 下 stdout 写阻塞会持有 GIL, 阻塞采集线程导致摄像头掉线.
# "当前检测..." 这种每帧日志限到 1Hz, 决策日志(连续5次/按无冰处理)保留.
_last_perframe_log = {"mode": None, "ts": 0.0}
_last_consume_log = {"ts": 0.0}
# 决策日志(连续判断结果/按无冰处理): 状态不变限 1Hz, 变化立即打.
_last_decision_log = {"state": None, "ts": 0.0}


def ice_worker_thread():
    """RK3588 适配: 把 handle_ice_detection 放到独立线程
    避免它里面的串口/MQTT 同步调用阻塞采集主循环."""
    while running:
        try:
            args = ice_event_queue.get(timeout=1)
            handle_ice_detection(*args)
        except queue.Empty:
            continue
        except Exception as e:
            print(f"ice_worker异常: {e}")


def rtp_sender_thread():
    """RK3588 适配: 独立线程做 RTP 编码+UDP, 不挤占采集线程."""
    while running:
        try:
            frame = rtp_frame_queue.get(timeout=1)
            if rtp_sender is not None:
                rtp_sender.send_frame(frame)
        except queue.Empty:
            continue
        except Exception as e:
            print(f"rtp_sender异常: {e}")


# ============= RTP发送器类 (照搬 Jetson x3.py) =============
class RTPSender:
    def __init__(self, host=RTP_HOST, port=RTP_PORT):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dest_addr = (host, port)
        self.sequence = 0
        self.ssrc = 0x12345678

    def send_frame(self, frame):
        try:
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ret:
                return

            header = bytearray(12)
            header[0] = 0x80  # RTP版本和标记位
            header[1] = 0x60  # 负载类型
            struct.pack_into('>H', header, 2, self.sequence % 65536)  # 序列号
            struct.pack_into('>I', header, 4, int(time.time()))  # 时间戳
            struct.pack_into('>I', header, 8, self.ssrc)  # 同步源标识

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


# ============= 信号处理 (照搬 Jetson x3.py) =============
def signal_handler(sig, frame):
    """处理Ctrl+C信号"""
    global running, mqtt_client_instance, ser, camera_cap, rtp_sender, yolo_pool
    print('\n程序被中断，正在优雅退出...')
    running = False

    # 关闭MQTT连接
    if mqtt_client_instance:
        mqtt_client_instance.disconnect()
        mqtt_client_instance.loop_stop()

    # 关闭串口
    if ser:
        ser.close()

    # 关闭摄像头
    if camera_cap:
        camera_cap.release()

    # 关闭 RKNN 推理池 (替换 Jetson 的 yolo_model)
    if yolo_pool:
        try:
            yolo_pool.release()
        except Exception:
            pass

    # 关闭RTP发送器
    if rtp_sender:
        rtp_sender.close()

    cv2.destroyAllWindows()
    sys.exit(0)


# 注册信号处理器
signal.signal(signal.SIGINT, signal_handler)


# ============= 结冰检测相关函数 (照搬 Jetson x3.py) =============
def control_servo1_rotation(enable_rotation):
    """
    控制servo1的自动旋转

    Args:
        enable_rotation: True=启用自动旋转，False=停止自动旋转
    """
    global servo1_auto_rotation

    try:
        if ser is None:
            print("串口未初始化，无法控制servo1旋转")
            return

        # 更新全局状态
        servo1_auto_rotation = enable_rotation

        # 发送旋转控制指令 (servo1stop: 0=旋转, 1=停止)
        rotation_data = {
            "servo1stop": 0 if enable_rotation else 1
        }

        cjson_str = json.dumps(rotation_data, separators=(',', ':')) + '\n'
        ser.write(cjson_str.encode('utf-8'))

        status = "启用" if enable_rotation else "停止"
        print(f"servo1自动旋转{status}: {rotation_data}")

        # 同步上报到云端
        if mqtt_client_instance:
            now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            msg = {
                "services": [
                    {
                        "serviceId": "wwb",
                        "properties": rotation_data,
                        "event_time": now
                    }
                ]
            }
            json_msg = json.dumps(msg)
            result = mqtt_client_instance.publish(MQTT_TOPIC, json_msg)
            if result.rc == mqtt_client.MQTT_ERR_SUCCESS:
                print(f"servo1旋转状态云端同步: {msg}")
            else:
                print(f"servo1旋转状态云端同步失败, code: {result.rc}")

    except Exception as e:
        print(f"控制servo1旋转异常: {e}")


def reset_servo2_position():
    """将servo2重置到0度位置"""
    try:
        if ser is None:
            print("串口未初始化，无法重置servo2位置")
            return

        reset_data = {"servo2": 0}
        cjson_str = json.dumps(reset_data, separators=(',', ':')) + '\n'
        ser.write(cjson_str.encode('utf-8'))

        print(f"servo2重置到0度: {reset_data}")

        if mqtt_client_instance:
            now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            msg = {"services": [{"serviceId": "wwb", "properties": reset_data, "event_time": now}]}
            json_msg = json.dumps(msg)
            result = mqtt_client_instance.publish(MQTT_TOPIC, json_msg)
            if result.rc == mqtt_client.MQTT_ERR_SUCCESS:
                print(f"servo2重置状态云端同步: {msg}")
            else:
                print(f"servo2重置状态云端同步失败, code: {result.rc}")

    except Exception as e:
        print(f"重置servo2位置异常: {e}")


def control_servo2_position(angle):
    """
    控制servo2的位置

    Args:
        angle: 舵机角度（0或45）
    """
    global current_servo2_position

    try:
        if ser is None:
            print("串口未初始化，无法控制servo2位置")
            return

        print(f"control_servo2_position调用: 目标角度={angle}, 当前位置={current_servo2_position}")

        # 如果目标角度与当前位置相同，不需要重复发送
        if angle == current_servo2_position:
            print(f"servo2位置已经是{angle}度，无需重复设置")
            return

        servo2_data = {"servo2": angle}
        cjson_str = json.dumps(servo2_data, separators=(',', ':')) + '\n'
        ser.write(cjson_str.encode('utf-8'))

        # 更新当前位置
        current_servo2_position = angle

        print(f"servo2位置设置为{angle}度: {servo2_data}")
        print(f"current_servo2_position更新为: {current_servo2_position}")

        if mqtt_client_instance:
            now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            msg = {"services": [{"serviceId": "wwb", "properties": servo2_data, "event_time": now}]}
            json_msg = json.dumps(msg)
            result = mqtt_client_instance.publish(MQTT_TOPIC, json_msg)
            if result.rc == mqtt_client.MQTT_ERR_SUCCESS:
                print(f"servo2位置云端同步: {msg}")
            else:
                print(f"servo2位置云端同步失败, code: {result.rc}")

    except Exception as e:
        print(f"控制servo2位置异常: {e}")


def publish_icy_status(client, icy_value):
    """上报icy状态到云端.

    RK3588 适配: 加节流避免 50FPS 把华为云 IoT 设备发布限流(~5msg/s)打满.
      - 状态变化(0->1 或 1->0): 立即发, 保证响应性
      - 状态不变: 距离上次同值上报至少 1 秒才发一次, 当心跳保活
    """
    global icy_status

    try:
        if client is None:
            return

        icy_status = icy_value

        now_ts = time.time()
        same = (icy_value == _last_icy_report["value"])
        if same and (now_ts - _last_icy_report["ts"]) < ICY_PUBLISH_MIN_INTERVAL:
            return  # 限流: 跳过这次重复上报
        _last_icy_report["value"] = icy_value
        _last_icy_report["ts"] = now_ts

        now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        msg = {"services": [{"serviceId": "wwb", "properties": {"icy": icy_value}, "event_time": now}]}

        json_msg = json.dumps(msg)
        result = client.publish(MQTT_TOPIC, json_msg)

        if result.rc == mqtt_client.MQTT_ERR_SUCCESS:
            print(f"icy状态上报: {icy_value}")
        else:
            print(f"icy状态上报失败, code: {result.rc}")
    except Exception as e:
        print(f"icy状态上报异常: {e}")


def publish_switch_status(client, switch_value):
    """上报switch状态到云端"""
    global switch_status

    try:
        if client is None:
            return

        switch_status = switch_value

        now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        msg = {"services": [{"serviceId": "wwb", "properties": {"switch": switch_value}, "event_time": now}]}

        json_msg = json.dumps(msg)
        result = client.publish(MQTT_TOPIC, json_msg)

        if result.rc == mqtt_client.MQTT_ERR_SUCCESS:
            print(f"switch状态上报: {switch_value}")
        else:
            print(f"switch状态上报失败, code: {result.rc}")
    except Exception as e:
        print(f"switch状态上报异常: {e}")


def trigger_deicing():
    """触发除冰操作"""
    global last_deicing_time, deicing_in_progress

    try:
        current_time = time.time()

        if deicing_in_progress:
            print("除冰操作已在进行中，跳过")
            return

        if ser is None:
            print("串口未初始化，无法执行除冰操作")
            return

        deicing_in_progress = True

        deicing_data = {"switch": 1, "servo2": SERVO2_DEICING_ANGLE}
        cjson_str = json.dumps(deicing_data, separators=(',', ':')) + '\n'
        ser.write(cjson_str.encode('utf-8'))

        global current_servo2_position
        current_servo2_position = SERVO2_DEICING_ANGLE

        last_deicing_time = current_time

        print(f"自动模式触发除冰操作: {deicing_data}")
        print(f"同步更新current_servo2_position: {current_servo2_position}")

        publish_switch_status(mqtt_client_instance, 1)
        print("自动除冰：继电器开启，icy状态已由检测逻辑上报")

        if mqtt_client_instance:
            now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            msg = {"services": [{"serviceId": "wwb",
                                 "properties": {"servo2": SERVO2_DEICING_ANGLE},
                                 "event_time": now}]}
            json_msg = json.dumps(msg)
            result = mqtt_client_instance.publish(MQTT_TOPIC, json_msg)
            if result.rc == mqtt_client.MQTT_ERR_SUCCESS:
                print(f"servo2除冰位置云端同步: {msg}")
            else:
                print(f"servo2除冰位置云端同步失败, code: {result.rc}")

        def close_switch():
            try:
                global deicing_in_progress

                if ser is None:
                    print("串口未初始化，无法关闭除冰开关")
                    deicing_in_progress = False
                    return

                close_data = {"switch": 0}
                cjson_str = json.dumps(close_data, separators=(',', ':')) + '\n'
                ser.write(cjson_str.encode('utf-8'))

                print(f"除冰开关自动关闭: {close_data}")

                publish_switch_status(mqtt_client_instance, 0)

                deicing_in_progress = False

                print("除冰操作完成，icy状态由检测结果决定")

            except Exception as e:
                deicing_in_progress = False
                print(f"关闭除冰开关异常: {e}")

        timer = threading.Timer(SWITCH_ON_DURATION, close_switch)
        timer.daemon = True
        timer.start()
        print(f"除冰开关将在 {SWITCH_ON_DURATION} 秒后自动关闭")

    except Exception as e:
        deicing_in_progress = False
        print(f"除冰操作异常: {e}")


def handle_ice_detection(detected_ice, temp, humi, auto_mode):
    """处理结冰检测逻辑 - 连续判断版本.

    RK3588 适配: 50FPS 下 print 频率太高会反压 GIL 导致摄像头掉线.
    保留所有"决策行"打印 (连续判断结果/按无冰处理/检测到结冰), 但把
    "当前检测.../连续判断中..." 这两条每帧都触发的日志降到 1Hz.
    业务语义不变.
    """
    global icy_status, servo1_auto_rotation, ice_detection_buffer

    try:
        ice_detection_buffer.append(detected_ice)

        if len(ice_detection_buffer) > CONSECUTIVE_CHECKS:
            ice_detection_buffer.pop(0)

        mode_name = "手动模式" if auto_mode == 1 else "自动模式"
        # 节流: 同模式下 1 秒内只打一次每帧日志
        now_ts = time.time()
        if (mode_name != _last_perframe_log["mode"]
                or now_ts - _last_perframe_log["ts"] >= 1.0):
            print(f"{mode_name} - 当前检测: {'结冰' if detected_ice else '无冰'}, 缓冲区: {ice_detection_buffer}")
            _last_perframe_log["mode"] = mode_name
            _last_perframe_log["ts"] = now_ts

        if len(ice_detection_buffer) < CONSECUTIVE_CHECKS:
            # 不打"连续判断中..." 它本来就只是进度提示, 1 秒打一次的"当前检测"已足够说明
            return

        ice_count = sum(ice_detection_buffer)
        no_ice_count = len(ice_detection_buffer) - ice_count

        # 决策状态: ("auto"/"manual", "ice"/"no_ice"). 不变 1Hz 限流, 变化立即打全部.
        cur_state = ("manual" if auto_mode == 1 else "auto",
                     "ice" if ice_count == CONSECUTIVE_CHECKS else "no_ice")
        now_ts2 = time.time()
        state_changed = (cur_state != _last_decision_log["state"])
        should_log = state_changed or (now_ts2 - _last_decision_log["ts"] >= 1.0)
        if should_log:
            _last_decision_log["state"] = cur_state
            _last_decision_log["ts"] = now_ts2

        if should_log:
            print(f"{mode_name} - 连续判断结果: 结冰={ice_count}次, 无冰={no_ice_count}次")

        if auto_mode == 1:
            if ice_count == CONSECUTIVE_CHECKS:
                if should_log:
                    print("手动模式 - 连续5次检测到结冰")
                publish_icy_status(mqtt_client_instance, 1)
            else:
                if should_log:
                    print(f"手动模式 - 未连续检测到结冰，按无冰处理")
                publish_icy_status(mqtt_client_instance, 0)

            ice_detection_buffer.clear()
            return

        with deicing_lock:
            if should_log:
                print(f"自动模式 - servo1旋转状态: {'是' if servo1_auto_rotation else '否'}")

            if ice_count == CONSECUTIVE_CHECKS:
                if should_log:
                    print("自动模式 - 连续5次检测到结冰")

                if servo1_auto_rotation:
                    print("停止servo1自动旋转")
                    control_servo1_rotation(False)

                print("servo2旋转至45度")
                control_servo2_position(45)

                publish_icy_status(mqtt_client_instance, 1)

                print("自动模式下启动除冰程序")
                trigger_deicing()

            else:
                if should_log:
                    print(f"自动模式 - 未连续检测到结冰，按无冰处理")

                if current_servo2_position != 0:
                    print("servo2旋转至0度")
                    control_servo2_position(0)
                    time.sleep(0.1)

                if not servo1_auto_rotation:
                    print("开始servo1自动旋转")
                    control_servo1_rotation(True)

                publish_icy_status(mqtt_client_instance, 0)

            ice_detection_buffer.clear()

    except Exception as e:
        print(f"结冰检测处理异常: {e}")


def handle_manual_servo_control(servo_data):
    """处理手动模式下的舵机控制"""
    global current_servo2_position, servo1_auto_rotation

    try:
        print(f"手动模式舵机控制: {servo_data}")

        if 'servo1' in servo_data:
            if servo1_auto_rotation:
                servo1_auto_rotation = False
                print("手动模式下停止servo1自动旋转")

        if 'servo1stop' in servo_data:
            stop_rotation = bool(servo_data['servo1stop'])
            servo1_auto_rotation = not stop_rotation
            print(f"手动模式下设置servo1旋转状态: {'停止' if stop_rotation else '旋转'}")

        if 'servo2' in servo_data:
            target_angle = servo_data['servo2']
            current_servo2_position = target_angle
            print(f"手动模式下设置servo2位置: {target_angle}度")

        if 'switch' in servo_data:
            switch_value = servo_data['switch']
            publish_switch_status(mqtt_client_instance, switch_value)
            if switch_value == 1:
                print("手动模式下开启继电器")
            else:
                print("手动模式下关闭继电器")

        global ice_detection_buffer
        if ice_detection_buffer:
            ice_detection_buffer.clear()
            print("手动模式下清空检测缓冲区")

    except Exception as e:
        print(f"手动模式舵机控制异常: {e}")


# ============= MQTT相关函数 (照搬 Jetson x3.py) =============
def connect_mqtt():
    """连接MQTT服务器"""
    def on_connect(client, userdata, flags, reason_code, properties=None):
        if reason_code == 0:
            print("Connected to MQTT Broker!")
        else:
            print(f"Failed to connect, return code {reason_code}")

    def on_disconnect(client, userdata, flags, reason_code, properties=None):
        # rc 含义: 0=主动断, 4=被 broker 踢, 7=keepalive 超时, 16=read 错误
        print(f"Disconnected from MQTT Broker (reason_code={reason_code})")

    def on_publish(client, userdata, mid, reason_code=None, properties=None):
        print(f"Message {mid} published successfully")

    try:
        client = mqtt_client.Client(
            client_id=MQTT_CLIENT_ID,
            callback_api_version=mqtt_client.CallbackAPIVersion.VERSION2
        )
    except AttributeError:
        print("Using legacy MQTT client API")
        client = mqtt_client.Client(client_id=MQTT_CLIENT_ID)
        def on_connect_legacy(client, userdata, flags, rc):
            if rc == 0:
                print("Connected to MQTT Broker!")
            else:
                print(f"Failed to connect, return code {rc}")
        def on_disconnect_legacy(client, userdata, rc):
            print(f"Disconnected from MQTT Broker (rc={rc})")
        def on_publish_legacy(client, userdata, mid):
            print(f"Message {mid} published successfully")
        client.on_connect = on_connect_legacy
        client.on_disconnect = on_disconnect_legacy
        client.on_publish = on_publish_legacy
    else:
        client.on_connect = on_connect
        client.on_disconnect = on_disconnect
        client.on_publish = on_publish

    client.username_pw_set(username=MQTT_USERNAME, password=MQTT_PASSWORD)
    client.tls_set()  # 启用TLS加密
    # 自动重连: 断开后从 1s 起重试, 最长间隔 30s. paho 默认是关闭的, 断了就不连.
    try:
        client.reconnect_delay_set(min_delay=1, max_delay=30)
    except Exception:
        pass

    try:
        client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
        return client
    except Exception as e:
        print(f"MQTT连接失败: {e}")
        return None


def on_message(client, userdata, msg):
    """处理MQTT下发消息"""
    try:
        payload = msg.payload.decode('utf-8')
        print(f"收到云端下发: {payload}")
        data = json.loads(payload)

        def extract_servo_auto_switch(d):
            result = {}
            if isinstance(d, dict):
                for k, v in d.items():
                    if k in ['servo1', 'servo2', 'servo3', 'servo4', 'auto', 'switch', 'box']:
                        result[k] = v
                    elif isinstance(v, dict):
                        result.update(extract_servo_auto_switch(v))
                    elif isinstance(v, list):
                        for item in v:
                            result.update(extract_servo_auto_switch(item))
            return result

        servo_auto_switch_data = extract_servo_auto_switch(data)

        filtered_data = {}
        for k in ['servo1', 'servo2', 'servo3', 'servo4']:
            if k in servo_auto_switch_data:
                filtered_data[k] = int(servo_auto_switch_data[k])
        if 'auto' in servo_auto_switch_data:
            filtered_data['auto'] = int(servo_auto_switch_data['auto'])
            global current_auto
            old_auto = current_auto
            current_auto = filtered_data['auto']

            if old_auto != current_auto:
                mode_name = "自动模式" if current_auto == 0 else "手动模式"
                print(f"模式切换: {old_auto} -> {current_auto} ({mode_name})")

                if old_auto == 1 and current_auto == 0:
                    global ice_detection_buffer
                    ice_detection_buffer.clear()
                    print("切换到自动模式，清空检测缓冲区")
        if 'switch' in servo_auto_switch_data:
            filtered_data['switch'] = int(servo_auto_switch_data['switch'])
        if 'box' in servo_auto_switch_data:
            filtered_data['box'] = int(servo_auto_switch_data['box'])
            global box_status
            box_status = filtered_data['box']

        if filtered_data and ser:
            cjson_str = json.dumps(filtered_data, separators=(',', ':')) + '\n'
            try:
                ser.write(cjson_str.encode('utf-8'))
                print(f"串口下发: {cjson_str.strip()}")

                if 'servo2' in filtered_data:
                    global current_servo2_position
                    current_servo2_position = filtered_data['servo2']
                    print(f"MQTT下发同步更新current_servo2_position: {current_servo2_position}")

                if current_auto == 1:
                    handle_manual_servo_control(filtered_data)
                if 'switch' in filtered_data:
                    global switch_status
                    switch_status = filtered_data['switch']

            except Exception as e:
                print(f"串口写入异常: {e}")

            now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            msg = {"services": [{"serviceId": "wwb", "properties": filtered_data, "event_time": now}]}
            json_msg = json.dumps(msg)
            result = client.publish(MQTT_TOPIC, json_msg)
            if result.rc == mqtt_client.MQTT_ERR_SUCCESS:
                print(f"云端同步下发: {msg}")
            else:
                print(f"云端同步下发失败, code: {result.rc}")
    except Exception as e:
        print(f"处理下发消息异常: {e}")


# ============= 串口相关函数 (照搬 Jetson x3.py) =============
def open_serial():
    """打开串口"""
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
    """从串口读取数据并发布到MQTT"""
    global current_auto  # 防止 current_auto = data['auto'] 创建局部变量

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
            elif ('servo1' in data or 'servo2' in data or 'servo3' in data or 'servo4' in data
                  or 'auto' in data or 'switch' in data or 'box' in data):
                properties = {}
                for k, v in data.items():
                    if k.startswith('servo'):
                        properties[k] = v
                        if k == 'servo2':
                            global current_servo2_position
                            current_servo2_position = v
                            print(f"串口反馈同步更新current_servo2_position: {current_servo2_position}")
                if 'auto' in data:
                    properties['auto'] = data['auto']
                    current_auto = data['auto']
                if 'switch' in data:
                    properties['switch'] = data['switch']
                    global switch_status
                    switch_status = data['switch']
                if 'box' in data:
                    properties['box'] = data['box']
                    global box_status
                    box_status = data['box']

                if current_auto == 1:
                    handle_manual_servo_control(data)
                msg = {"services": [{"serviceId": "wwb", "properties": properties, "event_time": now}]}

            if msg:
                json_msg = json.dumps(msg)
                result = client.publish(MQTT_TOPIC, json_msg)
                if result.rc == mqtt_client.MQTT_ERR_SUCCESS:
                    print(f"串口上报: {msg}")
                else:
                    print(f"MQTT发送失败, code: {result.rc}")

        except Exception as e:
            print(f"串口上报异常: {e}")
            time.sleep(1)


# ============= [RK3588 适配] YOLO/RKNN 推理 =============
# 替换 Jetson 端的 load_yolo_model() / yolo_model(frame)
def load_yolo_pool(model_path):
    """加载 RKNN 模型 -> 三核 NPU 推理池.
    用 RKNNPool 替换 Jetson 端的 YOLO(.pt). 业务侧仍按"输入帧 -> 检测框"语义使用."""
    if not os.path.exists(model_path):
        print(f"模型文件 {model_path} 不存在")
        return None
    try:
        pool = RKNNPool(model_path, num_workers=NPU_WORKERS)
        print("RKNN模型加载成功")
        return pool
    except Exception as e:
        print(f"RKNN模型加载失败: {e}")
        return None


def yolo_preprocess(frame):
    """letterbox 到 INPUT_SIZE×INPUT_SIZE + BGR2RGB, 返回 NHWC uint8 blob + 还原参数"""
    padded, scale, pad_x, pad_y = letterbox(frame, INPUT_SIZE)
    rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
    blob = np.expand_dims(rgb, axis=0)
    return blob, (scale, pad_x, pad_y)


# ============= [RK3588 适配] 摄像头 =============
def _v4l2_warmup(device_idx):
    """OpenCV 通过 V4L2 设 fourcc 在很多 UVC 摄像头上不可靠, 启动前用 v4l2-ctl 直接喊驱动."""
    dev = f"/dev/video{device_idx}"
    cmds = [
        ["v4l2-ctl", "-d", dev, "--set-ctrl=power_line_frequency=0"],
        ["v4l2-ctl", "-d", dev, "--set-ctrl=exposure_auto=3"],
        ["v4l2-ctl", "-d", dev, "--set-ctrl=exposure_auto_priority=0"],
        ["v4l2-ctl", "-d", dev, "--set-ctrl=backlight_compensation=0"],
        ["v4l2-ctl", "-d", dev,
         f"--set-fmt-video=width={CAMERA_WIDTH},height={CAMERA_HEIGHT},pixelformat=MJPG"],
    ]
    for c in cmds:
        try:
            subprocess.run(c, check=False, capture_output=True, timeout=2)
        except Exception:
            pass


def init_camera():
    """初始化摄像头. RK3588 平台: 默认设备号 21, 使用 V4L2+MJPG."""
    global camera_cap
    try:
        _v4l2_warmup(CAMERA_DEVICE)
        camera_cap = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)
        camera_cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        camera_cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        camera_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        # 关键: 把 V4L2 缓冲区设为 1, 避免推理慢于摄像头时缓冲堆积导致 USB disconnect
        try:
            camera_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        if not camera_cap.isOpened():
            print("摄像头初始化失败")
            return False
        # 验证: 真去读一帧, 有些 UVC 摄像头 isOpened()=True 但 read() 立刻 False
        ok, _ = camera_cap.read()
        if not ok:
            print("摄像头打开成功但首帧读取失败")
            camera_cap.release()
            return False
        print("摄像头初始化成功")
        return True
    except Exception as e:
        print(f"摄像头初始化失败: {e}")
        return False


# ============= [RK3588 适配] 视觉检测: 采集/推理/RTP 三线程 =============
# 这里跟 Jetson x3.py 的语义对照:
#   Jetson:  while: read -> infer -> plot -> rtp -> handle_ice
#   RK3588:  capture(read+letterbox+put) | consume(get+postprocess+draw) | rtp_send | ice_worker
# 业务可见的"语义"不变 (一帧图 -> 检测到 ice/未检测到 -> 进入 handle_ice_detection),
# 只是分线程执行, 不再串行阻塞采集.

def capture_loop():
    """采集线程: read -> letterbox -> 提交 NPU 池. 不做任何后处理/绘制."""
    global camera_cap, yolo_pool, running

    if not yolo_pool or not camera_cap:
        print("视觉检测组件未初始化")
        return

    print("视觉检测线程启动")

    frame_count = 0
    fail_count = 0
    reopen_attempts = 0
    t0 = time.time()
    last_heartbeat = time.time()
    last_frame_ts = time.time()

    while running:
        try:
            # 心跳: 5 秒没读到一帧就说自己快挂了, 方便定位摄像头掉线时机
            now_ts = time.time()
            if now_ts - last_heartbeat >= 5.0:
                stale = now_ts - last_frame_ts
                print(f"[capture-heartbeat] 距上一帧 {stale:.1f}s, "
                      f"cap={'open' if camera_cap and camera_cap.isOpened() else 'closed'}, "
                      f"frames={frame_count}, fail={fail_count}, reopen={reopen_attempts}")
                last_heartbeat = now_ts

            if camera_cap is None or not camera_cap.isOpened():
                reopen_attempts += 1
                if reopen_attempts == 1 or reopen_attempts % 5 == 0:
                    print(f"摄像头未打开，尝试重新打开（第{reopen_attempts}次）。"
                          f"请检查USB连接：ls /dev/video{CAMERA_DEVICE}")
                try:
                    if camera_cap is not None:
                        camera_cap.release()
                except Exception:
                    pass
                if init_camera():
                    print("摄像头重新打开成功")
                    reopen_attempts = 0
                else:
                    time.sleep(2)
                continue

            ret, frame = camera_cap.read()
            if not ret:
                fail_count += 1
                print(f"无法读取摄像头帧 (失败 {fail_count} 次)")
                if fail_count >= 3:
                    try:
                        camera_cap.release()
                    except Exception:
                        pass
                    camera_cap = None
                    fail_count = 0
                else:
                    time.sleep(0.1)
                continue
            fail_count = 0
            last_frame_ts = time.time()

            blob, lb = yolo_preprocess(frame)
            yolo_pool.put(blob, meta=(frame, lb))

            frame_count += 1
            if frame_count % 100 == 0:
                dt = time.time() - t0
                print(f"[capture] {frame_count} frames, ~{frame_count/dt:.1f} FPS, "
                      f"pool backlog={yolo_pool.qsize()}")

        except Exception as e:
            # 详细打印异常类型/位置, 之前裸 "视觉检测异常" 信息量太少
            import traceback
            print(f"视觉检测异常: {type(e).__name__}: {e}")
            traceback.print_exc()
            time.sleep(1)

    print("视觉检测线程退出")


def consume_loop():
    """消费线程: 从 NPU 取结果 -> 后处理+画框 -> 入 RTP 队列 + 入 ice 事件队列.
    把 Jetson 端 vision_detection_thread 后半段 (画框/RTP/ice 判断) 搬到这里."""
    global yolo_pool, running

    while running:
        try:
            out = yolo_pool.get(timeout=1.0)
            if out is None:
                continue
            outputs, meta = out
            orig_frame, (scale, pad_x, pad_y) = meta

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

            annotated_frame = draw_detections(
                orig_frame, boxes_xyxy, scores, class_ids,
                CLASS_NAMES, ice_class=ICE_DETECTION_CLASS)

            # RTP 上传
            try:
                rtp_frame_queue.put_nowait(annotated_frame)
            except queue.Full:
                try:
                    rtp_frame_queue.get_nowait()
                    rtp_frame_queue.put_nowait(annotated_frame)
                except queue.Empty:
                    pass

            # 本地预览
            try:
                display_frame_queue.put_nowait(annotated_frame)
            except queue.Full:
                try:
                    display_frame_queue.get_nowait()
                    display_frame_queue.put_nowait(annotated_frame)
                except queue.Empty:
                    pass

            # 结冰检测: 是否检测到 jiebing 类
            detected_ice = False
            n_boxes = len(class_ids)
            if n_boxes > 0:
                for cid in class_ids:
                    if CLASS_NAMES.get(int(cid)) == ICE_DETECTION_CLASS:
                        detected_ice = True
                        break
                # 节流: 50FPS 下每帧打这条会刷屏并反压 GIL, 限到 1Hz
                now_ts = time.time()
                if now_ts - _last_consume_log["ts"] >= 1.0:
                    print(f"检测到 {n_boxes} 个目标，结冰检测: {'是' if detected_ice else '否'}")
                    _last_consume_log["ts"] = now_ts

            # 跟 Jetson x3.py 一样, 每帧都调 handle_ice_detection.
            # 区别: Jetson 是同步调用, RK3588 经过队列异步给 ice_worker 处理,
            # 避免 50FPS 时 handle_ice_detection 里的串口/MQTT IO 反压采集.
            try:
                ice_event_queue.put_nowait((detected_ice, current_temp, current_humi, current_auto))
            except queue.Full:
                pass

        except Exception as e:
            print(f"消费线程异常: {e}")
            time.sleep(0.1)


# ============= [RK3588 适配] 本地预览窗口 (必须主线程) =============
WINDOW_NAME = "tieluchubing"


def _have_display():
    if not os.environ.get('DISPLAY'):
        return False
    if os.environ.get('SUDO_USER') and not os.environ.get('XAUTHORITY'):
        sudo_user = os.environ['SUDO_USER']
        candidate = f"/home/{sudo_user}/.Xauthority"
        if os.path.exists(candidate):
            os.environ['XAUTHORITY'] = candidate
    return True


def display_loop():
    """主线程驱动 cv2.imshow. 没桌面环境就纯等待."""
    global running
    if not _have_display():
        print("未检测到图形环境(DISPLAY 未设置), 跳过本地预览窗口")
        print("   提示: 用普通用户跑 或 sudo -E python3 x4.py")
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


# ============= 主函数 (照搬 Jetson x3.py 顺序, 替换推理/线程部分) =============
def main():
    global mqtt_client_instance, yolo_pool, rtp_sender, running
    global current_temp, current_humi, current_auto, icy_status
    global ice_detection_buffer, current_servo2_position, servo1_auto_rotation

    print("=== 铁路冰雪检测系统启动 ===")

    # 初始化全局变量
    current_temp = None
    current_humi = None
    current_auto = 1  # 默认手动模式，与单片机保持一致
    icy_status = 0
    switch_status = 0
    box_status = 0
    servo1_auto_rotation = True
    ice_detection_buffer = []  # 初始化检测缓冲区
    current_servo2_position = 0  # 初始化servo2位置为0度
    deicing_in_progress = False  # 初始化除冰操作标志

    # 1. 加载 RKNN 模型 (替换 Jetson 端 load_yolo_model)
    print("正在加载YOLO模型...")
    yolo_pool = load_yolo_pool(RKNN_MODEL_PATH)
    if not yolo_pool:
        print("YOLO模型加载失败，程序退出")
        return

    # 2. 初始化摄像头
    print("正在初始化摄像头...")
    if not init_camera():
        print("摄像头初始化失败，程序退出")
        return

    # 3. 初始化RTP发送器
    print("正在初始化RTP发送器...")
    try:
        rtp_sender = RTPSender()
        print("RTP发送器初始化成功")
    except Exception as e:
        print(f"RTP发送器初始化失败: {e}")
        return

    # 4. 打开串口（可选，某些环境可能没有串口）
    print("正在打开串口...")
    serial_ok = open_serial()
    if not serial_ok:
        print("串口打开失败，将跳过串口通信功能")

    # 5. 连接MQTT
    print("正在连接MQTT服务器...")
    mqtt_client_instance = connect_mqtt()
    if not mqtt_client_instance:
        print("MQTT连接失败，程序退出")
        return

    # 设置MQTT消息处理
    mqtt_client_instance.on_message = on_message
    mqtt_client_instance.loop_start()
    mqtt_client_instance.subscribe(MQTT_DOWN_TOPIC)

    # 6. 启动视觉检测线程组 (替换 Jetson 端单个 vision_detection_thread)
    #   capture: 采集 + 提交 NPU
    #   consume: 取推理结果 + 后处理 + 派发到 RTP / ice 队列
    #   ice_worker: 跑 handle_ice_detection (业务逻辑, 跟 Jetson 一致)
    #   rtp_sender_thread: 发 UDP, 不阻塞采集
    print("启动视觉检测线程...")
    threading.Thread(target=capture_loop, daemon=True).start()
    threading.Thread(target=consume_loop, daemon=True).start()
    threading.Thread(target=ice_worker_thread, daemon=True).start()
    threading.Thread(target=rtp_sender_thread, daemon=True).start()

    # 7. 启动串口通信线程（如果串口可用）
    if serial_ok:
        print("启动串口通信线程...")
        serial_thread = threading.Thread(target=publish_from_serial,
                                         args=(mqtt_client_instance,), daemon=True)
        serial_thread.start()

        # 手动模式下不启动servo1自动旋转
        # time.sleep(2)  # 等待串口稳定
        # print("启动servo1自动旋转...")
        # control_servo1_rotation(True)

    print("=== 系统启动完成，按Ctrl+C退出 ===")

    # 8. 主循环 - 跑本地预览窗口 (cv2.imshow 必须主线程, 没桌面就纯等待)
    try:
        display_loop()
    except KeyboardInterrupt:
        print("收到退出信号")

    # 9. 清理资源
    print("正在清理资源...")
    running = False

    if mqtt_client_instance:
        mqtt_client_instance.disconnect()
        mqtt_client_instance.loop_stop()

    if ser:
        ser.close()

    if camera_cap:
        camera_cap.release()

    if yolo_pool:
        try:
            yolo_pool.release()
        except Exception:
            pass

    if rtp_sender:
        rtp_sender.close()

    cv2.destroyAllWindows()
    print("程序已退出")


if __name__ == '__main__':
    main()

