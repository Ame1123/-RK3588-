#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
铁路冰雪检测系统 - 集成版本
结合了MQTT通信、串口控制、YOLO目标检测和RTP视频传输功能
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
# 设置OpenCV日志级别为静默
cv2.setLogLevel(0)
import socket
import struct
import numpy as np
import threading
from ultralytics import YOLO
from paho.mqtt import client as mqtt_client

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
SERIAL_PORT = '/dev/ttyTHS1'  # Jetson等设备常用串口
BAUDRATE = 115200

# RTP配置
RTP_HOST = '47.122.26.175'
RTP_PORT = 5004

# 摄像头配置
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240

# YOLO模型路径
MODEL_PATH = 'models/best.pt'

# 结冰检测配置
ICE_DETECTION_CLASS = 'jiebing'  # YOLO模型中结冰类别名称
DEICING_COOLDOWN = 10  # 除冰操作冷却时间（秒），防止频繁触发
SERVO2_DEICING_ANGLE = 45  # 除冰时舵机2的角度
SWITCH_ON_DURATION = 5  # 除冰开关打开持续时间（秒）

# Magnus公式参数（适用于0°C以下环境）- 已注释，不再使用环境条件检测
# MAGNUS_A = 17.62
# MAGNUS_B = 243.12

# ============= 全局变量 =============
running = True
mqtt_client_instance = None
ser = None
camera_cap = None
yolo_model = None
rtp_sender = None

# 结冰检测相关全局变量
current_temp = None  # 当前温度
current_humi = None  # 当前湿度
current_auto = 0  # 当前自动模式状态，默认自动模式
icy_status = 0  # 结冰状态：0=未结冰，1=结冰
switch_status = 0  # 继电器状态：0=关闭，1=打开
last_deicing_time = 0  # 上次除冰时间戳
deicing_lock = threading.Lock()  # 除冰操作锁
servo1_auto_rotation = True  # servo1自动旋转状态：True=旋转，False=停止
deicing_in_progress = False  # 除冰操作进行中标志

# 连续判断相关全局变量
ice_detection_buffer = []  # 检测结果缓冲区
CONSECUTIVE_CHECKS = 5  # 连续判断次数
current_servo2_position = 0  # 当前servo2位置：0=0度，45=45度

# ============= RTP发送器类 =============
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
                chunk = bytes(buffer[i:i+max_pkt_size])
                self.sock.sendto(header + chunk, self.dest_addr)
            self.sequence += 1
        except Exception as e:
            print(f"RTP发送失败: {e}")

    def close(self):
        if self.sock:
            self.sock.close()

# ============= 信号处理 =============
def signal_handler(sig, frame):
    """处理Ctrl+C信号"""
    global running, mqtt_client_instance, ser, camera_cap, rtp_sender
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
    
    # 关闭RTP发送器
    if rtp_sender:
        rtp_sender.close()
    
    cv2.destroyAllWindows()
    sys.exit(0)

# 注册信号处理器
signal.signal(signal.SIGINT, signal_handler)

# ============= 结冰检测相关函数 =============
# 注释掉环境条件检测函数，改为仅依靠摄像头检测
# def calculate_frost_point(temp, humi):
#     """
#     使用Magnus公式计算霜点温度
#     
#     Args:
#         temp: 空气温度（°C）
#         humi: 相对湿度（%）
#     
#     Returns:
#         霜点温度（°C），如果输入无效则返回None
#     """
#     try:
#         import math
#         
#         if temp is None or humi is None:
#             return None
#         
#         # 确保湿度在有效范围内
#         if humi <= 0 or humi > 100:
#             return None
#         
#         # Magnus公式计算霜点温度
#         alpha = (MAGNUS_A * temp) / (MAGNUS_B + temp) + math.log(humi / 100.0)
#         frost_point = (MAGNUS_B * alpha) / (MAGNUS_A - alpha)
#         
#         return frost_point
#     except Exception as e:
#         print(f"霜点温度计算异常: {e}")
#         return None

# def check_icing_conditions(temp, humi):
#     """
#     检查是否满足结冰条件
#     
#     Args:
#         temp: 空气温度（°C）
#         humi: 相对湿度（%）
#     
#     Returns:
#         bool: True表示满足结冰条件，False表示不满足
#     """
#     try:
#         if temp is None or humi is None:
#             return False
#         
#         # 温度必须 ≤ 0°C
#         if temp > 0:
#             return False
#         
#         # 计算霜点温度
#         frost_point = calculate_frost_point(temp, humi)
#         if frost_point is None:
#             return False
#         
#         # 结冰条件：空气温度 ≤ 霜点温度
#         return temp <= frost_point
#     except Exception as e:
#         print(f"结冰条件检查异常: {e}")
#         return False

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
    """
    将servo2重置到0度位置
    """
    try:
        if ser is None:
            print("串口未初始化，无法重置servo2位置")
            return
        
        # 发送servo2重置指令
        reset_data = {
            "servo2": 0
        }
        
        cjson_str = json.dumps(reset_data, separators=(',', ':')) + '\n'
        ser.write(cjson_str.encode('utf-8'))
        
        print(f"servo2重置到0度: {reset_data}")
        
        # 同步上报到云端
        if mqtt_client_instance:
            now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            msg = {
                "services": [
                    {
                        "serviceId": "wwb",
                        "properties": reset_data,
                        "event_time": now
                    }
                ]
            }
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
        
        # 发送servo2位置指令
        servo2_data = {
            "servo2": angle
        }
        
        cjson_str = json.dumps(servo2_data, separators=(',', ':')) + '\n'
        ser.write(cjson_str.encode('utf-8'))
        
        # 更新当前位置
        current_servo2_position = angle
        
        print(f"servo2位置设置为{angle}度: {servo2_data}")
        print(f"current_servo2_position更新为: {current_servo2_position}")
        
        # 同步上报到云端
        if mqtt_client_instance:
            now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            msg = {
                "services": [
                    {
                        "serviceId": "wwb",
                        "properties": servo2_data,
                        "event_time": now
                    }
                ]
            }
            json_msg = json.dumps(msg)
            result = mqtt_client_instance.publish(MQTT_TOPIC, json_msg)
            if result.rc == mqtt_client.MQTT_ERR_SUCCESS:
                print(f"servo2位置云端同步: {msg}")
            else:
                print(f"servo2位置云端同步失败, code: {result.rc}")
        
    except Exception as e:
        print(f"控制servo2位置异常: {e}")

def publish_icy_status(client, icy_value):
    """
    上报icy状态到云端
    
    Args:
        client: MQTT客户端
        icy_value: icy值（0或1）
    """
    global icy_status
    
    try:
        if client is None:
            return
        
        # 更新全局icy状态
        icy_status = icy_value
        
        now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        msg = {
            "services": [
                {
                    "serviceId": "wwb",
                    "properties": {
                        "icy": icy_value
                    },
                    "event_time": now
                }
            ]
        }
        
        json_msg = json.dumps(msg)
        result = client.publish(MQTT_TOPIC, json_msg)
        
        if result.rc == mqtt_client.MQTT_ERR_SUCCESS:
            print(f"icy状态上报: {icy_value}")
        else:
            print(f"icy状态上报失败, code: {result.rc}")
    except Exception as e:
        print(f"icy状态上报异常: {e}")

def publish_switch_status(client, switch_value):
    """
    上报switch状态到云端
    
    Args:
        client: MQTT客户端
        switch_value: switch值（0或1）
    """
    global switch_status
    
    try:
        if client is None:
            return
        
        # 更新全局switch状态
        switch_status = switch_value
        
        now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        msg = {
            "services": [
                {
                    "serviceId": "wwb",
                    "properties": {
                        "switch": switch_value
                    },
                    "event_time": now
                }
            ]
        }
        
        json_msg = json.dumps(msg)
        result = client.publish(MQTT_TOPIC, json_msg)
        
        if result.rc == mqtt_client.MQTT_ERR_SUCCESS:
            print(f"switch状态上报: {switch_value}")
        else:
            print(f"switch状态上报失败, code: {result.rc}")
    except Exception as e:
        print(f"switch状态上报异常: {e}")

def trigger_deicing():
    """
    触发除冰操作
    """
    global last_deicing_time, deicing_in_progress
    
    try:
        current_time = time.time()
        
        # 检查是否已有除冰操作在进行
        if deicing_in_progress:
            print("除冰操作已在进行中，跳过")
            return
        
        if ser is None:
            print("串口未初始化，无法执行除冰操作")
            return
        
        # 标记除冰操作开始
        deicing_in_progress = True
        
        # 执行除冰操作：switch=1, servo2=45
        deicing_data = {
            "switch": 1,
            "servo2": SERVO2_DEICING_ANGLE
        }
        
        cjson_str = json.dumps(deicing_data, separators=(',', ':')) + '\n'
        ser.write(cjson_str.encode('utf-8'))
        
        # 更新servo2位置状态同步
        global current_servo2_position
        current_servo2_position = SERVO2_DEICING_ANGLE
        
        # 更新除冰时间戳
        last_deicing_time = current_time
        
        print(f"自动模式触发除冰操作: {deicing_data}")
        print(f"同步更新current_servo2_position: {current_servo2_position}")
        
        # 上报switch状态到云端（注意：icy状态由检测逻辑负责，这里不重复上报）
        publish_switch_status(mqtt_client_instance, 1)  # 继电器打开
        print("自动除冰：继电器开启，icy状态已由检测逻辑上报")
        
        # 同步上报servo2状态到云端
        if mqtt_client_instance:
            now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            msg = {
                "services": [
                    {
                        "serviceId": "wwb",
                        "properties": {"servo2": SERVO2_DEICING_ANGLE},
                        "event_time": now
                    }
                ]
            }
            json_msg = json.dumps(msg)
            result = mqtt_client_instance.publish(MQTT_TOPIC, json_msg)
            if result.rc == mqtt_client.MQTT_ERR_SUCCESS:
                print(f"servo2除冰位置云端同步: {msg}")
            else:
                print(f"servo2除冰位置云端同步失败, code: {result.rc}")
        
        # 启动定时器，5秒后关闭开关
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
                
                # 上报switch关闭状态到云端
                publish_switch_status(mqtt_client_instance, 0)  # 继电器关闭
                
                # 除冰操作完成，标记结束
                deicing_in_progress = False
                
                # 注意：这里不直接设置icy状态，icy状态完全由检测结果决定
                print("除冰操作完成，icy状态由检测结果决定")
                        
            except Exception as e:
                deicing_in_progress = False
                print(f"关闭除冰开关异常: {e}")
        
        # 创建定时器线程，5秒后执行关闭操作
        timer = threading.Timer(SWITCH_ON_DURATION, close_switch)
        timer.daemon = True
        timer.start()
        print(f"除冰开关将在 {SWITCH_ON_DURATION} 秒后自动关闭")
                
    except Exception as e:
        deicing_in_progress = False
        print(f"除冰操作异常: {e}")

def handle_ice_detection(detected_ice, temp, humi, auto_mode):
    """
    处理结冰检测逻辑 - 连续判断版本
    
    Args:
        detected_ice: 摄像头是否检测到结冰
        temp: 当前温度
        humi: 当前湿度
        auto_mode: 自动模式状态（0=自动，1=手动）
    """
    global icy_status, servo1_auto_rotation, ice_detection_buffer
    
    try:
        # 无论手动还是自动模式，都采用连续检测逻辑来确定icy状态
        
        # 将检测结果添加到缓冲区
        ice_detection_buffer.append(detected_ice)
        
        # 保持缓冲区大小为CONSECUTIVE_CHECKS
        if len(ice_detection_buffer) > CONSECUTIVE_CHECKS:
            ice_detection_buffer.pop(0)
        
        # 打印当前状态信息
        mode_name = "手动模式" if auto_mode == 1 else "自动模式"
        print(f"{mode_name} - 当前检测: {'结冰' if detected_ice else '无冰'}, 缓冲区: {ice_detection_buffer}")
        
        # 如果缓冲区还没有满，继续收集数据
        if len(ice_detection_buffer) < CONSECUTIVE_CHECKS:
            print(f"{mode_name} - 连续判断中... ({len(ice_detection_buffer)}/{CONSECUTIVE_CHECKS})")
            return
        
        # 检查连续判断结果
        ice_count = sum(ice_detection_buffer)
        no_ice_count = len(ice_detection_buffer) - ice_count
        
        print(f"{mode_name} - 连续判断结果: 结冰={ice_count}次, 无冰={no_ice_count}次")
        
        # 手动模式：只更新icy状态，不进行自动舵机控制
        if auto_mode == 1:
            # 只有连续5次都检测到结冰才认为有冰，其他情况都按无冰处理
            if ice_count == CONSECUTIVE_CHECKS:
                print("手动模式 - 连续5次检测到结冰")
                publish_icy_status(mqtt_client_instance, 1)
            else:
                print(f"手动模式 - 未连续检测到结冰，按无冰处理")
                publish_icy_status(mqtt_client_instance, 0)
            
            # 清空缓冲区，准备下一轮判断
            ice_detection_buffer.clear()
            return
        
        # 自动模式下执行完整的连续检测逻辑（包含舵机控制和除冰操作）
        with deicing_lock:
            print(f"自动模式 - servo1旋转状态: {'是' if servo1_auto_rotation else '否'}")
            
            # 只有连续5次都检测到结冰才认为有冰并触发除冰
            if ice_count == CONSECUTIVE_CHECKS:
                print("自动模式 - 连续5次检测到结冰")
                
                # 停止servo1自动旋转
                if servo1_auto_rotation:
                    print("停止servo1自动旋转")
                    control_servo1_rotation(False)
                
                # servo2旋转至45度
                print("servo2旋转至45度")
                control_servo2_position(45)
                
                # 更新icy状态并上报（摄像头检测到结冰）
                publish_icy_status(mqtt_client_instance, 1)
                
                # 自动除冰逻辑：当自动模式时启动除冰程序
                print("自动模式下启动除冰程序")
                trigger_deicing()
                
            else:
                # 其他所有情况（包括混合结果、全无冰、间断性检测等）都按无冰处理
                print(f"自动模式 - 未连续检测到结冰，按无冰处理")
                
                # servo2旋转至0度
                if current_servo2_position != 0:
                    print("servo2旋转至0度")
                    control_servo2_position(0)
                    # 添加延时确保命令处理完成
                    time.sleep(0.1)
                
                # 开始servo1自动旋转
                if not servo1_auto_rotation:
                    print("开始servo1自动旋转")
                    control_servo1_rotation(True)
                
                # 更新icy状态并上报（按无冰处理）
                publish_icy_status(mqtt_client_instance, 0)
            
            # 清空缓冲区，准备下一轮判断
            ice_detection_buffer.clear()
            
    except Exception as e:
        print(f"结冰检测处理异常: {e}")

def handle_manual_servo_control(servo_data):
    """
    处理手动模式下的舵机控制
    
    Args:
        servo_data: 包含舵机控制指令的字典
    """
    global current_servo2_position, servo1_auto_rotation
    
    try:
        print(f"手动模式舵机控制: {servo_data}")
        
        # 处理servo1相关控制
        if 'servo1' in servo_data:
            # 手动模式下，如果收到servo1指令，停止自动旋转
            if servo1_auto_rotation:
                servo1_auto_rotation = False
                print("手动模式下停止servo1自动旋转")
        
        # 处理servo1stop指令
        if 'servo1stop' in servo_data:
            stop_rotation = bool(servo_data['servo1stop'])
            servo1_auto_rotation = not stop_rotation
            print(f"手动模式下设置servo1旋转状态: {'停止' if stop_rotation else '旋转'}")
        
        # 处理servo2位置控制
        if 'servo2' in servo_data:
            target_angle = servo_data['servo2']
            current_servo2_position = target_angle
            print(f"手动模式下设置servo2位置: {target_angle}度")
        
        # 处理switch控制
        if 'switch' in servo_data:
            switch_value = servo_data['switch']
            # 手动模式下的switch控制 - 只上报switch状态，不改变icy状态
            # icy状态完全由摄像头检测结果决定
            publish_switch_status(mqtt_client_instance, switch_value)
            if switch_value == 1:
                print("手动模式下开启继电器")
            else:
                print("手动模式下关闭继电器")
        
        # 清空检测缓冲区，避免自动模式切换时的干扰
        global ice_detection_buffer
        if ice_detection_buffer:
            ice_detection_buffer.clear()
            print("手动模式下清空检测缓冲区")
        
    except Exception as e:
        print(f"手动模式舵机控制异常: {e}")

# ============= MQTT相关函数 =============
def connect_mqtt():
    """连接MQTT服务器"""
    def on_connect(client, userdata, flags, reason_code, properties=None):
        if reason_code == 0:
            print("Connected to MQTT Broker!")
        else:
            print(f"Failed to connect, return code {reason_code}")

    def on_disconnect(client, userdata, flags, reason_code, properties=None):
        print("Disconnected from MQTT Broker")

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
            print("Disconnected from MQTT Broker")
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
        
        # 递归提取所有dict中的servo和auto参数
        def extract_servo_auto_switch(d):
            result = {}
            if isinstance(d, dict):
                for k, v in d.items():
                    if k in ['servo1', 'servo2', 'servo3', 'servo4', 'auto', 'switch']:
                        result[k] = v
                    elif isinstance(v, dict):
                        result.update(extract_servo_auto_switch(v))
                    elif isinstance(v, list):
                        for item in v:
                            result.update(extract_servo_auto_switch(item))
            return result

        servo_auto_switch_data = extract_servo_auto_switch(data)
        
        # 过滤并处理servo、auto和switch数据
        filtered_data = {}
        for k in ['servo1', 'servo2', 'servo3', 'servo4']:
            if k in servo_auto_switch_data:
                filtered_data[k] = int(servo_auto_switch_data[k])
        if 'auto' in servo_auto_switch_data:
            filtered_data['auto'] = int(servo_auto_switch_data['auto'])
            # 更新全局auto状态
            global current_auto
            old_auto = current_auto
            current_auto = filtered_data['auto']
            
            # 模式切换日志
            if old_auto != current_auto:
                mode_name = "自动模式" if current_auto == 0 else "手动模式"
                print(f"模式切换: {old_auto} -> {current_auto} ({mode_name})")
                
                # 从手动模式切换到自动模式时，清空检测缓冲区
                if old_auto == 1 and current_auto == 0:
                    global ice_detection_buffer
                    ice_detection_buffer.clear()
                    print("切换到自动模式，清空检测缓冲区")
        if 'switch' in servo_auto_switch_data:
            filtered_data['switch'] = int(servo_auto_switch_data['switch'])

        if filtered_data and ser:
            cjson_str = json.dumps(filtered_data, separators=(',', ':')) + '\n'
            try:
                ser.write(cjson_str.encode('utf-8'))
                print(f"串口下发: {cjson_str.strip()}")
                
                # 同步更新servo2位置状态
                if 'servo2' in filtered_data:
                    global current_servo2_position
                    current_servo2_position = filtered_data['servo2']
                    print(f"MQTT下发同步更新current_servo2_position: {current_servo2_position}")
                
                # 手动模式下的舵机控制处理
                if current_auto == 1:  # 手动模式
                    handle_manual_servo_control(filtered_data)
                # 添加switch状态的同步更新
                if 'switch' in filtered_data:
                    global switch_status
                    switch_status = filtered_data['switch']
                    
            except Exception as e:
                print(f"串口写入异常: {e}")
            
            # 同步上报到云端
            now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            msg = {
                "services": [
                    {
                        "serviceId": "wwb",
                        "properties": filtered_data,
                        "event_time": now
                    }
                ]
            }
            json_msg = json.dumps(msg)
            result = client.publish(MQTT_TOPIC, json_msg)
            if result.rc == mqtt_client.MQTT_ERR_SUCCESS:
                print(f"云端同步下发: {msg}")
            else:
                print(f"云端同步下发失败, code: {result.rc}")
    except Exception as e:
        print(f"处理下发消息异常: {e}")

# ============= 串口相关函数 =============
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
                # 更新全局温湿度值
                global current_temp, current_humi
                current_temp = data['temp']
                current_humi = data['humi']
                
                msg = {
                    "services": [
                        {
                            "serviceId": "wwb",
                            "properties": {
                                "temp": data['temp'],
                                "humi": data['humi']
                            },
                            "event_time": now
                        }
                    ]
                }
            elif ('servo1' in data or 'servo2' in data or 'servo3' in data or 'servo4' in data or 'auto' in data or 'switch' in data):
                properties = {}
                # 添加servo相关数据
                for k, v in data.items():
                    if k.startswith('servo'):
                        properties[k] = v
                        # 同步更新servo2位置状态
                        if k == 'servo2':
                            global current_servo2_position
                            current_servo2_position = v
                            print(f"串口反馈同步更新current_servo2_position: {current_servo2_position}")
                # 添加auto数据
                if 'auto' in data:
                    properties['auto'] = data['auto']
                    # 更新全局auto状态
                    current_auto = data['auto']
                # 添加switch数据
                if 'switch' in data:
                    properties['switch'] = data['switch']
                
                # 添加switch数据
                if 'switch' in data:
                    properties['switch'] = data['switch']
                    # 同步更新全局switch状态
                    global switch_status
                    switch_status = data['switch']
                
                # 手动模式下的舵机控制处理
                if current_auto == 1:  # 手动模式
                    handle_manual_servo_control(data)
                msg = {
                    "services": [
                        {
                            "serviceId": "wwb",
                            "properties": properties,
                            "event_time": now
                        }
                    ]
                }
            
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

# ============= YOLO相关函数 =============
def load_yolo_model(model_path):
    """加载YOLO模型"""
    if not os.path.exists(model_path):
        print(f"模型文件 {model_path} 不存在")
        return None
    
    try:
        model = YOLO(model_path)
        # 导出ONNX模型（如果不存在）
        onnx_path = os.path.splitext(model_path)[0] + '.onnx'
        if not os.path.exists(onnx_path):
            print("导出ONNX模型...")
            model.export(format="onnx", opset=12, dynamic=True, simplify=True)
        print("YOLO模型加载成功")
        return model
    except Exception as e:
        print(f"YOLO模型加载失败: {e}")
        return None

def init_camera():
    """初始化摄像头"""
    global camera_cap
    try:
        camera_cap = cv2.VideoCapture(0)
        camera_cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        camera_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        print("摄像头初始化成功")
        return True
    except Exception as e:
        print(f"摄像头初始化失败: {e}")
        return False

def vision_detection_thread():
    """视觉检测和RTP传输线程"""
    global yolo_model, camera_cap, rtp_sender, running
    
    if not yolo_model or not camera_cap or not rtp_sender:
        print("视觉检测组件未初始化")
        return
    
    print("视觉检测线程启动")
    
    while running and camera_cap.isOpened():
        try:
            ret, frame = camera_cap.read()
            if not ret:
                print("无法读取摄像头帧")
                break
            
            # YOLO检测
            results = yolo_model(frame)
            annotated_frame = results[0].plot()
            
            # 通过RTP发送视频流
            rtp_sender.send_frame(annotated_frame)
            
            # 结冰检测逻辑
            detected_ice = False
            if len(results[0].boxes) > 0:
                # 检查是否检测到结冰类别
                for box in results[0].boxes:
                    if hasattr(box, 'cls') and box.cls is not None:
                        # 获取类别名称
                        class_id = int(box.cls[0])
                        if hasattr(yolo_model, 'names') and class_id in yolo_model.names:
                            class_name = yolo_model.names[class_id]
                            if class_name == ICE_DETECTION_CLASS:
                                detected_ice = True
                                break
                
                print(f"检测到 {len(results[0].boxes)} 个目标，结冰检测: {'是' if detected_ice else '否'}")
            
            # 处理结冰检测逻辑
            handle_ice_detection(detected_ice, current_temp, current_humi, current_auto)
            
            time.sleep(0.033)  # 约30fps
            
        except Exception as e:
            print(f"视觉检测异常: {e}")
            time.sleep(1)
    
    print("视觉检测线程退出")

# ============= 主函数 =============
def main():
    """主函数"""
    global mqtt_client_instance, yolo_model, rtp_sender, running
    global current_temp, current_humi, current_auto, icy_status
    global ice_detection_buffer, current_servo2_position, servo1_auto_rotation
    
    print("=== 铁路冰雪检测系统启动 ===")
    
    # 初始化全局变量
    current_temp = None
    current_humi = None
    current_auto = 0  # 默认自动模式
    icy_status = 0
    switch_status = 0
    servo1_auto_rotation = True
    ice_detection_buffer = []  # 初始化检测缓冲区
    current_servo2_position = 0  # 初始化servo2位置为0度
    deicing_in_progress = False  # 初始化除冰操作标志
    
    # 1. 初始化YOLO模型
    print("正在加载YOLO模型...")
    yolo_model = load_yolo_model(MODEL_PATH)
    if not yolo_model:
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
    
    # 6. 启动视觉检测线程
    print("启动视觉检测线程...")
    vision_thread = threading.Thread(target=vision_detection_thread, daemon=True)
    vision_thread.start()
    
    # 7. 启动串口通信线程（如果串口可用）
    if serial_ok:
        print("启动串口通信线程...")
        serial_thread = threading.Thread(target=publish_from_serial, args=(mqtt_client_instance,), daemon=True)
        serial_thread.start()
        
        # 启动servo1自动旋转
        time.sleep(2)  # 等待串口稳定
        print("启动servo1自动旋转...")
        control_servo1_rotation(True)
    
    print("=== 系统启动完成，按Ctrl+C退出 ===")
    
    # 8. 主循环
    try:
        while running:
            time.sleep(1)
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
    
    if rtp_sender:
        rtp_sender.close()
    
    cv2.destroyAllWindows()
    print("程序已退出")

if __name__ == '__main__':
    main()
