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
# MQTT配置（根据新参数修改）
MQTT_BROKER = '382ed4958e.st1.iotda-device.cn-south-1.myhuaweicloud.com'
MQTT_PORT = 8883
MQTT_CLIENT_ID = '6956379ec00ccb6d4b30120d_xianlan_0_0_2026010109'
MQTT_USERNAME = '6956379ec00ccb6d4b30120d_xianlan'
MQTT_PASSWORD = os.environ.get('MQTT_PASSWORD', '')
# 设备ID
DEVICE_ID = "6956379ec00ccb6d4b30120d_xianlan"
# 上报Topic
MQTT_TOPIC = f"$oc/devices/{DEVICE_ID}/sys/properties/report"
# 下发Topic
MQTT_DOWN_TOPIC = f"$oc/devices/{DEVICE_ID}/sys/messages/down"

# 串口配置
SERIAL_PORT = '/dev/ttyTHS1'  # Jetson等设备常用串口
BAUDRATE = 115200

# RTP配置
RTP_HOST = '47.109.108.78'  # Flask视频流服务器地址
RTP_PORT = 5004

# 摄像头配置
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240

# YOLO模型路径
MODEL_PATH = 'models/best.pt'

# 结冰检测配置
ICE_DETECTION_CLASS = 'jiebing'  # YOLO模型中结冰类别名称
DEICING_COOLDOWN = 10  # 除冰操作冷却时间（秒），防止频繁触发
MOTOR_DEICING_SPEED = 80  # 除冰时伸缩电机速度 (0-100, 0=停止, 100=最快)
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
current_wind_speed = None  # 当前风速 (m/s)
current_wind_dir = None  # 当前风向 (度, 0-360)
current_wind_valid = 0  # 风速风向数据有效标志
current_rain = 0  # 当前雨雪状态：0=正常（无雨雪），1=报警（有雨雪）
current_rain_valid = 0  # 雨雪数据有效标志
current_auto = 1  # 当前自动模式状态，默认手动模式（1=手动，更安全）
icy_status = 0  # 结冰状态：0=未结冰，1=结冰
last_deicing_time = 0  # 上次除冰时间戳
deicing_lock = threading.Lock()  # 除冰操作锁
servo1_auto_rotation = True  # servo1自动旋转状态：True=旋转，False=停止

# 启动保护：忽略云端推送的初始auto状态
startup_time = None  # 程序启动时间
STARTUP_IGNORE_DURATION = 10  # 启动后10秒内忽略云端的auto命令

# 连续判断相关全局变量
ice_detection_buffer = []  # 检测结果缓冲区
CONSECUTIVE_CHECKS = 5  # 连续判断次数
current_motor_speed = 0  # 当前伸缩电机速度：0=停止，100=最快

# ============= 增量上报优化配置 =============
# 上次上报的值（用于变化检测）
last_reported = {
    'temp': None,
    'humi': None,
    'wind_speed': None,
    'wind_dir': None,
    'rain': None,
    'icy': None,  # 结冰状态
    'servo1': None,
    'servo2': None,
    'servo3': None,
    'servo4': None,
    'auto': None,
    'switch': None,
    'motor_speed': None,
    'last_report_time': 0  # 上次强制上报时间
}

# 变化检测阈值（过滤传感器噪声，避免无意义的频繁上报）
# 注：STM32上报精度为2位小数，设置阈值略大于噪声波动即可
REPORT_THRESHOLDS = {
    'temp': 0.05,       # 温度变化超过0.05°C才上报（过滤ADC噪声）
    'humi': 0.1,        # 湿度变化超过0.1%才上报（过滤ADC噪声）
    'wind_speed': 0.05, # 风速变化超过0.05m/s才上报
    'wind_dir': 1,      # 风向变化超过1°才上报（整数，基本无噪声）
}

# 强制上报间隔（秒）- 即使没有变化，每隔这么久也上报一次
FORCED_REPORT_INTERVAL = 10  # 改为10秒强制上报一次

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
    global servo1_auto_rotation, current_auto
    
    try:
        if ser is None:
            print("串口未初始化，无法控制servo1旋转")
            return
        
        # 每次都先发送auto:0确保STM32处于自动模式
        # 这样servo1stop命令才会被处理（STM32只在自动模式下处理servo1stop）
        print(f"确保STM32处于自动模式 (当前Python端current_auto={current_auto})")
        mode_data = {"auto": 0}
        cjson_str = json.dumps(mode_data, separators=(',', ':')) + '\n'
        ser.write(cjson_str.encode('utf-8'))
        current_auto = 0
        print(f"已发送自动模式命令: {mode_data}")
        # 延时等待模式切换完成
        time.sleep(0.15)
        
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
                        "serviceId": "xianlan",
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

def stop_motor():
    """
    停止伸缩电机（速度设为0）
    """
    try:
        if ser is None:
            print("串口未初始化，无法停止电机")
            return
        
        # 发送电机停止指令
        motor_data = {
            "motor_speed": 0
        }
        
        cjson_str = json.dumps(motor_data, separators=(',', ':')) + '\n'
        ser.write(cjson_str.encode('utf-8'))
        
        print(f"电机停止: {motor_data}")
        
        # 同步上报到云端
        if mqtt_client_instance:
            now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            msg = {
                "services": [
                    {
                        "serviceId": "xianlan",
                        "properties": motor_data,
                        "event_time": now
                    }
                ]
            }
            json_msg = json.dumps(msg)
            result = mqtt_client_instance.publish(MQTT_TOPIC, json_msg)
            if result.rc == mqtt_client.MQTT_ERR_SUCCESS:
                print(f"电机停止状态云端同步: {msg}")
            else:
                print(f"电机停止状态云端同步失败, code: {result.rc}")
        
    except Exception as e:
        print(f"停止电机异常: {e}")

def set_motor_speed(speed):
    """
    设置伸缩电机速度
    
    Args:
        speed: 电机速度 (0-100, 0=停止, 100=最快)
    """
    global current_motor_speed
    
    try:
        if ser is None:
            print("串口未初始化，无法控制电机速度")
            return
        
        # 限制速度范围
        speed = max(0, min(100, speed))
        
        print(f"set_motor_speed调用: 目标速度={speed}, 当前速度={current_motor_speed}")
        
        # 如果目标速度与当前速度相同，不需要重复发送
        if speed == current_motor_speed:
            print(f"电机速度已经是{speed}，无需重复设置")
            return
        
        # 发送电机速度指令
        motor_data = {
            "motor_speed": speed
        }
        
        cjson_str = json.dumps(motor_data, separators=(',', ':')) + '\n'
        ser.write(cjson_str.encode('utf-8'))
        
        # 更新当前速度
        current_motor_speed = speed
        
        print(f"电机速度设置为{speed}: {motor_data}")
        print(f"current_motor_speed更新为: {current_motor_speed}")
        
        # 同步上报到云端
        if mqtt_client_instance:
            now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            msg = {
                "services": [
                    {
                        "serviceId": "xianlan",
                        "properties": motor_data,
                        "event_time": now
                    }
                ]
            }
            json_msg = json.dumps(msg)
            result = mqtt_client_instance.publish(MQTT_TOPIC, json_msg)
            if result.rc == mqtt_client.MQTT_ERR_SUCCESS:
                print(f"电机速度云端同步: {msg}")
            else:
                print(f"电机速度云端同步失败, code: {result.rc}")
        
    except Exception as e:
        print(f"控制电机速度异常: {e}")

def sync_icy_to_stm32(icy_value):
    """
    同步icy状态到STM32（用于OLED显示）
    """
    try:
        if ser is not None:
            icy_data = {"icy": icy_value}
            cjson_str = json.dumps(icy_data, separators=(',', ':')) + '\n'
            ser.write(cjson_str.encode('utf-8'))
            print(f"icy状态同步到STM32: {icy_data}")
            return True
        return False
    except Exception as e:
        print(f"icy同步STM32异常: {e}")
        return False

def publish_icy_status(client, icy_value):
    """
    上报icy状态到云端
    """
    try:
        if client is None:
            return
        
        now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        msg = {
            "services": [
                {
                    "serviceId": "xianlan",
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
            print(f"icy状态上报云端: {icy_value}")
        else:
            print(f"icy状态上报失败, code: {result.rc}")
    except Exception as e:
        print(f"icy状态上报异常: {e}")

def icy_sync_thread_func():
    """
    icy状态同步线程，每秒发送一次icy状态到STM32和云端
    """
    global running, icy_status, mqtt_client_instance
    
    print("启icy同步线程，每秒发送一次")
    
    while running:
        try:
            # 同步到STM32
            sync_icy_to_stm32(icy_status)
            # 同步到云端
            publish_icy_status(mqtt_client_instance, icy_status)
        except Exception as e:
            print(f"icy同步异常: {e}")
        
        # 等待1秒
        time.sleep(1)

def rain_sync_thread_func():
    """
    雨雪状态同步线程，每秒发送一次雨雪状态到云端
    """
    global running, current_rain, mqtt_client_instance
    
    print("启动雨雪同步线程，每秒发送一次")
    
    while running:
        try:
            if mqtt_client_instance:
                now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
                msg = {
                    "services": [
                        {
                            "serviceId": "xianlan",
                            "properties": {
                                "rain": current_rain
                            },
                            "event_time": now
                        }
                    ]
                }
                json_msg = json.dumps(msg)
                result = mqtt_client_instance.publish(MQTT_TOPIC, json_msg)
                if result.rc == mqtt_client.MQTT_ERR_SUCCESS:
                    print(f"雨雪状态上报云端: {'有雨雪' if current_rain == 1 else '无雨雪'}")
                else:
                    print(f"雨雪状态上报失败, code: {result.rc}")
        except Exception as e:
            print(f"雨雪同步异常: {e}")
        
        # 等待1秒
        time.sleep(1)

def control_voice(play):
    """
    控制语音模块播放/停止
    
    Args:
        play: True=播放语音, False=停止语音
    """
    global ser
    
    try:
        if ser is None:
            print("串口未初始化，无法控制语音模块")
            return
        
        voice_value = 1 if play else 0
        voice_data = {"voice": voice_value}
        cjson_str = json.dumps(voice_data, separators=(',', ':')) + '\n'
        ser.write(cjson_str.encode('utf-8'))
        
        status = "播放" if play else "停止"
        print(f"语音模块{status}: {voice_data}")
        
        # 同步上报到云端
        if mqtt_client_instance:
            now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            msg = {
                "services": [
                    {
                        "serviceId": "xianlan",
                        "properties": voice_data,
                        "event_time": now
                    }
                ]
            }
            json_msg = json.dumps(msg)
            result = mqtt_client_instance.publish(MQTT_TOPIC, json_msg)
            if result.rc == mqtt_client.MQTT_ERR_SUCCESS:
                print(f"语音状态云端同步: {msg}")
            else:
                print(f"语音状态云端同步失败, code: {result.rc}")
        
    except Exception as e:
        print(f"控制语音模块异常: {e}")

def trigger_deicing():
    """
    触发除冰操作
    """
    global last_deicing_time
    
    try:
        current_time = time.time()
        
        # 取消冷却时间检查，允许立即执行除冰操作
        # if current_time - last_deicing_time < DEICING_COOLDOWN:
        #     print(f"除冰操作仍在冷却中，剩余 {int(DEICING_COOLDOWN - (current_time - last_deicing_time))} 秒")
        #     return
        
        if ser is None:
            print("串口未初始化，无法执行除冰操作")
            return
        
        # 执行除冰操作：switch=1, motor_speed=除冰速度
        deicing_data = {
            "switch": 1,
            "motor_speed": MOTOR_DEICING_SPEED
        }
        
        cjson_str = json.dumps(deicing_data, separators=(',', ':')) + '\n'
        ser.write(cjson_str.encode('utf-8'))
        
        # 更新电机速度状态同步
        global current_motor_speed
        current_motor_speed = MOTOR_DEICING_SPEED
        
        # 更新除冰时间戳
        last_deicing_time = current_time
        
        print(f"执行除冰操作: {deicing_data}")
        print(f"同步更新current_motor_speed: {current_motor_speed}")
        
        # 同步上报到云端
        if mqtt_client_instance:
            now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            msg = {
                "services": [
                    {
                        "serviceId": "xianlan",
                        "properties": deicing_data,
                        "event_time": now
                    }
                ]
            }
            json_msg = json.dumps(msg)
            result = mqtt_client_instance.publish(MQTT_TOPIC, json_msg)
            if result.rc == mqtt_client.MQTT_ERR_SUCCESS:
                print(f"除冰操作云端同步: {msg}")
            else:
                print(f"除冰操作云端同步失败, code: {result.rc}")
        
        # 启动定时器，5秒后关闭开关
        def close_switch():
            try:
                if ser is None:
                    print("串口未初始化，无法关闭除冰开关")
                    return
                
                close_data = {"switch": 0}
                cjson_str = json.dumps(close_data, separators=(',', ':')) + '\n'
                ser.write(cjson_str.encode('utf-8'))
                
                print(f"除冰开关自动关闭: {close_data}")
                
                # 同步上报关闭状态到云端
                if mqtt_client_instance:
                    now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
                    msg = {
                        "services": [
                            {
                                "serviceId": "xianlan",
                                "properties": close_data,
                                "event_time": now
                            }
                        ]
                    }
                    json_msg = json.dumps(msg)
                    result = mqtt_client_instance.publish(MQTT_TOPIC, json_msg)
                    if result.rc == mqtt_client.MQTT_ERR_SUCCESS:
                        print(f"除冰开关关闭云端同步: {msg}")
                    else:
                        print(f"除冰开关关闭云端同步失败, code: {result.rc}")
                        
            except Exception as e:
                print(f"关闭除冰开关异常: {e}")
        
        # 创建定时器线程，5秒后执行关闭操作
        timer = threading.Timer(SWITCH_ON_DURATION, close_switch)
        timer.daemon = True
        timer.start()
        print(f"除冰开关将在 {SWITCH_ON_DURATION} 秒后自动关闭")
                
    except Exception as e:
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
        # 手动模式下只更新icy状态，不进行自动舵机控制
        if auto_mode == 1:
            # 手动模式：只更新icy状态，不控制舵机
            detected_icy_status = 1 if detected_ice else 0
            if icy_status != detected_icy_status:
                icy_status = detected_icy_status
                publish_icy_status(mqtt_client_instance, icy_status)
                print(f"手动模式下更新icy状态: {icy_status}")
            
            # 手动模式下清空检测缓冲区，避免影响模式切换
            if ice_detection_buffer:
                ice_detection_buffer.clear()
                print("手动模式下清空检测缓冲区")
            
            return
        
        # 自动模式下执行原有的连续检测逻辑
        with deicing_lock:
            # 将检测结果添加到缓冲区
            ice_detection_buffer.append(detected_ice)
            
            # 保持缓冲区大小为CONSECUTIVE_CHECKS
            if len(ice_detection_buffer) > CONSECUTIVE_CHECKS:
                ice_detection_buffer.pop(0)
            
            # 打印当前状态信息
            print(f"当前检测: {'结冰' if detected_ice else '无冰'}, servo1旋转状态: {'是' if servo1_auto_rotation else '否'}, 缓冲区: {ice_detection_buffer}")
            
            # 如果缓冲区还没有满，继续收集数据
            if len(ice_detection_buffer) < CONSECUTIVE_CHECKS:
                print(f"连续判断中... ({len(ice_detection_buffer)}/{CONSECUTIVE_CHECKS})")
                return
            
            # 检查连续判断结果
            ice_count = sum(ice_detection_buffer)
            no_ice_count = len(ice_detection_buffer) - ice_count
            
            print(f"连续判断结果: 结冰={ice_count}次, 无冰={no_ice_count}次")
            
            # 情况1：连续5次都检测到结冰
            if ice_count == CONSECUTIVE_CHECKS:
                print("连续5次检测到结冰")
                
                # 1. 首先播放语音警报（最重要，先发送）
                print("检测到结冰，播放语音警报")
                control_voice(True)
                time.sleep(0.1)  # 等待语音命令被处理
                
                # 2. 停止servo1自动旋转
                if servo1_auto_rotation:
                    print("停止servo1自动旋转")
                    control_servo1_rotation(False)
                
                # 3. 伸缩电机启动（速度设为80）
                print("伸缩电机启动，速度设为80")
                set_motor_speed(80)
                
                # 4. 更新icy状态（由同步线程每秒发送）
                icy_status = 1
                
                # 5. 自动除冰逻辑
                if auto_mode == 0 or auto_mode is None:
                    print("自动模式下启动除冰程序")
                    trigger_deicing()
                
                # 清空缓冲区，准备下一轮判断
                ice_detection_buffer.clear()
                
            # 情况2：连续5次都检测无冰
            elif no_ice_count == CONSECUTIVE_CHECKS:
                print("="*50)
                print("连续5次检测无冰 - 开始恢复流程")
                print(f"当前servo1旋转状态: {servo1_auto_rotation}")
                print("="*50)
                
                # 伸缩电机停止（速度设为0）
                print("步骤1: 伸缩电机停止")
                set_motor_speed(0)
                
                # 添加延时确保电机命令处理完成
                time.sleep(0.2)
                
                # 开始servo1自动旋转
                print(f"步骤2: 检查servo1状态 - 当前: {servo1_auto_rotation}")
                if not servo1_auto_rotation:
                    print(">>> 执行恢复servo1自动旋转 <<<")
                    control_servo1_rotation(True)
                    # 额外延时确保servo1命令被处理
                    time.sleep(0.2)
                    print(f">>> servo1旋转状态已更新为: {servo1_auto_rotation} <<<")
                else:
                    print("servo1已经在旋转中，无需重复启动")
                
                # 更新icy状态（由同步线程每秒发送）
                print("步骤3: 更新icy状态")
                icy_status = 0
                print(f"icy状态: {icy_status}")
                
                # 停止语音播放
                print("步骤4: 停止语音播放")
                control_voice(False)
                
                # 清空缓冲区，准备下一轮判断
                ice_detection_buffer.clear()
                print("步骤5: 清空检测缓冲区，恢复流程完成")
                print("="*50)
                
            # 情况3：混合结果，根据多数判断
            else:
                print(f"检测结果混合 (结冰={ice_count}次, 无冰={no_ice_count}次)")
                
                # 如果无冰次数占多数，恢复正常状态
                if no_ice_count > ice_count:
                    print("无冰次数占多数，恢复正常状态")
                    # 停止电机
                    set_motor_speed(0)
                    time.sleep(0.1)
                    # 恢复servo1旋转
                    if not servo1_auto_rotation:
                        control_servo1_rotation(True)
                    # 停止语音
                    control_voice(False)
                    # 更新icy状态（由同步线程每秒发送）
                    icy_status = 0
                
                # 如果结冰次数占多数，执行除冰
                elif ice_count > no_ice_count:
                    print("结冰次数占多数，执行除冰")
                    # 1. 首先播放语音（最重要，先发送）
                    control_voice(True)
                    time.sleep(0.1)
                    # 2. 停止servo1旋转
                    if servo1_auto_rotation:
                        control_servo1_rotation(False)
                        time.sleep(0.1)
                    # 3. 启动电机
                    set_motor_speed(80)
                    # 4. 更新icy状态（由同步线程每秒发送）
                    icy_status = 1
                
                # 结冰和无冰次数相等，保持当前状态
                else:
                    print("结冰和无冰次数相等，保持当前状态")
                
                # 清空缓冲区
                ice_detection_buffer.clear()
            
    except Exception as e:
        print(f"结冰检测处理异常: {e}")

def handle_manual_control(control_data):
    """
    处理手动模式下的舵机和电机控制
    
    Args:
        control_data: 包含控制指令的字典
    """
    global current_motor_speed, servo1_auto_rotation
    
    try:
        print(f"手动模式控制: {control_data}")
        
        # 处理servo1相关控制
        if 'servo1' in control_data:
            # 手动模式下，如果收到servo1指令，停止自动旋转
            if servo1_auto_rotation:
                servo1_auto_rotation = False
                print("手动模式下停止servo1自动旋转")
        
        # 处理servo1stop指令
        if 'servo1stop' in control_data:
            stop_rotation = bool(control_data['servo1stop'])
            servo1_auto_rotation = not stop_rotation
            print(f"手动模式下设置servo1旋转状态: {'停止' if stop_rotation else '旋转'}")
        
        # 处理电机速度控制
        if 'motor_speed' in control_data:
            target_speed = control_data['motor_speed']
            current_motor_speed = target_speed
            print(f"手动模式下设置电机速度: {target_speed}")
        
        # 清空检测缓冲区，避免自动模式切换时的干扰
        global ice_detection_buffer
        if ice_detection_buffer:
            ice_detection_buffer.clear()
            print("手动模式下清空检测缓冲区")
        
    except Exception as e:
        print(f"手动模式控制异常: {e}")

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
    global startup_time
    
    try:
        payload = msg.payload.decode('utf-8')
        print(f"收到云端下发: {payload}")
        data = json.loads(payload)
        
        # 递归提取所有dict中的servo、motor和auto参数
        def extract_control_data(d):
            result = {}
            if isinstance(d, dict):
                for k, v in d.items():
                    if k in ['servo1', 'servo2', 'servo3', 'servo4', 'motor_speed', 'auto', 'switch', 'voice']:
                        result[k] = v
                    elif isinstance(v, dict):
                        result.update(extract_control_data(v))
                    elif isinstance(v, list):
                        for item in v:
                            result.update(extract_control_data(item))
            return result

        control_data = extract_control_data(data)
        print(f"提取的控制数据: {control_data}")  # 调试日志
        
        # 过滤并处理servo、motor、auto和switch数据
        filtered_data = {}
        for k in ['servo1', 'servo2', 'servo3', 'servo4']:
            if k in control_data:
                filtered_data[k] = int(control_data[k])
        
        print(f"过滤后的数据: {filtered_data}")  # 调试日志
        
        # 处理motor_speed
        if 'motor_speed' in control_data:
            filtered_data['motor_speed'] = int(control_data['motor_speed'])
        
        # 启动保护：在启动后一段时间内忽略云端推送的auto命令（防止影子状态覆盖）
        if 'auto' in control_data:
            if startup_time and (time.time() - startup_time) < STARTUP_IGNORE_DURATION:
                print(f"启动保护期内（{STARTUP_IGNORE_DURATION}秒），忽略云端auto命令: {control_data['auto']}")
                # 不处理auto命令，从filtered_data中移除
            else:
                filtered_data['auto'] = int(control_data['auto'])
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
                        
                        # 舵机1和舵机2归位到90度
                        print("舵机归位：servo1=90, servo2=90")
                        servo_data = {"servo1": 90, "servo2": 90}
                        cjson_str_servo = json.dumps(servo_data, separators=(',', ':')) + '\n'
                        ser.write(cjson_str_servo.encode('utf-8'))
                        print(f"发送舵机归位命令: {servo_data}")
        if 'switch' in control_data:
            filtered_data['switch'] = int(control_data['switch'])
        
        # 语音模块控制 (voice: 0=停止, 1=播放)
        if 'voice' in control_data:
            filtered_data['voice'] = int(control_data['voice'])
            print(f"语音控制: {'播放' if filtered_data['voice'] == 1 else '停止'}")

        if filtered_data and ser:
            cjson_str = json.dumps(filtered_data, separators=(',', ':')) + '\n'
            try:
                ser.write(cjson_str.encode('utf-8'))
                print(f"串口下发: {cjson_str.strip()}")
                
                # 同步更新电机速度状态
                if 'motor_speed' in filtered_data:
                    global current_motor_speed
                    current_motor_speed = filtered_data['motor_speed']
                    print(f"MQTT下发同步更新current_motor_speed: {current_motor_speed}")
                
                # 手动模式下的控制处理
                if current_auto == 1:  # 手动模式
                    handle_manual_control(filtered_data)
                    
            except Exception as e:
                print(f"串口写入异常: {e}")
            
            # 同步上报到云端
            now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            msg = {
                "services": [
                    {
                        "serviceId": "xianlan",
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

def should_report(key, new_value):
    """
    判断是否应该上报数据（增量上报优化）
    返回: (should_report: bool, reason: str)
    """
    global last_reported
    
    old_value = last_reported.get(key)
    current_time = time.time()
    
    # 首次上报
    if old_value is None:
        return True, "首次上报"
    
    # 强制上报间隔检查
    if current_time - last_reported.get('last_report_time', 0) >= FORCED_REPORT_INTERVAL:
        return True, f"强制上报({FORCED_REPORT_INTERVAL}s)"
    
    # 开关类数据：值变化就上报
    if key in ['rain', 'auto', 'switch', 'icy']:
        if new_value != old_value:
            return True, f"{key}变化: {old_value}->{new_value}"
        return False, "无变化"
    
    # 数值类数据：检查是否变化（阈值为0表示任何变化都上报）
    threshold = REPORT_THRESHOLDS.get(key, 0)
    if threshold == 0:
        # 零阈值：只要值不同就上报
        if new_value != old_value:
            return True, f"{key}变化: {old_value}->{new_value}"
    elif abs(new_value - old_value) >= threshold:
        # 有阈值：超过阈值才上报
        return True, f"{key}变化: {old_value:.1f}->{new_value:.1f} (阈值{threshold})"
    
    # 舵机角度：变化超过1度才上报
    if key.startswith('servo'):
        if abs(new_value - old_value) >= 1:
            return True, f"{key}变化: {old_value}->{new_value}"
        return False, "无变化"
    
    # 电机速度：变化就上报
    if key == 'motor_speed':
        if new_value != old_value:
            return True, f"motor_speed变化: {old_value}->{new_value}"
        return False, "无变化"
    
    return False, "无变化"


def update_last_reported(key, value):
    """更新上次上报的值"""
    global last_reported
    last_reported[key] = value
    last_reported['last_report_time'] = time.time()


def publish_from_serial(client):
    """从串口读取数据并发布到MQTT（增量上报优化版）"""
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
                
                # 增量上报检查
                should_temp, reason_temp = should_report('temp', data['temp'])
                should_humi, reason_humi = should_report('humi', data['humi'])
                
                if should_temp or should_humi:
                    msg = {
                        "services": [
                            {
                                "serviceId": "xianlan",
                                "properties": {
                                    "temp": data['temp'],
                                    "humi": data['humi']
                                },
                                "event_time": now
                            }
                        ]
                    }
                    update_last_reported('temp', data['temp'])
                    update_last_reported('humi', data['humi'])
                    print(f"📤 温湿度上报: {data['temp']:.1f}°C, {data['humi']:.1f}% ({reason_temp if should_temp else reason_humi})")
                else:
                    print(f"⏭️ 温湿度跳过: {data['temp']:.1f}°C, {data['humi']:.1f}% (无显著变化)")
                    
            elif 'wind_speed' in data and 'wind_dir' in data:
                # 更新全局风速风向值
                global current_wind_speed, current_wind_dir, current_wind_valid
                current_wind_speed = data['wind_speed']
                current_wind_dir = data['wind_dir']
                current_wind_valid = data.get('wind_valid', 0)
                
                # 增量上报检查
                should_speed, reason_speed = should_report('wind_speed', current_wind_speed)
                should_dir, reason_dir = should_report('wind_dir', current_wind_dir)
                
                if should_speed or should_dir:
                    msg = {
                        "services": [
                            {
                                "serviceId": "xianlan",
                                "properties": {
                                    "wind_speed": current_wind_speed,
                                    "wind_dir": current_wind_dir
                                },
                                "event_time": now
                            }
                        ]
                    }
                    update_last_reported('wind_speed', current_wind_speed)
                    update_last_reported('wind_dir', current_wind_dir)
                    print(f"📤 风速风向上报: {current_wind_speed} m/s, {current_wind_dir}° ({reason_speed if should_speed else reason_dir})")
                else:
                    print(f"⏭️ 风速风向跳过: {current_wind_speed} m/s, {current_wind_dir}° (无显著变化)")
                    
            elif 'rain' in data:
                # 更新全局雨雪状态（由同步线程每秒上报）
                global current_rain, current_rain_valid
                current_rain = data['rain']
                current_rain_valid = data.get('rain_valid', 0)
                print(f"雨雪状态更新: {'有雨雪' if current_rain == 1 else '无雨雪'}")
            elif ('servo1' in data or 'servo2' in data or 'servo3' in data or 'servo4' in data or 'motor_speed' in data or 'auto' in data or 'switch' in data):
                properties = {}
                changed_keys = []
                
                # 添加servo相关数据（增量检查）
                for k, v in data.items():
                    if k.startswith('servo'):
                        should, reason = should_report(k, v)
                        if should:
                            properties[k] = v
                            update_last_reported(k, v)
                            changed_keys.append(f"{k}={v}")
                    # 同步更新电机速度状态
                    if k == 'motor_speed':
                        global current_motor_speed
                        current_motor_speed = v
                        should, reason = should_report('motor_speed', v)
                        if should:
                            properties[k] = v
                            update_last_reported('motor_speed', v)
                            changed_keys.append(f"motor_speed={v}")
                            print(f"串口反馈同步更新current_motor_speed: {current_motor_speed}")
                
                # 添加auto数据（开关类，变化就上报）
                if 'auto' in data:
                    current_auto = data['auto']
                    should, reason = should_report('auto', data['auto'])
                    if should:
                        properties['auto'] = data['auto']
                        update_last_reported('auto', data['auto'])
                        changed_keys.append(f"auto={data['auto']}")
                
                # 添加switch数据（开关类，变化就上报）
                if 'switch' in data:
                    should, reason = should_report('switch', data['switch'])
                    if should:
                        properties['switch'] = data['switch']
                        update_last_reported('switch', data['switch'])
                        changed_keys.append(f"switch={data['switch']}")
                
                # 手动模式下的控制处理
                if current_auto == 1:  # 手动模式
                    handle_manual_control(data)
                
                # 只有有变化的属性才上报
                if properties:
                    msg = {
                        "services": [
                            {
                                "serviceId": "xianlan",
                                "properties": properties,
                                "event_time": now
                            }
                        ]
                    }
                    print(f"📤 控制状态上报: {', '.join(changed_keys)}")
                else:
                    print(f"⏭️ 控制状态跳过: 无显著变化")
            
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
    """初始化摄像头 - 自动尝试多种设备"""
    global camera_cap
    
    # 可能的摄像头设备列表（按优先级排序）
    camera_sources = [0, 1, 2]
    
    for source in camera_sources:
        try:
            print(f"尝试摄像头设备: {source}")
            camera_cap = cv2.VideoCapture(source)
            
            if camera_cap.isOpened():
                camera_cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
                camera_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
                print(f"✅ 摄像头初始化成功 (设备: {source})")
                return True
            else:
                camera_cap.release()
                
        except Exception as e:
            print(f"设备 {source} 失败: {e}")
            continue
    
    # 所有设备都尝试失败
    print("=" * 50)
    print("⚠️  警告：未找到可用的摄像头！")
    print("   系统将在无摄像头模式下继续运行")
    print("   MQTT、串口通信等功能不受影响")
    print("=" * 50)
    camera_cap = None
    return False

def vision_detection_thread():
    """视觉检测和RTP传输线程"""
    global yolo_model, camera_cap, rtp_sender, running
    
    if not yolo_model:
        print("⚠️  YOLO模型未初始化，视觉检测功能禁用")
        return
    
    if not camera_cap:
        print("=" * 50)
        print("⚠️  摄像头未初始化，视觉检测线程无法启动")
        print("   系统将在没有摄像头的情况下继续运行")
        print("   其他功能（MQTT、串口通信等）不受影响")
        print("=" * 50)
        return
    
    if not rtp_sender:
        print("⚠️  RTP发送器未初始化，视频流功能禁用")
    
    print("✅ 视觉检测线程启动")
    
    frame_error_count = 0  # 连续读取失败计数
    MAX_FRAME_ERRORS = 10  # 最大连续失败次数
    
    while running and camera_cap and camera_cap.isOpened():
        try:
            ret, frame = camera_cap.read()
            if not ret:
                frame_error_count += 1
                if frame_error_count >= MAX_FRAME_ERRORS:
                    print("=" * 50)
                    print("❌ 摄像头连续读取失败，可能已断开连接！")
                    print("   请检查摄像头连接")
                    print("=" * 50)
                    break
                time.sleep(0.1)
                continue
            
            frame_error_count = 0  # 重置错误计数
            
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
    global ice_detection_buffer, current_motor_speed, servo1_auto_rotation
    global startup_time
    
    print("=== 铁路冰雪检测系统启动 ===")
    
    # 记录启动时间，用于启动保护
    startup_time = time.time()
    print(f"启动保护：{STARTUP_IGNORE_DURATION}秒内忽略云端auto命令")
    
    # 初始化全局变量
    current_temp = None
    current_humi = None
    current_auto = 1  # 默认手动模式（1=手动），更安全，避免误触发自动除冰
    icy_status = 0
    servo1_auto_rotation = True
    ice_detection_buffer = []  # 初始化检测缓冲区
    current_motor_speed = 0  # 初始化电机速度为0（停止）

    # 1. 初始化YOLO模型
    print("正在加载YOLO模型...")
    yolo_model = load_yolo_model(MODEL_PATH)
    if not yolo_model:
        print("YOLO模型加载失败，程序退出")
        return
    
    # 2. 初始化摄像头（可选，失败不影响其他功能）
    print("正在初始化摄像头...")
    camera_ok = init_camera()
    if not camera_ok:
        print("摄像头初始化失败，视觉检测功能将禁用")
        # 不退出程序，继续运行其他功能
    
    # 3. 初始化RTP发送器（可选，仅在摄像头可用时需要）
    if camera_ok:
        print("正在初始化RTP发送器...")
        try:
            rtp_sender = RTPSender()
            print("RTP发送器初始化成功")
        except Exception as e:
            print(f"RTP发送器初始化失败: {e}")
            rtp_sender = None
    else:
        print("跳过RTP发送器初始化（摄像头不可用）")
        rtp_sender = None
    
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
        
        # 等待串口稳定
        time.sleep(2)
        
        # 开机强制同步手动模式到STM32和云端
        print("="*50)
        print("开机同步：强制设置为手动模式（auto=1）")
        print("="*50)
        
        # 发送3次手动模式命令给STM32，确保生效
        for i in range(3):
            mode_data = {"auto": 1}
            cjson_str = json.dumps(mode_data, separators=(',', ':')) + '\n'
            ser.write(cjson_str.encode('utf-8'))
            print(f"[{i+1}/3] 发送手动模式到STM32: {mode_data}")
            time.sleep(0.2)
        
        # 同步上报手动模式到云端，覆盖设备影子
        if mqtt_client_instance:
            now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            for i in range(3):
                msg = {
                    "services": [
                        {
                            "serviceId": "xianlan",
                            "properties": {"auto": 1},
                            "event_time": now
                        }
                    ]
                }
                json_msg = json.dumps(msg)
                result = mqtt_client_instance.publish(MQTT_TOPIC, json_msg)
                if result.rc == mqtt_client.MQTT_ERR_SUCCESS:
                    print(f"[{i+1}/3] 同步手动模式到云端成功")
                else:
                    print(f"[{i+1}/3] 同步手动模式到云端失败, code: {result.rc}")
                time.sleep(0.3)
            
            # 开机强制同步icy=0到云端，覆盖设备影子中的旧结冰状态
            print("开机同步：强制设置icy=0（无结冰）")
            for i in range(3):
                msg = {
                    "services": [
                        {
                            "serviceId": "xianlan",
                            "properties": {"icy": 0},
                            "event_time": now
                        }
                    ]
                }
                json_msg = json.dumps(msg)
                result = mqtt_client_instance.publish(MQTT_TOPIC, json_msg)
                if result.rc == mqtt_client.MQTT_ERR_SUCCESS:
                    print(f"[{i+1}/3] 同步icy=0到云端成功")
                else:
                    print(f"[{i+1}/3] 同步icy=0到云端失败, code: {result.rc}")
                time.sleep(0.3)
        
        print("="*50)
        print("系统启动完成，当前为手动模式（auto=1）")
        print("如需启用自动模式，请通过网页端切换")
        print("="*50)
        
        # 8. 启动icy同步线程（每秒发送一次icy状态，在串口稳定后启动）
        print("启动icy同步线程...")
        icy_thread = threading.Thread(target=icy_sync_thread_func, daemon=True)
        icy_thread.start()
        
        # 9. 启动雨雪同步线程（每秒发送一次雨雪状态）
        print("启动雨雪同步线程...")
        rain_thread = threading.Thread(target=rain_sync_thread_func, daemon=True)
        rain_thread.start()
    
    print("=== 系统启动完成，按Ctrl+C退出 ===")
    
    # 9. 主循环
    try:
        while running:
            time.sleep(1)
    except KeyboardInterrupt:
        print("收到退出信号")
    
    # 10. 清理资源
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
