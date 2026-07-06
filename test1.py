import threading

import paho.mqtt.client as mqtt
from paho.mqtt.client import MQTTv311
import struct
import json
import base64
import hmac
import os
import time
from urllib.parse import quote
import serial

ServerUrl = "mqtts.heclouds.com"  # 服务器url
ServerPort = 1883  # 服务器端口
DeviceName = "mmp"  # 设备
Productid = "YuQ5UxO21C"  # 产品ID
accesskey = os.environ.get("ONENET_ACCESS_KEY", "")

# 发布的topic
Pub_topic1 = "$sys/" + Productid + "/" + DeviceName + "/thing/property/post"
mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, DeviceName)
#ser = serial.Serial('/dev/ttyTHS1', 9600, timeout=1)
# 需要订阅的topic
# 数据上传成功的消息
Sub_topic1 = "$sys/" + Productid + "/" + DeviceName + "/thing/property/post/reply"
# 接收数据上传失败的消息
Sub_topic2 = "$sys/" + Productid + "/" + DeviceName + "/thing/property/set"

Sub_topic3 = "$sys/" + Productid + "/" + DeviceName + "/thing/property/set_reply"

Sub_topic4 = "$sys/" + Productid + "/" + DeviceName + "/cmd/#"
# 测试用json数据格式
jsonstr = "{\"id\":\"123\",\"params\":{\"temp\":{\"value\":24.6},\"humi\":{\"value\":76.1}}}"


# 认证token生成函数
def get_token(id, access_key):
    version = '2018-10-31'
    #   res = 'products/%s' % id  # 通过产品ID访问产品API
    # res = 'userid/%s' % id  # 通过产品ID访问产品API
    res = "products/" + Productid + "/devices/" + DeviceName
    # 用户自定义token过期时间
    et = str(int(time.time()) + 36000000)
    # et = str(int(1722499200))
    # 签名方法，支持md5、sha1、sha256
    method = 'sha1'
    method1 = 'sha256'
    # 对access_key进行decode
    key = base64.b64decode(access_key)

    # 计算sign
    org = et + '\n' + method + '\n' + res + '\n' + version
    sign_b = hmac.new(key=key, msg=org.encode(), digestmod=method)
    sign = base64.b64encode(sign_b.digest()).decode()

    # value 部分进行url编码，method/res/version值较为简单无需编码
    sign = quote(sign, safe='')
    res = quote(res, safe='')

    # token参数拼接
    token = 'version=%s&res=%s&et=%s&method=%s&sign=%s' % (version, res, et, method, sign)

    return token


def on_subscribe(client, userdata, mid, reason_code_list, properties):
    # Since we subscribed only for a single channel, reason_code_list contains
    # a single entry
    if reason_code_list[0].is_failure:
        print(f"Broker rejected you subscription: {reason_code_list[0]}")
    else:
        print(f"Broker granted the following QoS: {reason_code_list[0].value}")


def on_unsubscribe(client, userdata, mid, reason_code_list, properties):
    # Be careful, the reason_code_list is only present in MQTTv5.
    # In MQTTv3 it will always be empty
    if len(reason_code_list) == 0 or not reason_code_list[0].is_failure:
        print("unsubscribe succeeded (if SUBACK is received in MQTTv3 it success)")
    else:
        print(f"Broker replied with failure: {reason_code_list[0]}")
    client.disconnect()


# 当客户端收到来自服务器的CONNACK响应时的回调。也就是申请连接，服务器返回结果是否成功等
def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code.is_failure:
        print(f"Failed to connect: {reason_code}. loop_forever() will retry connection")
    else:
        # we should always subscribe from on_connect callback to be sure
        # our subscribed is persisted across reconnections.
        # client.subscribe("$SYS/#")
        print("连接结果:" + mqtt.connack_string(reason_code))
        # 连接成功后就订阅topic
        #client.subscribe(Sub_topic1)
        client.subscribe(Sub_topic2)
        client.subscribe(Sub_topic4)


# 从服务器接收发布消息时的回调。
def on_message(client, userdata, message):
    print('kkkkk'+str(message.payload, 'utf-8'))
    messages = json.loads(message.payload)
    #print(messages.get('id'))
    try:
        if messages.get('params'):
            print(type(messages.get('params')))
            if messages.get('params').get('moshi'):
                print(type(messages.get('params').get('moshi')))
                if messages.get('params').get('moshi'):
                    ser.write(1)
                else:
                    ser.write(0)
            mystr = "{\"id\":\""+ str(messages.get('id')) +"\",\"code\":200,\"msg\":\"success\"}"
            #mystr = "{\"id\":\"30\",\"code\":200,\"msg\":\"success\"}"
            client.publish(Sub_topic3, mystr, qos=0)
    except Exception as e:
        print(e)


# 当消息已经被发送给中间人，on_publish()回调将会被触发
def on_publish(client, userdata, mid):
    print("ssss"+str(mid))

def readSerial():
    print("readSerial")
    if ser.is_open:
        while True:
            # 读取串口数据
            if ser.in_waiting > 0:
                try:
                    params = ser.readline().decode('utf-8').rstrip()
                    params = json.loads(params)
                    print(params)
                except Exception as e:
                    print(" faile json" + str(e))

def main():
    passw = get_token(DeviceName, accesskey)
    print(passw)

    mqttc.on_connect = on_connect
    mqttc.on_message = on_message
    mqttc.on_subscribe = on_subscribe
    mqttc.on_unsubscribe = on_unsubscribe

    # client = mqtt.Client(DeviceName,protocol=MQTTv311)
    # client.tls_set(certfile='/Users/mryu/PycharmProjects/MyProject/onenet/MQTTS-certificate.pem') #鉴权证书
    mqttc.connect(ServerUrl, port=ServerPort, keepalive=120)

    mqttc.username_pw_set(Productid, passw)

    mqttc.loop_start()
    # serialth = threading.Thread(target=readSerial)
    # serialth.daemon = True
    # serialth.start()

    while (1):
        mqttc.publish(Pub_topic1, jsonstr, qos=0)
        #print("okk")
        time.sleep(3)


if __name__ == '__main__':
    main()
