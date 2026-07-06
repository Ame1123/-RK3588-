import paho.mqtt.client as mqtt
import base64
import hmac
import os
import time
from urllib.parse import quote

ServerUrl = "mqtts.heclouds.com"  # 服务器url
ServerPort = 1883  # 服务器端口
DeviceName = "mmp"  # 设备
Productid = "YuQ5UxO21C"  # 产品ID
accesskey = os.environ.get("ONENET_ACCESS_KEY", "")

# 发布的topic
Pub_topic1 = "$sys/" + Productid + "/" + DeviceName + "/thing/property/post"

# 需要订阅的topic
# 数据上传成功的消息
Sub_topic1 = "$sys/" + Productid + "/" + DeviceName + "/thing/property/post/reply"
# 接收数据上传失败的消息
Sub_topic2 = "$sys/" + Productid + "/" + DeviceName + "/thing/property/set"

# 测试用json数据格式
#jsonstr = "{\"id\":\"123\",\"params\":{\"temp\":{\"value\":24.6},\"humi\":{\"value\":76.1}}}"

class MyMQTTClient(mqtt.Client):
    def __init__(self):
        super().__init__()
        self._callback_api_version = mqtt.CallbackAPIVersion.VERSION2


    # 认证token生成函数
    def get_token(self, access_key):
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


    def on_subscribe(self, client, userdata, mid, reason_code_list, properties):
        # Since we subscribed only for a single channel, reason_code_list contains
        # a single entry
        print("jjjjjjjjj")
        if reason_code_list[0].is_failure:
            print(f"Broker rejected you subscription: {reason_code_list[0]}")
        else:
            print(f"Broker granted the following QoS: {reason_code_list[0].value}")


    def on_unsubscribe(self, client, userdata, mid, reason_code_list, properties):
        # Be careful, the reason_code_list is only present in MQTTv5.
        # In MQTTv3 it will always be empty
        if len(reason_code_list) == 0 or not reason_code_list[0].is_failure:
            print("unsubscribe succeeded (if SUBACK is received in MQTTv3 it success)")
        else:
            print(f"Broker replied with failure: {reason_code_list[0]}")
        client.disconnect()


    # 当客户端收到来自服务器的CONNACK响应时的回调。也就是申请连接，服务器返回结果是否成功等
    def on_connect(self, client, userdata, flags, rc, properties):
        print(f"Connected with result code {rc}")
        # 在这里可以添加连接成功后的逻辑，比如订阅主题
        client.subscribe(Sub_topic1)


    # 从服务器接收发布消息时的回调。
    def on_message(self, client,  userdata, message):
        print(str(message.payload, 'utf-8'))


    # 当消息已经被发送给中间人，on_publish()回调将会被触发
    def on_publish(self, client, userdata, mid):
        print(str(mid))

    # def oneNetMqttInIt(self):
    #     passw = self.get_token(accesskey)
    #     self.mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, DeviceName)
    #     self.mqttc.on_connect = self.on_connect
    #     self.mqttc.on_message = self.on_message
    #     self.mqttc.on_subscribe = self.on_subscribe
    #     self.mqttc.connect(ServerUrl, port=ServerPort, keepalive=120)
    #     self.mqttc.username_pw_set(Productid, passw)
    #     self.mqttc.loop_start()
    #
    # def oneNetMqttSend(self,jsonstr):
    #     self.mqttc.publish(Pub_topic1, jsonstr, qos=0,retain=False)
    #     time.sleep(2)

#

# 创建MyMQTTClient的实例
my_client = MyMQTTClient()
passw = my_client.get_token(accesskey)
# 设置MQTT代理的地址和端口
my_client.connect(ServerUrl, port=ServerPort, keepalive=120)
my_client.username_pw_set(Productid, passw)
# 启动MQTT客户端的网络循环
# 这会阻塞当前线程，所以通常会在一个单独的线程中运行
# 或者使用my_client.loop_start()来非阻塞地启动
my_client.loop_forever()

# onenet = OneNet()
# onenet.oneNetMqttInIt() #\"GeoLocation\":{\"value\":{[\"latitude\":27.8187116667,\"longitude\":113.094379000,\"altitude\":500]}}

while True:
#     #jsonstr = "{\"id\":\"123\",\"params\":{\"gps\":{\"value\":{\"lat\":27.8187116667,\"lon\":113.094379000}},\"humi\":{\"value\":76.1}}}"
#    jsonstr = "{\"id\":\"123\",\"params\":{\"GeoLocation\":{\"value\":{\"latitude\":29.57,\"longitude\":106.54}},\"humi\":{\"value\":76.1}}}"
# print(onenet.get_token(accesskey))
# temp = 23.3
# humi = 26.6
# while True:
#     #jsonstr = "{\"id\":\"123\",\"params\":{\"temp\":{\"value\":" + str(temp) + "},\"humi\":{\"value\":" + str(humi) + "}}}",\"babycar\":{\"value\":"+ str('姿态正常') +"},\"kunchong\":{\"value\":"+ str('无')
#     jsonstr = ("{\"id\":\"123\",\"params\":{\"temp\":{\"value\":"
#                                         +str(18)+
#                                         "},\"humi\":{\"value\":"+
#                                         str(19)+
#                                         "},\"babyface\":{\"value\":\""+ str('婴儿平静')+
#                                         "\"},\"babycar\":{\"value\":\""+ str('姿态正常') +
#                                         "\"},\"kunchong\":{\"value\":\""+ str('无') +"\"}}}")

    jsonstr = "{\"id\":\"123\",\"params\":{\"temp\":{\"value\":24.6},\"humi\":{\"value\":76.1}}}"
    my_client.publish(Pub_topic1, jsonstr)
    time.sleep(3)
#{"id":"123","params":{"temp":{"value":17},"humi":{"value":17},"oldface":{"value":"--"},"oldcar":{"value":"姿态异常"},"kuncong":{"value":"--"},"weizhi":{"value":"oldman"},"chaosheng":{"value":"前方安全"}}}
