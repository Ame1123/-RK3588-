import cv2
from ultralytics import YOLO
import os
import time
import socket
import struct
import numpy as np

class RTPSender:
    def __init__(self, host='47.122.26.175', port=5004):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dest_addr = (host, port)
        self.sequence = 0
        self.ssrc = 0x12345678
        
    def send_frame(self, frame):
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ret: return
        
        header = bytearray(12)
        header[0] = 0x80  # RTP版本和标记位
        header[1] = 0x60  # 负载类型
        struct.pack_into('>H', header, 2, self.sequence % 65536)  # 序列号
        struct.pack_into('>I', header, 4, int(time.time()))  # 时间戳
        struct.pack_into('>I', header, 8, self.ssrc)  # 同步源标识
        
        max_pkt_size = 60000
        for i in range(0, len(buffer), max_pkt_size):
            chunk = bytes(buffer[i:i+max_pkt_size])  # 显式转换为bytes
            self.sock.sendto(header + chunk, self.dest_addr)  # 字节流拼接
        self.sequence += 1

def load_model(model_path):
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model file {model_path} not found")
    model = YOLO(model_path)
    onnx_path = os.path.splitext(model_path)[0] + '.onnx'
    if not os.path.exists(onnx_path):
        model.export(format="onnx", opset=12, dynamic=True, simplify=True)
    return model

def main():
    model = load_model('models/best.pt')
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    
    rtp_sender = RTPSender()
    
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret: break
        
        results = model(frame)
        annotated_frame = results[0].plot()
        rtp_sender.send_frame(annotated_frame)
        
        #cv2.imshow("YOLOv8 RTP Sender", annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()   
    
    
    
