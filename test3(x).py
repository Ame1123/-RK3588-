import cv2
import numpy as np
import socket
from collections import deque

class FrameReceiver:
    def __init__(self, port=5004):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', port))
        self.frame_packets = deque(maxlen=30)
        self.last_seq = -1

    def _process_packet(self, data):
        seq_num = int.from_bytes(data[2:4], 'big')
        marker = (data[0] & 0x80) >> 7
        
        if b'FRAME_START' in data:
            self.frame_packets.clear()
            return False
            
        if b'FRAME_END' in data:
            frame_data = b''.join(self.frame_packets)
            self.frame_packets.clear()
            return cv2.imdecode(np.frombuffer(frame_data, np.uint8), 1)
            
        if seq_num > self.last_seq:
            payload = data[12:]
            self.frame_packets.append(payload)
            self.last_seq = seq_num
            
        return False

    def receive_frame(self, timeout=1):
        self.sock.settimeout(timeout)
        try:
            while True:
                data, _ = self.sock.recvfrom(65535)
                frame = self._process_packet(data)
                if frame is not False:
                    return frame
        except socket.timeout:
            return None

if __name__ == "__main__":
    receiver = FrameReceiver()
    while True:
        frame = receiver.receive_frame()
        if frame is not None:
            cv2.imshow("Received Frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    cv2.destroyAllWindows()

