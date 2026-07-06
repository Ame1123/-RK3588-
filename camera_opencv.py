import time

import cv2
from ultralytics import YOLO
from base_camera import BaseCamera

class Camera(BaseCamera):
    video_source = 0
    # video_source = "nvarguscamerasrc \
    # !video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1\
    # !nvvidconv flip-method=0 ! videoconvert ! video/x-raw, format=BGR ! appsink"

    @staticmethod
    def set_video_source(source):
        Camera.video_source = source

    @staticmethod
    def frames():
        model = YOLO('model/best.engine', task='detect')
        camera = cv2.VideoCapture(Camera.video_source)
        conf = 0.25  # 设置检测置信度值
        iou = 0.7  # 设置检测IOU值
        time.sleep(2)
        if not camera.isOpened():
            raise RuntimeError('Could not start camera.')

        while True:
            # read current frame
           # _, img = camera.read()

            ret, now_img = camera.read()
            # 摄像头分辨率
            # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
            # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
            if ret:
                results = model(now_img, conf=conf, iou=iou)[0]

                conf_list = results.boxes.conf.tolist()
                conf_list = ['%.2f %%' % (each * 100) for each in conf_list]
                cls_list = results.boxes.cls.tolist()
                cls_list = [int(i) for i in cls_list]
                resize_cvimg = cv2.resize(results.plot(), (400, 400))
            # encode as a jpeg image and return it
                yield cv2.imencode('.jpg', resize_cvimg)[1].tobytes()
