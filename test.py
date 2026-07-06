import cv2
from ultralytics import YOLO
import os
import time

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
    
    # 设置摄像头分辨率320x240
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    
    if not cap.isOpened():
        print("Error: Could not open camera")
        return

    # FPS计算变量
    prev_time = 0
    curr_time = 0
    
    while cap.isOpened():
        success, frame = cap.read()
        if not success: break
            
        results = model(frame)
        annotated_frame = results[0].plot()
        
        # 精确计算FPS
        curr_time = time.time()
        fps = 1 / (curr_time - prev_time)
        prev_time = curr_time
        
        cv2.putText(annotated_frame, f"FPS: {int(fps)}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow("YOLOv8 Inference", annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord("q"): break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

