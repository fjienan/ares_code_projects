import cv2
from pyk4a import PyK4A, Config, ColorResolution
import torch
from ultralytics import YOLO

# 1. 初始化Azure Kinect
k4a = PyK4A(Config(color_resolution=ColorResolution.RES_1080P, depth_mode=None))
k4a.start()

# 2. 加载YOLOv11模型（这里用ultralytics的YOLOv8做演示，实际用YOLOv11请替换模型加载部分）
# 如果你有yolov11.pt权重文件，替换下面的路径
model = YOLO('~/Desktop/ultralytics/runs/detect/train17/weights/best.pt')
while True:
    capture = k4a.get_capture()
    if capture.color is not None:
        img = capture.color
        # BGR转RGB
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # 3. 推理
        results = model(img_rgb)
        # 4. 可视化
        results.render()  # 直接在img_rgb上画框
        img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
        cv2.imshow('YOLOv11 Detection', img_bgr)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

k4a.stop()
cv2.destroyAllWindows()