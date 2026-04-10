from ultralytics import YOLO
import os

path = 'D:\\anaconda\\envs\\Shining\\shin\\widdget_code\\WindowsNoEditor\\大.pt'

# 加载模型
model = YOLO(path)

# 导出为ONNX格式
model.export(format='onnx', imgsz=640, dynamic=True)