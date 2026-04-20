from ultralytics import YOLO
#这里装载总包
import torch
import torchvision
#这里装载图片打开及处理
from PIL import Image
import os
import json
import shutil
import torchvision.transforms as transforms
#import torch_snippets   #需要用的时候再解封
#from torch_snippets import *
#这里装载模型构建及其骨架
import torchvision.models as models
import torch.nn as nn
import torch.nn.functional as F
#这里装载优化器及数学工具
import torch.optim as optim
import numpy as np
import pandas as pd
import random
#这里装载数据集
import torch.utils.data as data
#定义一下工作区域迁移
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

def create_yolo_dataset():
    # 定义路径
    source_folder = 'D:\\anaconda\\envs\\Shining\\shin\\widdget_code\\WindowsNoEditor\\try2\\Saved\\BugIt\\WindowsNoEditor_Cropped'
    dataset_folder = 'D:\\anaconda\\envs\\Shining\\shin\\control_code\\YOLO_try\\dataset'
    
    # 创建目标文件夹结构
    splits = ['train', 'val', 'test']
    for split in splits:
        os.makedirs(os.path.join(dataset_folder, split, 'images'), exist_ok=True)
        os.makedirs(os.path.join(dataset_folder, split, 'labels'), exist_ok=True)
    
    # 获取所有图片文件
    image_files = []
    for file in os.listdir(source_folder):
        if file.endswith('.png') and '_' in file:
            # 提取基础文件名（如4_00003）
            parts = file.split('_')
            if len(parts) >= 2:
                base_name = '_'.join(parts[:2])
                image_files.append(base_name)
    
    # 去重
    image_files = list(set(image_files))
    print(f"找到 {len(image_files)} 个基础图片文件")
    
    # 随机打乱并按比例分配
    random.shuffle(image_files)
    total = len(image_files)
    train_end = int(0.6 * total)
    val_end = int(0.8 * total)
    
    train_files = image_files[:train_end]
    val_files = image_files[train_end:val_end]
    test_files = image_files[val_end:]
    
    print(f"训练集: {len(train_files)} 个文件")
    print(f"验证集: {len(val_files)} 个文件")
    print(f"测试集: {len(test_files)} 个文件")
    
    # 处理每个分割
    splits_data = [
        ('train', train_files),
        ('val', val_files),
        ('test', test_files)
    ]
    
    for split_name, files in splits_data:
        print(f"\n处理 {split_name} 集...")
        image_count = 0
        
        for base_name in files:
            # 处理8个视角的图片
            for camera_id in range(1, 9):
                image_file = f"{base_name}_{camera_id}.png"
                image_path = os.path.join(source_folder, image_file)
                
                # 检查图片是否存在
                if os.path.exists(image_path):
                    # 复制图片
                    dest_image_path = os.path.join(dataset_folder, split_name, 'images', image_file)
                    shutil.copy2(image_path, dest_image_path)
                    
                    # 处理对应的标签文件
                    json_file = f"{base_name}_projections.json"
                    json_path = os.path.join(source_folder, json_file)
                    
                    if os.path.exists(json_path):
                        with open(json_path, 'r') as f:
                            data = json.load(f)
                        
                        # 获取对应相机的点
                        picture_key = f"picture{camera_id}"
                        if picture_key in data:
                            points = data[picture_key]
                            
                            # 创建YOLO格式的标签文件
                            label_file = f"{base_name}_{camera_id}.txt"
                            label_path = os.path.join(dataset_folder, split_name, 'labels', label_file)
                            
                            with open(label_path, 'w') as label_f:
                                # 写入无人机类别（0）和5个关键点
                                # YOLO keypoints格式: class_id x1 y1 x2 y2 ... xn yn
                                line = "0"  # 类别ID
                                
                                # 归一化坐标 (假设图片尺寸为853x480)
                                valid_points = []
                                for point in points:
                                    if point is not None and len(point) >= 3 and point[2] > 0:  # visibility > 0
                                        valid_points.append([point[0], point[1]])
    
                                if len(valid_points) > 0:
                                    # 计算边界框
                                    points_array = np.array(valid_points)
                                    x_min, y_min = points_array.min(axis=0)
                                    x_max, y_max = points_array.max(axis=0)
        
                                    # 边界框中心点和宽高（归一化）
                                    cx = ((x_min + x_max) / 2) / 853.0
                                    cy = ((y_min + y_max) / 2) / 480.0
                                    width = (x_max - x_min) / 853.0
                                    height = (y_max - y_min) / 480.0
        
                                    # 构建标签行
                                    line = f"0 {cx:.6f} {cy:.6f} {width:.6f} {height:.6f}"  # 类别 + 边界框
        
                                    # 添加关键点信息
                                    for point in points:
                                        if point is not None and len(point) >= 3:
                                            x_norm = point[0] / 853.0
                                            y_norm = point[1] / 480.0
                                            visibility = point[2]
                                            line += f" {x_norm:.6f} {y_norm:.6f} {visibility}"
                                        else:
                                            line += " 0.0 0.0 0"  # 对于缺失点使用0,0,0（未标记）
                                else:
                                    # 如果没有有效点，创建一个默认的小边界框
                                    line = f"0 0.5 0.5 0.1 0.1"  # 类别 + 默认边界框
                                # 添加5个默认关键点
                                    for _ in range(5):
                                        line += " 0.0 0.0 0"
    
                                label_f.write(line)
                    
                    image_count += 1
        
        print(f"  复制了 {image_count} 张图片到 {split_name} 集")
    
    print("\n数据集创建完成！")
    print(f"数据集位置: {dataset_folder}")

# 运行函数
if __name__ == "__main__":
    create_yolo_dataset()
