import os
import sys
import airsim
import subprocess
import random
import threading
import json
import glob
import cv2
import numpy as np
import win32gui
from ctypes import wintypes
import ctypes
import win32ui
import time 
import torch
import math
import glob
import csv
from ultralytics import YOLO
from PySide6.QtCore import (QCoreApplication,
    QMetaObject, QObject, Slot,
    QSize, Qt, QThread, Signal)
from PySide6.QtGui import (QImage,QPixmap)
from PySide6.QtWidgets import (QCheckBox, QFormLayout, QHBoxLayout,
    QLabel, QPushButton, QSizePolicy, QSlider, QMessageBox, QFrame,
    QSpacerItem, QTabWidget, QTextEdit, QVBoxLayout, QFileDialog,
    QWidget, QMainWindow, QLineEdit, QGridLayout, QWidget,
    QRadioButton)
import matplotlib
matplotlib.use('QtAgg')
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from matplotlib import font_manager
from global_import import ( X_MAX, Y_MAX, Z_MAX, l, angle, tip, 
    speed, picture_split, width, height, K, rotor_position, pt_name, plot_real_data, 
    plot_yolo_data, plot_EKF_data, R_diag_global, q_a_global, q_alpha_global, 
     manual_delay, plot_finial_data, r_pixel, manual_delay_1,
    keyboard_listener,
    create_record_file, create_trigger_file, generate_random_waypoint, create_camera_trigger_file
    , calculate_camera_position_bias,
    RigidBodyEKF, TightlyCoupledEKF)
import global_import
import warnings
warnings.filterwarnings('ignore', category=UserWarning, module='torch.nn.modules.conv')
from scipy import linalg
import traceback
import shutil
import pyqtgraph.opengl as gl

# 创建一个信号类用于线程间通信
class StreamSignal(QObject):
    text_written = Signal(str)

# 创建一个重定向类
class StreamRedirector:
    def __init__(self, signal):
        self.signal = signal

    def write(self, text):
        if text.strip():  # 只发送非空文本
            self.signal.text_written.emit(text)

    def flush(self):
        pass  # 必须实现flush方法

class DataCollectionThread(QThread):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.running = True
        self.stop_event = threading.Event()
        
    def run(self):
        try:
            # 启动键盘监听线程
            keyboard_thread = threading.Thread(target=keyboard_listener, args=(self.stop_event,), daemon=True)
            keyboard_thread.start()
            self.collect_data_impl()
        except Exception as e:
            print(f"数据收集过程中出现错误: {e}")
        finally:
            print("数据收集任务已完成")
    
    def stop(self):
        self.running = False
        self.stop_event.set()
        
    def collect_data_impl(self):
        print("开始收集数据,控制端已锁死")
        record_count = 0

        client = airsim.MultirotorClient()
        try:
            client.enableApiControl(True)
        except Exception as e:
            print(f"仿真端疑似未打开: {e}")
            return
        
        client.armDisarm(True)
        client.takeoffAsync().join()
        
        def get_drone_info():
            '''获取无人机详细信息'''
            state = client.getMultirotorState()
            position = state.kinematics_estimated.position
            velocity = state.kinematics_estimated.linear_velocity
            orientation = state.kinematics_estimated.orientation
    
            # 计算欧拉角（俯仰、偏航、翻滚）
            pitch, roll, yaw = airsim.to_eularian_angles(orientation)
    
            return {
                "position": {
                    "x": round(position.x_val, 3),
                    "y": round(position.y_val, 3),
                    "z": round(position.z_val, 3)
                },
                "velocity": {
                    "x": round(velocity.x_val, 3),
                    "y": round(velocity.y_val, 3),
                    "z": round(velocity.z_val, 3)
                },
                "orientation": {
                   "pitch": round(pitch, 3),
                    "roll": round(roll, 3),
                    "yaw": round(yaw, 3)
                }
            }
        
        # 飞行和触发拍照
        print("上升到3m高度...")
        #client.moveToZAsync(-3, 1).join()

        print("开始在随机航点间飞行并记录...")

        # 每隔tip秒记录一次，并飞向新的随机位置
        while self.running and not self.stop_event.is_set():
            # 生成随机航点
            target_x, target_y, target_z = generate_random_waypoint()
    
            # 飞向随机位置（速度设为speedm/s）
            print(f"飞往随机位置: ({target_x:.1f}, {target_y:.1f}, {target_z:.1f})")
            client.moveToPositionAsync(target_x, target_y, target_z, speed, yaw_mode=airsim.YawMode(False, random.uniform(-180, 180)))
    
            # 获取无人机信息并创建触发文件
            drone_info = get_drone_info() 
            record_name = f"{record_count}"
            print(f"记录 {record_name}, 位置: {drone_info['position']}, "
                f"俯仰: {drone_info['orientation']['pitch']:.2f}, "
                f"偏航: {drone_info['orientation']['yaw']:.2f},"
                f"翻滚: {drone_info['orientation']['roll']:.2f}")
            create_trigger_file(record_name)
            create_record_file(record_name, drone_info)
    
            record_count += 1
    
            # 等待tip秒或直到停止
            if self.stop_event.wait(tip):
                break

        # 取消当前飞行任务
        client.hoverAsync().join()

        # 降落
        print("降落...")
        client.landAsync().join()
        client.armDisarm(False)
        client.enableApiControl(False)
        print("任务完成")
    # 数据收集线程类
# 数据采集线程

class fly(QThread):
    #pose_data_ready = Signal(dict, int)  # 位姿数据, record_count
    #real_time_data_ready = Signal(dict)

    def __init__(self, parent=None, flight_mode = None):
        super().__init__(parent)
        self.running = True
        self.stop_event = threading.Event()
        self.record_count = 0
        self.flight_mode = flight_mode
        
    def run(self):
        try:
            # 启动键盘监听线程
            keyboard_thread = threading.Thread(target=keyboard_listener, args=(self.stop_event,), daemon=True)
            keyboard_thread.start()
            self.collect_data_impl()
        except Exception as e:
            print(f"飞行过程中出现错误: {e}")
        finally:
            print("飞行任务已完成")
    
    def stop(self):
        self.running = False
        self.stop_event.set()
        
    def collect_data_impl(self):
        print("开始飞行,控制端已锁死")
        self.record_count = 0

        client = airsim.MultirotorClient()

        try:
            client.enableApiControl(True)
        except Exception as e:
            print(f"仿真端疑似未打开: {e}")
            return
        
        client.armDisarm(True)
        client.takeoffAsync().join()
        
        # 飞行和触发拍照
        #print("上升到3m高度...")
        #client.moveToZAsync(-3, 1).join()

        print("开始在航点间飞行并记录...")

        # 根据选择的飞行模式执行不同逻辑
        if self.flight_mode == "random_waypoint":
            self._random_waypoint_flight(client)
        elif self.flight_mode == "circle_flight":
            self._circle_flight(client)
        elif self.flight_mode == "figure_eight":
            self._figure_eight_flight(client)
        elif self.flight_mode == "hover_mode":
            self._hover_flight(client)
        elif self.flight_mode == "line_scan":
            self._square_flight(client)
        '''
        # 每隔tip秒记录一次，并飞向新的随机位置
        while self.running and not self.stop_event.is_set():
            # 生成随机航点
            target_x, target_y, target_z = generate_random_waypoint()
    
            # 飞向随机位置（速度设为speedm/s）
            print(f"飞往随机位置: ({target_x:.1f}, {target_y:.1f}, {target_z:.1f})")
            client.moveToPositionAsync(target_x, target_y, target_z, speed, yaw_mode=airsim.YawMode(False, random.uniform(-180, 180)))

            self.record_count += 1
    
            # 等待tip秒或直到停止
            if self.stop_event.wait(tip):
                break
        '''
        # 取消当前飞行任务
        client.hoverAsync().join()

        # 降落
        print("降落...")
        client.landAsync().join()
        client.armDisarm(False)
        client.enableApiControl(False)
        print("任务完成")
    #飞行模块主要部分

    def _random_waypoint_flight(self, client):
        """随机航点飞行模式"""
        while self.running and not self.stop_event.is_set():
            target_x, target_y, target_z = generate_random_waypoint()
            print(f"飞往随机位置：({target_x:.1f}, {target_y:.1f}, {target_z:.1f})")
            client.moveToPositionAsync(target_x, target_y, target_z, speed, 
                                    yaw_mode=airsim.YawMode(False, random.uniform(-180, 180)))

            self.record_count += 1
            
            if self.stop_event.wait(tip):
                break

    def _figure_eight_flight(self, client, speed_factor=2.0):
        """
        执行无限循环的8字形轨迹飞行 (LQR控制)
        - 中心位于坐标原点 (0,0)
        - 机头朝向完美跟随飞行方向 (已修正反向)
        - 隐藏地面参考轨迹
        :param speed_factor: 速度倍率 (例如 2.0 表示两倍速)
        """
        # --- 1. 轨迹生成函数 (优化版：自动计算速度和加速度) ---
        def create_trajectory(speed_scale):
            base_steps = 400
            steps_per_segment = max(int(base_steps / speed_scale), 50)
            total_steps = steps_per_segment * 4
            radius = 2.0
            
            # 1.1 生成位置轨迹
            p_traj = np.zeros((2, total_steps))
            
            center_left = -radius
            center_right = radius

            # Segment 1: 右侧圆上半 (左->右)
            for i in range(steps_per_segment):
                theta = math.pi - math.pi / steps_per_segment * i
                p_traj[0, i] = radius * math.cos(theta) + center_left
                p_traj[1, i] = radius * math.sin(theta)

            # Segment 2: 左侧圆下半 (左->右)
            for i in range(steps_per_segment):
                theta = math.pi + math.pi / steps_per_segment * i
                p_traj[0, i + steps_per_segment] = radius * math.cos(theta) + center_right
                p_traj[1, i + steps_per_segment] = radius * math.sin(theta)

            # Segment 3: 左侧圆上半 (右->左)
            for i in range(steps_per_segment):
                theta = math.pi / steps_per_segment * i
                p_traj[0, i + 2*steps_per_segment] = radius * math.cos(theta) + center_right
                p_traj[1, i + 2*steps_per_segment] = radius * math.sin(theta)

            # Segment 4: 右侧圆下半 (右->左)
            for i in range(steps_per_segment):
                theta = 2 * math.pi - math.pi / steps_per_segment * i
                p_traj[0, i + 3*steps_per_segment] = radius * math.cos(theta) + center_left
                p_traj[1, i + 3*steps_per_segment] = radius * math.sin(theta)

            # 1.2 根据位置变化自动计算速度和加速度 (消除推导误差)
            dt = 0.02
            v_traj = np.zeros_like(p_traj)
            a_traj = np.zeros_like(p_traj)
            
            # 计算速度 (中心差分法)
            # v(t) = (p(t+1) - p(t-1)) / 2dt
            for i in range(total_steps):
                prev_idx = (i - 1 + total_steps) % total_steps
                next_idx = (i + 1) % total_steps
                
                # 速度
                v_traj[:, i] = (p_traj[:, next_idx] - p_traj[:, prev_idx]) / (2 * dt)
                
            # 计算加速度 (中心差分法)
            # a(t) = (v(t+1) - v(t-1)) / 2dt
            for i in range(total_steps):
                prev_idx = (i - 1 + total_steps) % total_steps
                next_idx = (i + 1) % total_steps
                
                # 加速度
                a_traj[:, i] = (v_traj[:, next_idx] - v_traj[:, prev_idx]) / (2 * dt)
                
            return p_traj, v_traj, a_traj

        # --- 2. LQR求解函数 ---
        def solve_dlqr(A, B, Q, R):
            S = np.matrix(linalg.solve_discrete_are(A, B, Q, R))
            K = np.matrix(linalg.inv(B.T * S * B + R) * (B.T * S * A))
            return K

        # --- 3. 控制量转换函数 ---
        def move_by_acceleration_horizontal(client, ax_cmd, ay_cmd, z_cmd, yaw_cmd, duration):
            state = client.simGetGroundTruthKinematics()
            angles = airsim.to_eularian_angles(state.orientation)
            yaw_my = angles[2]
            g = 9.8
            
            sin_yaw = math.sin(yaw_my)
            cos_yaw = math.cos(yaw_my)
            A_psi = np.array([[sin_yaw, cos_yaw], [-cos_yaw, sin_yaw]])
            A_psi_inverse = np.linalg.inv(A_psi)
            
            angle_h_cmd = 1/g * np.dot(A_psi_inverse, np.array([[-ax_cmd], [-ay_cmd]]))
            a_x_cmd = math.atan(angle_h_cmd[0, 0])
            a_y_cmd = -math.atan(angle_h_cmd[1, 0])
            
            client.moveByRollPitchYawZAsync(a_x_cmd, a_y_cmd, yaw_cmd, z_cmd, duration)

        # ================= 主逻辑开始 =================
        print(f"Initializing Infinite Figure-8 Flight (Speed x{speed_factor})...")
        
        dt = 0.02
        z_target = -3.0
        
        # 1. 生成轨迹 (自动计算导数，确保方向正确)
        p_traj, v_traj, a_traj = create_trajectory(speed_factor)
        total_steps = p_traj.shape[1]
        
        # 2. 计算LQR
        A = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        B = np.array([[0, 0],
                      [0, 0],
                      [dt, 0],
                      [0, dt]])
        Q = np.diag([2, 2, 2, 2])
        R = np.diag([.1, .1])
        K = solve_dlqr(A, B, Q, R)

        # 3. 预先飞到起点并对准方向
        print(f"移动至起始点。。。")
        client.moveToPositionAsync(0, 0, z_target, 2).join()
        
        # 计算初始偏航：使用自动生成的速度方向
        init_yaw = math.atan2(v_traj[1, 0], v_traj[0, 0])
        client.rotateToYawAsync(math.degrees(init_yaw), 5).join()
        time.sleep(1)

        # 4. 【修改】不再绘制地面参考轨迹，这几行已注释
        # plot_traj = ...
        # client.simPlotLineList(plot_traj, ...)

        # 5. 开始无限循环
        t = 0
        plot_last_pos = [airsim.Vector3r(0, 0, z_target)]
        
        while self.running and not self.stop_event.is_set():
            idx = t % total_steps
            
            # 读取当前的状态
            UAV_state = client.simGetGroundTruthKinematics()
            pos_now = np.array([[UAV_state.position.x_val], [UAV_state.position.y_val], [UAV_state.position.z_val]])
            vel_now = np.array([[UAV_state.linear_velocity.x_val], [UAV_state.linear_velocity.y_val], [UAV_state.linear_velocity.z_val]])
            
            # 构建状态向量
            state_now = np.vstack((pos_now[0:2], vel_now[0:2]))
            
            # 获取当前时刻的目标状态
            state_des = np.vstack((p_traj[:, idx:idx+1], v_traj[:, idx:idx+1]))
            
            # LQR控制律
            a_cmd = -np.dot(K, state_now - state_des) + a_traj[:, idx:idx+1]
            
            # 计算目标偏航角 (使用自动生成的 v_traj，方向必然正确)
            yaw_target = math.atan2(v_traj[1, idx], v_traj[0, idx])
            
            move_by_acceleration_horizontal(client, a_cmd[0, 0], a_cmd[1, 0], z_target, yaw_target, duration=dt)
            
            # 仅绘制无人机实际飞行轨迹 (红线)
            #plot_v_start = [airsim.Vector3r(pos_now[0, 0], pos_now[1, 0], pos_now[2, 0])]
            #client.simPlotLineList(plot_last_pos + plot_v_start, color_rgba=[1.0, 0.0, 0.0, 1.0], is_persistent=True)
            #plot_last_pos = plot_v_start
            
            t += 1
            self.record_count += 1
            
            if self.stop_event.wait(dt):
                break
        
        print("飞行结束。")

    def _hover_flight(self, client):
        """定点悬停模式"""
        state = client.getMultirotorState()
        hover_x = 0
        hover_y = 0
        hover_z = -2
        
        while self.running and not self.stop_event.is_set():
            client.moveToPositionAsync(hover_x, hover_y, hover_z, speed)
            
            self.record_count += 1
            
            if self.stop_event.wait(tip):
                break

    def _line_scan_flight(self, client):
        """直线扫描飞行模式"""
        loop = 0
        speed = 4
        
        # 先上升到目标高度
        client.moveToPositionAsync(0, 0, -2, 0.5).join()
        time.sleep(1)
        
        # 定义航点和对应朝向（度）
        waypoints = [
            (2.5, 2.5, -2, 0),    # 东北方向，朝向 45°
            (2.5, -2.5, -2, -90),  # 东南方向，朝向 -45°
            (-2.5, -2.5, -2, -180),# 西南方向，朝向 -135°
            (-2.5, 2.5, -2, 90)   # 西北方向，朝向 135°
        ]
        
        while self.running and not self.stop_event.is_set():
            loop += 1
            idx = (loop - 1) % 4
            target = waypoints[idx]
            
            print(f"航点 {loop}: 飞往 ({target[0]}, {target[1]}, {target[2]}), 朝向 {target[3]}°")
            
            client.moveToPositionAsync(
                target[0], target[1], target[2], speed,
                yaw_mode=airsim.YawMode(False, target[3])  # 设置固定朝向
            ).join()
            
            if self.stop_event.wait(tip):
                break

    def _circle_flight(self, client, radius=2.0, speed_factor=1.0):
        """
        执行无限循环的圆形轨迹飞行
        - 中心位于坐标原点 (0,0)
        - 半径由 radius 参数指定
        - 机头朝向完美跟随飞行方向
        - 隐藏地面参考轨迹，仅绘制实际飞行轨迹
        :param radius: 飞行半径 (米)
        :param speed_factor: 速度倍率 (例如 2.0 表示两倍速，通过减少轨迹点数实现)
        """
        # --- 1. 轨迹生成函数 (圆形版) ---
        def create_trajectory(r, scale):
            # 基础步数：一圈 400 个点
            base_steps = 400
            # 速度越快，步数越少，在固定dt下飞行速度变快
            steps = max(int(base_steps / scale), 50) 
            dt = 0.02
            
            # 1.1 生成位置轨迹
            p_traj = np.zeros((2, steps))
            
            # 生成圆形：x = r*cos(theta), y = r*sin(theta)
            for i in range(steps):
                theta = 2 * math.pi * i / steps
                p_traj[0, i] = r * math.cos(theta) # X轴 (North)
                p_traj[1, i] = r * math.sin(theta) # Y轴 (East)

            # 1.2 根据位置变化自动计算速度和加速度 (中心差分法)
            v_traj = np.zeros_like(p_traj)
            a_traj = np.zeros_like(p_traj)
            
            # 计算速度
            for i in range(steps):
                prev_idx = (i - 1 + steps) % steps
                next_idx = (i + 1) % steps
                v_traj[:, i] = (p_traj[:, next_idx] - p_traj[:, prev_idx]) / (2 * dt)
                
            # 计算加速度
            for i in range(steps):
                prev_idx = (i - 1 + steps) % steps
                next_idx = (i + 1) % steps
                a_traj[:, i] = (v_traj[:, next_idx] - v_traj[:, prev_idx]) / (2 * dt)
                
            return p_traj, v_traj, a_traj

        # --- 2. LQR求解函数 ---
        def solve_dlqr(A, B, Q, R):
            S = np.matrix(linalg.solve_discrete_are(A, B, Q, R))
            K = np.matrix(linalg.inv(B.T * S * B + R) * (B.T * S * A))
            return K

        # --- 3. 控制量转换函数 (保持不变) ---
        def move_by_acceleration_horizontal(client, ax_cmd, ay_cmd, z_cmd, yaw_cmd, duration):
            state = client.simGetGroundTruthKinematics()
            angles = airsim.to_eularian_angles(state.orientation)
            yaw_my = angles[2]
            g = 9.8
            
            sin_yaw = math.sin(yaw_my)
            cos_yaw = math.cos(yaw_my)
            A_psi = np.array([[sin_yaw, cos_yaw], [-cos_yaw, sin_yaw]])
            A_psi_inverse = np.linalg.inv(A_psi)
            
            angle_h_cmd = 1/g * np.dot(A_psi_inverse, np.array([[-ax_cmd], [-ay_cmd]]))
            a_x_cmd = math.atan(angle_h_cmd[0, 0])
            a_y_cmd = -math.atan(angle_h_cmd[1, 0])
            
            client.moveByRollPitchYawZAsync(a_x_cmd, a_y_cmd, yaw_cmd, z_cmd, duration)

        # ================= 主逻辑开始 =================
        print(f"开始圆形飞行(半径 {radius}m, 速度 x{speed_factor})...")
        
        dt = 0.02
        z_target = -3.0
        
        # 1. 生成轨迹
        p_traj, v_traj, a_traj = create_trajectory(radius, speed_factor)
        total_steps = p_traj.shape[1]
        
        # 2. 计算LQR参数
        A = np.array([[1, 0, dt, 0],
                    [0, 1, 0, dt],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
        B = np.array([[0, 0],
                    [0, 0],
                    [dt, 0],
                    [0, dt]])
        Q = np.diag([2, 2, 2, 2])
        R = np.diag([.1, .1])
        K = solve_dlqr(A, B, Q, R)

        # 3. 预先飞到起点并对准方向
        # 起点为圆的最右侧点 (radius, 0)
        start_x = p_traj[0, 0]
        start_y = p_traj[1, 0]
        print(f"移动至起始点 ({start_x:.1f}, {start_y:.1f})...")
        client.moveToPositionAsync(start_x, start_y, z_target, 2).join()
        
        # 计算初始偏航：圆形轨迹在起点的切线方向 (y正方向，即正东)
        init_yaw = math.atan2(v_traj[1, 0], v_traj[0, 0])
        client.rotateToYawAsync(math.degrees(init_yaw), 5).join()
        time.sleep(1)

        # 4. 开始无限循环
        t = 0
        plot_last_pos = [airsim.Vector3r(start_x, start_y, z_target)]
        
        while self.running and not self.stop_event.is_set():
            idx = t % total_steps
            
            # 读取当前的状态
            UAV_state = client.simGetGroundTruthKinematics()
            pos_now = np.array([[UAV_state.position.x_val], [UAV_state.position.y_val], [UAV_state.position.z_val]])
            vel_now = np.array([[UAV_state.linear_velocity.x_val], [UAV_state.linear_velocity.y_val], [UAV_state.linear_velocity.z_val]])
            
            # 构建状态向量 [x, y, vx, vy]
            state_now = np.vstack((pos_now[0:2], vel_now[0:2]))
            
            # 获取当前时刻的目标状态
            state_des = np.vstack((p_traj[:, idx:idx+1], v_traj[:, idx:idx+1]))
            
            # LQR控制律: u = -K(x - x_d) + a_d (前馈)
            a_cmd = -np.dot(K, state_now - state_des) + a_traj[:, idx:idx+1]
            
            # 计算目标偏航角 (跟随切线方向)
            yaw_target = math.atan2(v_traj[1, idx], v_traj[0, idx])
            
            move_by_acceleration_horizontal(client, a_cmd[0, 0], a_cmd[1, 0], z_target, yaw_target, duration=dt)
            
            # 仅绘制无人机实际飞行轨迹 (红线)
            #plot_v_start = [airsim.Vector3r(pos_now[0, 0], pos_now[1, 0], pos_now[2, 0])]
            #client.simPlotLineList(plot_last_pos + plot_v_start, color_rgba=[1.0, 0.0, 0.0, 1.0], is_persistent=True)
            #plot_last_pos = plot_v_start
            
            t += 1
            self.record_count += 1
            
            if self.stop_event.wait(dt):
                break
        
        print("飞行结束")

    def _square_flight(self, client, radius=3.0, speed_factor=0.5):
        """
        执行无限循环的正方形轨迹飞行
        - 中心位于坐标原点 (0,0)
        - 正方形外接圆半径由 radius 参数指定 (顶点到中心距离)
        - 机头朝向跟随飞行方向
        :param radius: 外接圆半径 (米)，决定了正方形的大小
        :param speed_factor: 速度倍率
        """
        # --- 1. 轨迹生成函数 (正方形版) ---
        def create_trajectory(r, scale):
            # 基础步数：一圈 400 个点 (每边 100 个点)
            base_steps = 400
            # 速度越快，步数越少
            steps_per_side = max(int(base_steps / (scale * 4)), 20) 
            total_steps = steps_per_side * 4
            dt = 0.02
            
            # 1.1 生成位置轨迹
            p_traj = np.zeros((2, total_steps))
            
            # 定义正方形的四个顶点 (起点在 (r, 0)，逆时针飞行)
            # 这里的 r 是外接圆半径，顶点坐标为 (r*cos(45), r*sin(45)) 等
            # 简单起见，定义一个边长为 side 的正方形，中心在原点
            # side = r * sqrt(2)
            side_len = r * 1.414 
            
            # 定义四个顶点 (东北 -> 西北 -> 西南 -> 东南)
            # 注意 AirSim 坐标系：X=北, Y=东
            vertices = [
                [ side_len/2,  side_len/2], # 东北
                [-side_len/2,  side_len/2], # 西北
                [-side_len/2, -side_len/2], # 西南
                [side_len/2, -side_len/2]  # 东南
            ]

            # 线性插值生成每一段轨迹
            for seg in range(4):
                start_pt = np.array(vertices[seg])
                end_pt = np.array(vertices[(seg + 1) % 4])
                
                for i in range(steps_per_side):
                    alpha = i / steps_per_side
                    idx = seg * steps_per_side + i
                    p_traj[:, idx] = start_pt * (1 - alpha) + end_pt * alpha

            # 1.2 根据位置变化自动计算速度和加速度 (中心差分法)
            v_traj = np.zeros_like(p_traj)
            a_traj = np.zeros_like(p_traj)
            
            # 计算速度
            for i in range(total_steps):
                prev_idx = (i - 1 + total_steps) % total_steps
                next_idx = (i + 1) % total_steps
                # 中心差分计算速度
                v_traj[:, i] = (p_traj[:, next_idx] - p_traj[:, prev_idx]) / (2 * dt)
                    
            # 计算加速度
            for i in range(total_steps):
                prev_idx = (i - 1 + total_steps) % total_steps
                next_idx = (i + 1) % total_steps
                # 中心差分计算加速度
                # 在拐角处，这个值会非常大，代表转弯所需的向心加速度(数值上的近似)
                a_traj[:, i] = (v_traj[:, next_idx] - v_traj[:, prev_idx]) / (2 * dt)
                    
            return p_traj, v_traj, a_traj

        # --- 2. LQR求解函数 ---
        def solve_dlqr(A, B, Q, R):
            S = np.matrix(linalg.solve_discrete_are(A, B, Q, R))
            K = np.matrix(linalg.inv(B.T * S * B + R) * (B.T * S * A))
            return K

        # --- 3. 控制量转换函数 (保持不变) ---
        def move_by_acceleration_horizontal(client, ax_cmd, ay_cmd, z_cmd, yaw_cmd, duration):
            state = client.simGetGroundTruthKinematics()
            angles = airsim.to_eularian_angles(state.orientation)
            yaw_my = angles[2]
            g = 9.8
            
            sin_yaw = math.sin(yaw_my)
            cos_yaw = math.cos(yaw_my)
            A_psi = np.array([[sin_yaw, cos_yaw], [-cos_yaw, sin_yaw]])
            A_psi_inverse = np.linalg.inv(A_psi)
            
            angle_h_cmd = 1/g * np.dot(A_psi_inverse, np.array([[-ax_cmd], [-ay_cmd]]))
            a_x_cmd = math.atan(angle_h_cmd[0, 0])
            a_y_cmd = -math.atan(angle_h_cmd[1, 0])
            
            client.moveByRollPitchYawZAsync(a_x_cmd, a_y_cmd, yaw_cmd, z_cmd, duration)

        # ================= 主逻辑开始 =================
        print(f"开始正方形飞行(外接圆半径 {radius}m, 速度 x{speed_factor})...")
        
        dt = 0.02
        z_target = -3.0
        
        # 1. 生成轨迹
        p_traj, v_traj, a_traj = create_trajectory(radius, speed_factor)
        total_steps = p_traj.shape[1]
        
        # 2. 计算LQR参数
        A = np.array([[1, 0, dt, 0],
                    [0, 1, 0, dt],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
        B = np.array([[0, 0],
                    [0, 0],
                    [dt, 0],
                    [0, dt]])
        Q = np.diag([2, 2, 2, 2])
        R = np.diag([.1, .1])
        K = solve_dlqr(A, B, Q, R)

        # 3. 预先飞到起点并对准方向
        # 起点为第一个顶点
        start_x = p_traj[0, 0]
        start_y = p_traj[1, 0]
        print(f"移动至起始点 ({start_x:.1f}, {start_y:.1f})...")
        client.moveToPositionAsync(start_x, start_y, z_target, 2).join()
        
        # 计算初始偏航：第一段直线的方向
        init_yaw = math.atan2(v_traj[1, 0], v_traj[0, 0])
        client.rotateToYawAsync(math.degrees(init_yaw), 5).join()
        time.sleep(1)

        # 4. 开始无限循环
        t = 0
        plot_last_pos = [airsim.Vector3r(start_x, start_y, z_target)]
        
        while self.running and not self.stop_event.is_set():
            idx = t % total_steps
            
            # 读取当前的状态
            UAV_state = client.simGetGroundTruthKinematics()
            pos_now = np.array([[UAV_state.position.x_val], [UAV_state.position.y_val], [UAV_state.position.z_val]])
            vel_now = np.array([[UAV_state.linear_velocity.x_val], [UAV_state.linear_velocity.y_val], [UAV_state.linear_velocity.z_val]])
            
            # 构建状态向量 [x, y, vx, vy]
            state_now = np.vstack((pos_now[0:2], vel_now[0:2]))
            
            # 获取当前时刻的目标状态
            state_des = np.vstack((p_traj[:, idx:idx+1], v_traj[:, idx:idx+1]))
            
            # LQR控制律: u = -K(x - x_d) + a_d (前馈)
            a_cmd = -np.dot(K, state_now - state_des) + a_traj[:, idx:idx+1]
            
            # 计算目标偏航角 (跟随切线方向)
            yaw_target = math.atan2(v_traj[1, idx], v_traj[0, idx])
            
            move_by_acceleration_horizontal(client, a_cmd[0, 0], a_cmd[1, 0], z_target, yaw_target, duration=dt)
            
            # 仅绘制无人机实际飞行轨迹 (红线)
            #plot_v_start = [airsim.Vector3r(pos_now[0, 0], pos_now[1, 0], pos_now[2, 0])]
            #client.simPlotLineList(plot_last_pos + plot_v_start, color_rgba=[1.0, 0.0, 0.0, 1.0], is_persistent=True)
            #plot_last_pos = plot_v_start
            
            t += 1
            self.record_count += 1
            
            if self.stop_event.wait(dt):
                break
        
        print("飞行结束")
# 飞行检测模块

# 录屏前置模块
user32 = ctypes.windll.user32
user32.PrintWindow.restype = wintypes.HANDLE
user32.PrintWindow.argtypes = [wintypes.HWND, wintypes.HDC, wintypes.UINT]
PW_RENDERFULLCONTENT = 0x0002
camera = [0, 0, 0, 0, 0, 0, 0, 0]
sjj_path = None
threeD_path = None

class vedioThread(QThread):
    frame_ready = Signal(object, float)
    handle_ready = Signal(object, float)
    error_occurred = Signal(str)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.running = False
        self.fps = 8
        self.hwnd = None
        self.width, self.height = 0, 0
        # GDI 资源
        self.hwndDC, self.mfcDC, self.saveDC, self.saveBitMap = None, None, None, None
        # 保存最新帧
        self.current_frame = None
        
        # 预计算的不变量
        self.yolo_thread = yoloThread()
        self.yolo_thread_handle = QThread()
        self.yolo_thread.moveToThread(self.yolo_thread_handle)
        self.yolo_thread_handle.started.connect(self.yolo_thread.initialize_3d_reconstruction)
        self.handle_ready.connect(self.yolo_thread.get_yolo_object_positions)
        self.yolo_thread_handle.start()
        self.truth_worker = Airsim_real_time()
        self.truth_thread = QThread()
        self.truth_worker.moveToThread(self.truth_thread)
        self.truth_thread.started.connect(self.truth_worker.start_fetching)
        self.truth_thread.start()

    def run(self):
        self.running = True
        # 使用 try...finally 确保无论如何都会执行清理
        try:
            if not self._init_printwindow():
                self.error_occurred.emit("错误：无法初始化 PrintWindow。")
                return

            print("成功初始化 PrintWindow，将进行窗口捕获。")
            
            consecutive_failures = 0
            max_failures = 3
            count = 0 
            
            while self.running:
                try:
                    # 1. 窗口有效性检查 (防止 PrintWindow 卡死)
                    # 如果窗口句柄无效，直接跳出循环，不再尝试捕获
                    if not self._is_window_valid(self.hwnd):
                        print("检测到窗口句柄无效，停止捕获。")
                        break
                    
                    # 2. 刷新句柄逻辑 (如果窗口重启过)
                    if not self._refresh_window_handle():
                        consecutive_failures += 1
                        if consecutive_failures >= max_failures:
                            self.error_occurred.emit("错误：无法找到 UE4 窗口。")
                            break
                        self.msleep(500) # 失败后等待半秒
                        continue
                    
                    consecutive_failures = 0
                    
                    # 3. 捕获图像
                    frame = self._capture_with_printwindow()
                    if frame is not None:
                        #self.current_frame = frame
                        # 只有在 running 为 True 时才进行耗时计算
                        if self.running:
                            #self.get_yolo_object_positions()
                            current_time = time.time()
                            self.frame_ready.emit(frame, current_time)
                            if count % 4 == 0:
                                self.handle_ready.emit(frame, current_time)
                        count += 1
                    else:
                        self.msleep(10)
                        
                except Exception as e:
                    print(f"捕获循环出错：{str(e)}")
                    break
                
                # 控制帧率，同时让线程有机会响应 running 标志
                #if self.running:
                    #self.msleep(int(1000 / self.fps))

        finally:
            # 无论线程如何结束（正常停止、异常崩溃、窗口关闭），
            # 都会在这里执行清理，保证 GDI 资源不泄漏。
            print("线程退出，正在清理资源...")
            self._cleanup()
            if self.yolo_thread_handle.isRunning():
                self.yolo_thread_handle.quit()
                # 这里必须 wait，不然 YOLO 线程也会报 "Destroyed while running"
                if not self.yolo_thread_handle.wait(2000): # 最多等2秒
                    print("YOLO 线程未能及时停止，强制终止")
                    self.yolo_thread_handle.terminate()
                    self.yolo_thread_handle.wait()
            if hasattr(self, 'truth_worker') and self.truth_worker:
                self.truth_worker.running = False
            if hasattr(self, 'truth_thread') and self.truth_thread.isRunning():
                self.truth_thread.quit()
                self.truth_thread.wait(2000)
            print("视频捕获线程已完全停止。")
    # 主运行逻辑（😅存在感可以说是很低了）

    def _init_printwindow(self):
        try:
            self.hwnd = self._find_ue4_window()
            if not self.hwnd:
                print("PrintWindow: 未找到UE4窗口。")
                return False
            
            left, top, right, bottom = win32gui.GetWindowRect(self.hwnd)
            self.width, self.height = right - left, bottom - top
            if self.width <= 0 or self.height <= 0:
                print("PrintWindow: 窗口尺寸无效。")
                return False
            
            print(f"找到UE4窗口，句柄: {self.hwnd}, 尺寸: {self.width}x{self.height}")

            # 获取窗口设备上下文
            self.hwndDC = win32gui.GetWindowDC(self.hwnd)
            self.mfcDC = win32ui.CreateDCFromHandle(self.hwndDC)
            self.saveDC = self.mfcDC.CreateCompatibleDC()
            
            # 创建位图
            self.saveBitMap = win32ui.CreateBitmap()
            self.saveBitMap.CreateCompatibleBitmap(self.mfcDC, self.width, self.height)
            self.saveDC.SelectObject(self.saveBitMap)
            
            return True
        except Exception as e:
            print(f"PrintWindow初始化失败: {e}")
            traceback.print_exc()
            return False
    # 初始化PrintWindow所需所有资源

    def _capture_with_printwindow(self):
        # 进入 API 前再次检查窗口
        if not self.hwnd or not win32gui.IsWindow(self.hwnd):
            return None
            
        # 调用 PrintWindow
        result = user32.PrintWindow(self.hwnd, self.saveDC.GetSafeHdc(), PW_RENDERFULLCONTENT)
        
        if result:
            try:
                bmpstr = self.saveBitMap.GetBitmapBits(True)
                img = np.frombuffer(bmpstr, dtype=np.uint8)
                img.shape = (self.height, self.width, 4)
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
                return img
            except Exception as e:
                print(f"解析位图出错: {e}")
                return None
        return None
    # 捕获一帧

    def _find_ue4_window(self):
        window_titles = [
            'try2 (64-bit Development PCD3D_SM5)', 
            'try2 (64-bit Development)',
            'Unreal Engine',
            'Shining仿真平台',
            'try2'
        ]
        for title in window_titles:
            hwnd = win32gui.FindWindow(None, title)
            if hwnd:
                return hwnd
        return win32gui.FindWindow("UnrealWindow", None)
    # 寻找UE4窗口句柄

    def set_fps(self, fps):
        self.fps = fps
    # 设置帧率（虽然远低于这个值😭）

#========================================神秘安全措施========================================#
    def _cleanup(self):
        """清理所有 GDI 资源"""
        try:
            # 按正确顺序释放资源（先释放依赖项）
            if self.saveBitMap:
                try:
                    win32gui.DeleteObject(self.saveBitMap.GetHandle())
                except Exception:
                    pass
                self.saveBitMap = None
                
            if self.saveDC:
                try:
                    self.saveDC.DeleteDC()
                except Exception:
                    pass
                self.saveDC = None
                
            if self.mfcDC:
                try:
                    self.mfcDC.DeleteDC()
                except Exception:
                    pass
                self.mfcDC = None
                
            if self.hwndDC and self.hwnd:
                try:
                    # 检查窗口是否仍然有效
                    if win32gui.IsWindow(self.hwnd):
                        win32gui.ReleaseDC(self.hwnd, self.hwndDC)
                except Exception:
                    pass
                self.hwndDC = None
                
            self.hwnd = None
            print("GDI 资源已清理。")
        except Exception as e:
            print(f"释放资源时出错：{e}")
            traceback.print_exc()
    # 安全释放资源

    def _is_window_valid(self, hwnd):
        if not hwnd:
            return False
        try:
            return win32gui.IsWindow(hwnd) and win32gui.IsWindowVisible(hwnd)
        except Exception:
            return False
    # 检查窗口句柄是否仍然有效

    def _refresh_window_handle(self):
        if not self._is_window_valid(self.hwnd):
            print("窗口句柄失效，尝试重新获取...")
            self._cleanup()  # 清理旧资源
            if self._init_printwindow():
                print("成功重新获取窗口句柄")
                return True
            else:
                print("无法重新获取窗口句柄")
                return False
        return True
    # 刷新窗口句柄,失去则重新获取

    def stop(self):
        print("收到停止视频捕获信号")
        self.running = False
        self.requestInterruption()
    # 停止视频捕获
# 视频采集线程😭，你承受了太多，我一定会优化你的

class yoloThread(QObject):
    error_occurred = Signal(str)
    points_3d_ready = Signal(object, float)

    def __init__(self, parent=None):
        super(yoloThread, self).__init__(parent)
        # 预计算的不变量
        self.model = None

    @Slot()
    def initialize_3d_reconstruction(self):
        try:
            # 1. 预加载YOLO模型（只加载一次）
            if self.model is None:
                model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'WindowsNoEditor', pt_name)
                device = 'cuda' if torch.cuda.is_available() else 'cpu'
                print(f"正在加载模型，使用设备: {device}")
            
                # 加载模型并显式推送到GPU
                self.model = YOLO(model_path).to(device)
                print("YOLO模型已加载")
        except Exception as e:
            print(f"加载模型时出错: {e}")
            traceback.print_exc()
    # 在开始工作前预计算所有不变量
    
    @Slot(object, float)
    def get_yolo_position(self, frame):
        """
        使用YOLO-Pose模型获取物体关键点位置
        返回一个8行5列的数组，每行代表一个切割图像中的5个关键点
        """
        all_points = []

        try:
            # 检查图片是否成功读取
            if frame is not None:
                # 批量处理所有裁剪图像以提高效率
                cropped_images = []
                valid_indices = []
                
                # 首先收集所有有效的裁剪区域
                for i, (x, y) in enumerate(picture_split):
                    # 确保裁剪区域不超出图像边界
                    if (0 <= y < frame.shape[0] and 0 <= x < frame.shape[1] and 
                        y + height <= frame.shape[0] and x + width <= frame.shape[1]):
                        # 裁剪图片
                        cropped_image = frame[y:y+height, x:x+width]
                        cropped_images.append(cropped_image)
                        valid_indices.append(i)
                
                # 批量进行YOLO检测
                if cropped_images:
                    results = self.model(cropped_images, verbose = False)  # 关闭详细输出
                    #results = self.model(cropped_images, verbose = True)  # 开启详细输出
                    
                    # 处理每个检测结果
                    result_idx = 0
                    for i in range(8):
                        image_points = []
                        
                        if i in valid_indices:
                            # 获取对应的检测结果
                            result = results[result_idx]
                            
                            # 更严格的检查条件，确保关键点数据存在且非空
                            if (len(result) > 0 and 
                                hasattr(result[0], 'keypoints') and 
                                result[0].keypoints is not None and
                                hasattr(result[0].keypoints, 'xy') and
                                result[0].keypoints.xy is not None and
                                len(result[0].keypoints.xy) > 0 and
                                result[0].keypoints.xy[0].shape[0] > 0):
                                
                                try:
                                    # 获取关键点坐标
                                    keypoints = result[0].keypoints.xy[0].cpu().numpy()
                                    
                                    # 提取前5个关键点
                                    for j in range(5):
                                        if j < len(keypoints):
                                            image_points.append(keypoints[j].tolist())
                                        else:
                                            image_points.append([0, 0])  # 用(0,0)填充缺失的关键点
                                except (IndexError, AttributeError, ValueError) as e:
                                    print(f"处理关键点时出错 (图像 {i}): {e}")
                                    image_points = [[0, 0] for _ in range(5)]
                            else:
                                # 如果没有检测到关键点或关键点为空，用默认值填充
                                image_points = [[0, 0] for _ in range(5)]
                            
                            result_idx += 1
                        else:
                            # 如果裁剪区域超出边界，用默认值填充
                            image_points = [[0, 0] for _ in range(5)]
                            
                        all_points.append(image_points)
                    
                #print(f"成功检测到关键点，形状: {len(all_points)} x 5")
            else:
                print("无法获取视频帧")
                # 初始化默认值
                all_points = [[[0, 0] for _ in range(5)] for _ in range(8)]

            for i in range(8):
                if camera[i]:
                    all_points[i] = [[0, 0] for _ in range(5)]

            return all_points

        except Exception as e:
            print(f"YOLO关键点检测过程中出现错误: {e}")
            traceback.print_exc()
            # 出现异常时使用默认值
            return [[[0, 0] for _ in range(5)] for _ in range(8)]
    # 获取2D关键点
    
    def get_yolo_object_positions(self, frame, current_time):
        all_points = self.get_yolo_position(frame)
        self.points_3d_ready.emit(all_points, current_time)
        #print(positions)
    # 结算位置汇总(这个是个汇总函数😭)
# 拆出来的YOLO线程

class Airsim_real_time(QObject):
    def __init__(self, parent=None):
        super(Airsim_real_time, self).__init__(parent)
        # 预计算的不变量
        self.clientB = airsim.MultirotorClient()
        self.running = False

    @Slot()
    def start_fetching(self):
        self.running = True
        while self.running:
            try:
                t0 = time.time()
                drone_info = self.get_drone_info(self.clientB)
                timestamped_real_data = {
                    "timestamp": t0,
                    "drone_pose": drone_info
                }
                plot_real_data.append(timestamped_real_data)
                QThread.msleep(250)
            except Exception as e:
                print(f"获取真值失败: {e}")
    
    def get_drone_info(self, client):
        state = client.getMultirotorState()
        position = state.kinematics_estimated.position
        velocity = state.kinematics_estimated.linear_velocity
        orientation = state.kinematics_estimated.orientation

        # 计算欧拉角（俯仰、偏航、翻滚）
        pitch, roll, yaw = airsim.to_eularian_angles(orientation)

        return {
            "position": {
                "x": round(position.x_val, 3),
                "y": round(position.y_val, 3),
                "z": round(-position.z_val, 3)
            },
            "orientation": {
                "pitch": round(-pitch, 3),
                "roll": round(-roll, 3),
                "yaw": round(yaw, 3)
            }
        }
    # 获取无人机详细信息
# 真值获取外置版

class Stats():#😁👍嗨嗨嗨，主窗口构建与几乎所有的功能
    def __init__(self):
        self.window = QMainWindow()
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.window.setWindowTitle("shining仿真平台控制端")
        # 创建中央部件
        self.central_widget = QWidget()
        self.window.setCentralWidget(self.central_widget)
        # 设置UI到中央部件
        self.setupUi(self.central_widget)
        self.window.resize(1045, 592)
        # 其他功能初始化设置
        self.camera_param()
        self.clothes = 0
        self.change_clothes()
        self.initial_judge()
        self.video_thread = None
        self.fly_thread = None
        self.EKF_thread = None
        self.current_drone_pose = None
        self.clientB = airsim.MultirotorClient()
        #self.clientB.simPause(True)
        #self.EKF = RigidBodyEKF(qa=1.0, qalpha=1.0, R_diag=0.02**2)
        self.EKF = None
        self.init_R = None
        self.R_diag = None
        self.current_time = time.time()
        # 设置第二窗口

        # 3D展示模块😢能跑就行，这里会不了一点点
        self.setup_matplotlib_3d_view()
        self.reconstructed_points = []  # 存储最新的3D点数据
        self.point_plots = []  # 存储3D点的绘图对象
        self.comparison_fig = None # 用于存储对比图表的 Figure 引用

        # 预计算的不变量(3D重建移民)
        self.K_inv = None
        self.camera_positions = None
        self.A_matrices = None
        self.R_wc_matrices = None

        # 添加触发事件😊基本所有按钮都在这了
            # 打开UE仿真界面收集数据
        self.button1.clicked.connect(self.open_collect_UE)
            # 进行数据收集
        self.start.clicked.connect(self.collect_data)
            # 设置输出重定向
        self.setup_output_redirect()
            # 应用天气
        self.apply.clicked.connect(self.apply_weather)
            # 清除旧数据集
        self.pushButton_2.clicked.connect(self.clear_old_data)
            # 旧数据是否存在
       #self.check_existing_data()
            # 应用相机参数
        self.pushButton_3.clicked.connect(self.camera_param)
            # 无人机换装
        self.button2.clicked.connect(self.change_clothes)
            # 开启监视器
        self.pushButton_7.clicked.connect(self.vedio_param)
            # 启动飞行展示
        self.pushButton_4.clicked.connect(self.start_flight)
            # 应用新pt
        self.pushButton_5.clicked.connect(self.upload_pt)
            # 对比图查看
        self.pushButton_8.clicked.connect(self.show_large_comparison_plot)
        self.pushButton_19.clicked.connect(self.show_large_comparison_plot)
            # 应用R_diag
        self.pushButton_9.clicked.connect(self.apply_R_diag)
            # 导出当前绘图数据
        self.pushButton_20.clicked.connect(self.output_data)

    def setup_output_redirect(self):
        # 创建信号实例
        self.stream_signal = StreamSignal()
        self.stream_signal.text_written.connect(self.append_log)
        
        # 重定向标准输出和标准错误
        sys.stdout = StreamRedirector(self.stream_signal)
        sys.stderr = StreamRedirector(self.stream_signal)
    # woc啥玩意我啥时候写的这个功能😅

    def append_log(self, text):
        """向日志控件添加文本"""
        self.log.append(text.strip())  # strip()去除末尾换行符
        self.log_2.append(text.strip())  # strip()去除末尾换行符
        self.textEdit.append(text.strip())  # strip()去除末尾换行符
        self.log.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())  # 自动滚动到底部
        self.log_2.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())  # 自动滚动到底部
        self.textEdit.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())  # 自动滚动到底部
    # 打开日志功能

    def setupUi(self, widget):
        if not widget.objectName():
            widget.setObjectName(u"widget")
        widget.resize(1100, 690)
        self.verticalLayout_8 = QVBoxLayout(widget)
        self.verticalLayout_8.setObjectName(u"verticalLayout_8")
        self.horizontalLayout_36 = QHBoxLayout()
        self.horizontalLayout_36.setObjectName(u"horizontalLayout_36")
        self.button1 = QPushButton(widget)
        self.button1.setObjectName(u"button1")

        self.horizontalLayout_36.addWidget(self.button1)

        self.button2 = QPushButton(widget)
        self.button2.setObjectName(u"button2")

        self.horizontalLayout_36.addWidget(self.button2)

        self.horizontalSpacer = QSpacerItem(588, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_36.addItem(self.horizontalSpacer)


        self.verticalLayout_8.addLayout(self.horizontalLayout_36)

        self.tabWidget = QTabWidget(widget)
        self.tabWidget.setObjectName(u"tabWidget")
        self.tab_1 = QWidget()
        self.tab_1.setObjectName(u"tab_1")
        self.verticalLayout_5 = QVBoxLayout(self.tab_1)
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.horizontalLayout_17 = QHBoxLayout()
        self.horizontalLayout_17.setObjectName(u"horizontalLayout_17")
        self.boolweather = QCheckBox(self.tab_1)
        self.boolweather.setObjectName(u"boolweather")

        self.horizontalLayout_17.addWidget(self.boolweather)

        self.horizontalSpacer_4 = QSpacerItem(808, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_17.addItem(self.horizontalSpacer_4)


        self.verticalLayout_5.addLayout(self.horizontalLayout_17)

        self.horizontalLayout_16 = QHBoxLayout()
        self.horizontalLayout_16.setObjectName(u"horizontalLayout_16")
        self.formLayout = QFormLayout()
        self.formLayout.setObjectName(u"formLayout")
        self.label1 = QLabel(self.tab_1)
        self.label1.setObjectName(u"label1")

        self.formLayout.setWidget(0, QFormLayout.LabelRole, self.label1)

        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.horizontalSlider_1 = QSlider(self.tab_1)
        self.horizontalSlider_1.setObjectName(u"horizontalSlider_1")
        self.horizontalSlider_1.setOrientation(Qt.Horizontal)

        self.horizontalLayout_2.addWidget(self.horizontalSlider_1)

        self.label_rain_1 = QLabel(self.tab_1)
        self.label_rain_1.setObjectName(u"label_rain_1")

        self.horizontalLayout_2.addWidget(self.label_rain_1)


        self.formLayout.setLayout(0, QFormLayout.FieldRole, self.horizontalLayout_2)

        self.verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.formLayout.setItem(1, QFormLayout.LabelRole, self.verticalSpacer)

        self.label2 = QLabel(self.tab_1)
        self.label2.setObjectName(u"label2")

        self.formLayout.setWidget(2, QFormLayout.LabelRole, self.label2)

        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.horizontalSlider_2 = QSlider(self.tab_1)
        self.horizontalSlider_2.setObjectName(u"horizontalSlider_2")
        self.horizontalSlider_2.setOrientation(Qt.Horizontal)

        self.horizontalLayout_3.addWidget(self.horizontalSlider_2)

        self.label_rain_2 = QLabel(self.tab_1)
        self.label_rain_2.setObjectName(u"label_rain_2")

        self.horizontalLayout_3.addWidget(self.label_rain_2)


        self.formLayout.setLayout(2, QFormLayout.FieldRole, self.horizontalLayout_3)

        self.verticalSpacer_2 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.formLayout.setItem(3, QFormLayout.LabelRole, self.verticalSpacer_2)

        self.label3 = QLabel(self.tab_1)
        self.label3.setObjectName(u"label3")

        self.formLayout.setWidget(4, QFormLayout.LabelRole, self.label3)

        self.horizontalLayout_4 = QHBoxLayout()
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.horizontalSlider_3 = QSlider(self.tab_1)
        self.horizontalSlider_3.setObjectName(u"horizontalSlider_3")
        self.horizontalSlider_3.setOrientation(Qt.Horizontal)

        self.horizontalLayout_4.addWidget(self.horizontalSlider_3)

        self.label_rain_3 = QLabel(self.tab_1)
        self.label_rain_3.setObjectName(u"label_rain_3")

        self.horizontalLayout_4.addWidget(self.label_rain_3)


        self.formLayout.setLayout(4, QFormLayout.FieldRole, self.horizontalLayout_4)

        self.verticalSpacer_3 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.formLayout.setItem(5, QFormLayout.LabelRole, self.verticalSpacer_3)

        self.label4 = QLabel(self.tab_1)
        self.label4.setObjectName(u"label4")

        self.formLayout.setWidget(6, QFormLayout.LabelRole, self.label4)

        self.horizontalLayout_5 = QHBoxLayout()
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.horizontalSlider_4 = QSlider(self.tab_1)
        self.horizontalSlider_4.setObjectName(u"horizontalSlider_4")
        self.horizontalSlider_4.setOrientation(Qt.Horizontal)

        self.horizontalLayout_5.addWidget(self.horizontalSlider_4)

        self.label_rain_4 = QLabel(self.tab_1)
        self.label_rain_4.setObjectName(u"label_rain_4")

        self.horizontalLayout_5.addWidget(self.label_rain_4)


        self.formLayout.setLayout(6, QFormLayout.FieldRole, self.horizontalLayout_5)

        self.label5 = QLabel(self.tab_1)
        self.label5.setObjectName(u"label5")

        self.formLayout.setWidget(8, QFormLayout.LabelRole, self.label5)

        self.horizontalLayout_6 = QHBoxLayout()
        self.horizontalLayout_6.setObjectName(u"horizontalLayout_6")
        self.horizontalSlider_5 = QSlider(self.tab_1)
        self.horizontalSlider_5.setObjectName(u"horizontalSlider_5")
        self.horizontalSlider_5.setOrientation(Qt.Horizontal)

        self.horizontalLayout_6.addWidget(self.horizontalSlider_5)

        self.label_rain_5 = QLabel(self.tab_1)
        self.label_rain_5.setObjectName(u"label_rain_5")

        self.horizontalLayout_6.addWidget(self.label_rain_5)


        self.formLayout.setLayout(8, QFormLayout.FieldRole, self.horizontalLayout_6)

        self.label6 = QLabel(self.tab_1)
        self.label6.setObjectName(u"label6")

        self.formLayout.setWidget(10, QFormLayout.LabelRole, self.label6)

        self.horizontalLayout_7 = QHBoxLayout()
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.horizontalSlider_6 = QSlider(self.tab_1)
        self.horizontalSlider_6.setObjectName(u"horizontalSlider_6")
        self.horizontalSlider_6.setOrientation(Qt.Horizontal)

        self.horizontalLayout_7.addWidget(self.horizontalSlider_6)

        self.label_rain_6 = QLabel(self.tab_1)
        self.label_rain_6.setObjectName(u"label_rain_6")

        self.horizontalLayout_7.addWidget(self.label_rain_6)


        self.formLayout.setLayout(10, QFormLayout.FieldRole, self.horizontalLayout_7)

        self.verticalSpacer_4 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.formLayout.setItem(7, QFormLayout.LabelRole, self.verticalSpacer_4)

        self.verticalSpacer_5 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.formLayout.setItem(9, QFormLayout.LabelRole, self.verticalSpacer_5)

        self.verticalSpacer_6 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.formLayout.setItem(11, QFormLayout.LabelRole, self.verticalSpacer_6)

        self.label7 = QLabel(self.tab_1)
        self.label7.setObjectName(u"label7")

        self.formLayout.setWidget(12, QFormLayout.LabelRole, self.label7)

        self.verticalSpacer_7 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.formLayout.setItem(13, QFormLayout.LabelRole, self.verticalSpacer_7)

        self.label8 = QLabel(self.tab_1)
        self.label8.setObjectName(u"label8")

        self.formLayout.setWidget(14, QFormLayout.LabelRole, self.label8)

        self.horizontalLayout_9 = QHBoxLayout()
        self.horizontalLayout_9.setObjectName(u"horizontalLayout_9")
        self.horizontalSlider_7 = QSlider(self.tab_1)
        self.horizontalSlider_7.setObjectName(u"horizontalSlider_7")
        self.horizontalSlider_7.setOrientation(Qt.Horizontal)

        self.horizontalLayout_9.addWidget(self.horizontalSlider_7)

        self.label_rain_7 = QLabel(self.tab_1)
        self.label_rain_7.setObjectName(u"label_rain_7")

        self.horizontalLayout_9.addWidget(self.label_rain_7)


        self.formLayout.setLayout(12, QFormLayout.FieldRole, self.horizontalLayout_9)

        self.horizontalLayout_10 = QHBoxLayout()
        self.horizontalLayout_10.setObjectName(u"horizontalLayout_10")
        self.horizontalSlider_8 = QSlider(self.tab_1)
        self.horizontalSlider_8.setObjectName(u"horizontalSlider_8")
        self.horizontalSlider_8.setOrientation(Qt.Horizontal)

        self.horizontalLayout_10.addWidget(self.horizontalSlider_8)

        self.label_rain_8 = QLabel(self.tab_1)
        self.label_rain_8.setObjectName(u"label_rain_8")

        self.horizontalLayout_10.addWidget(self.label_rain_8)


        self.formLayout.setLayout(14, QFormLayout.FieldRole, self.horizontalLayout_10)


        self.horizontalLayout_16.addLayout(self.formLayout)

        self.verticalLayout_4 = QVBoxLayout()
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.verticalLayout_2 = QVBoxLayout()
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.horizontalLayout_14 = QHBoxLayout()
        self.horizontalLayout_14.setObjectName(u"horizontalLayout_14")
        self.label = QLabel(self.tab_1)
        self.label.setObjectName(u"label")

        self.horizontalLayout_14.addWidget(self.label)

        self.horizontalSpacer_2 = QSpacerItem(348, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_14.addItem(self.horizontalSpacer_2)


        self.verticalLayout_2.addLayout(self.horizontalLayout_14)

        self.formLayout_2 = QFormLayout()
        self.formLayout_2.setObjectName(u"formLayout_2")
        self.X_lable = QLabel(self.tab_1)
        self.X_lable.setObjectName(u"X_lable")

        self.formLayout_2.setWidget(0, QFormLayout.LabelRole, self.X_lable)

        self.horizontalLayout_11 = QHBoxLayout()
        self.horizontalLayout_11.setObjectName(u"horizontalLayout_11")
        self.horizontalSlider_9 = QSlider(self.tab_1)
        self.horizontalSlider_9.setObjectName(u"horizontalSlider_9")
        self.horizontalSlider_9.setOrientation(Qt.Horizontal)

        self.horizontalLayout_11.addWidget(self.horizontalSlider_9)

        self.label_rain_9 = QLabel(self.tab_1)
        self.label_rain_9.setObjectName(u"label_rain_9")

        self.horizontalLayout_11.addWidget(self.label_rain_9)


        self.formLayout_2.setLayout(0, QFormLayout.FieldRole, self.horizontalLayout_11)

        self.Y_lable = QLabel(self.tab_1)
        self.Y_lable.setObjectName(u"Y_lable")

        self.formLayout_2.setWidget(2, QFormLayout.LabelRole, self.Y_lable)

        self.horizontalLayout_12 = QHBoxLayout()
        self.horizontalLayout_12.setObjectName(u"horizontalLayout_12")
        self.horizontalSlider_10 = QSlider(self.tab_1)
        self.horizontalSlider_10.setObjectName(u"horizontalSlider_10")
        self.horizontalSlider_10.setOrientation(Qt.Horizontal)

        self.horizontalLayout_12.addWidget(self.horizontalSlider_10)

        self.label_rain_10 = QLabel(self.tab_1)
        self.label_rain_10.setObjectName(u"label_rain_10")

        self.horizontalLayout_12.addWidget(self.label_rain_10)


        self.formLayout_2.setLayout(2, QFormLayout.FieldRole, self.horizontalLayout_12)

        self.Z_lable = QLabel(self.tab_1)
        self.Z_lable.setObjectName(u"Z_lable")

        self.formLayout_2.setWidget(4, QFormLayout.LabelRole, self.Z_lable)

        self.horizontalLayout_13 = QHBoxLayout()
        self.horizontalLayout_13.setObjectName(u"horizontalLayout_13")
        self.horizontalSlider_11 = QSlider(self.tab_1)
        self.horizontalSlider_11.setObjectName(u"horizontalSlider_11")
        self.horizontalSlider_11.setOrientation(Qt.Horizontal)

        self.horizontalLayout_13.addWidget(self.horizontalSlider_11)

        self.label_rain_11 = QLabel(self.tab_1)
        self.label_rain_11.setObjectName(u"label_rain_11")

        self.horizontalLayout_13.addWidget(self.label_rain_11)


        self.formLayout_2.setLayout(4, QFormLayout.FieldRole, self.horizontalLayout_13)

        self.verticalSpacer_8 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.formLayout_2.setItem(1, QFormLayout.LabelRole, self.verticalSpacer_8)

        self.verticalSpacer_9 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.formLayout_2.setItem(3, QFormLayout.LabelRole, self.verticalSpacer_9)


        self.verticalLayout_2.addLayout(self.formLayout_2)


        self.verticalLayout_4.addLayout(self.verticalLayout_2)

        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.horizontalLayout_15 = QHBoxLayout()
        self.horizontalLayout_15.setObjectName(u"horizontalLayout_15")
        self.label_2 = QLabel(self.tab_1)
        self.label_2.setObjectName(u"label_2")

        self.horizontalLayout_15.addWidget(self.label_2)

        self.horizontalSpacer_3 = QSpacerItem(398, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_15.addItem(self.horizontalSpacer_3)


        self.verticalLayout_3.addLayout(self.horizontalLayout_15)

        self.log = QTextEdit(self.tab_1)
        self.log.setObjectName(u"log")

        self.verticalLayout_3.addWidget(self.log)


        self.verticalLayout_4.addLayout(self.verticalLayout_3)


        self.horizontalLayout_16.addLayout(self.verticalLayout_4)


        self.verticalLayout_5.addLayout(self.horizontalLayout_16)

        self.horizontalLayout_18 = QHBoxLayout()
        self.horizontalLayout_18.setObjectName(u"horizontalLayout_18")
        self.apply = QPushButton(self.tab_1)
        self.apply.setObjectName(u"apply")

        self.horizontalLayout_18.addWidget(self.apply)

        self.start = QPushButton(self.tab_1)
        self.start.setObjectName(u"start")

        self.horizontalLayout_18.addWidget(self.start)

        self.pushButton = QPushButton(self.tab_1)
        self.pushButton.setObjectName(u"pushButton")

        self.horizontalLayout_18.addWidget(self.pushButton)

        self.pushButton_2 = QPushButton(self.tab_1)
        self.pushButton_2.setObjectName(u"pushButton_2")

        self.horizontalLayout_18.addWidget(self.pushButton_2)

        self.horizontalSpacer_5 = QSpacerItem(758, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_18.addItem(self.horizontalSpacer_5)


        self.verticalLayout_5.addLayout(self.horizontalLayout_18)

        self.tabWidget.addTab(self.tab_1, "")
        self.tab_2 = QWidget()
        self.tab_2.setObjectName(u"tab_2")
        self.horizontalLayout_33 = QHBoxLayout(self.tab_2)
        self.horizontalLayout_33.setObjectName(u"horizontalLayout_33")
        self.tabWidget_2 = QTabWidget(self.tab_2)
        self.tabWidget_2.setObjectName(u"tabWidget_2")
        self.tab_4 = QWidget()
        self.tab_4.setObjectName(u"tab_4")
        self.horizontalLayout = QHBoxLayout(self.tab_4)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.label_3 = QLabel(self.tab_4)
        self.label_3.setObjectName(u"label_3")

        self.horizontalLayout.addWidget(self.label_3)

        self.tabWidget_2.addTab(self.tab_4, "")
        self.tab_5 = QWidget()
        self.tab_5.setObjectName(u"tab_5")
        self.horizontalLayout_28 = QHBoxLayout(self.tab_5)
        self.horizontalLayout_28.setObjectName(u"horizontalLayout_28")
        self.label_20 = QLabel(self.tab_5)
        self.label_20.setObjectName(u"label_20")

        self.horizontalLayout_28.addWidget(self.label_20)

        self.tabWidget_2.addTab(self.tab_5, "")
        self.tab_3 = QWidget()
        self.tab_3.setObjectName(u"tab_3")
        self.horizontalLayout_30 = QHBoxLayout(self.tab_3)
        self.horizontalLayout_30.setObjectName(u"horizontalLayout_30")
        self.label_4 = QLabel(self.tab_3)
        self.label_4.setObjectName(u"label_4")

        self.horizontalLayout_30.addWidget(self.label_4)

        self.tabWidget_2.addTab(self.tab_3, "")
        self.tab_6 = QWidget()
        self.tab_6.setObjectName(u"tab_6")
        self.verticalLayout_10 = QVBoxLayout(self.tab_6)
        self.verticalLayout_10.setObjectName(u"verticalLayout_10")
        self.horizontalLayout_35 = QHBoxLayout()
        self.horizontalLayout_35.setObjectName(u"horizontalLayout_35")
        self.label_52 = QLabel(self.tab_6)
        self.label_52.setObjectName(u"label_52")

        self.horizontalLayout_35.addWidget(self.label_52)

        self.horizontalSpacer_10 = QSpacerItem(188, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_35.addItem(self.horizontalSpacer_10)


        self.verticalLayout_10.addLayout(self.horizontalLayout_35)

        self.horizontalLayout_41 = QHBoxLayout()
        self.horizontalLayout_41.setObjectName(u"horizontalLayout_41")
        self.gridLayout_2 = QGridLayout()
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.pushButton_6 = QPushButton(self.tab_6)
        self.pushButton_6.setObjectName(u"pushButton_6")
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_6.sizePolicy().hasHeightForWidth())
        self.pushButton_6.setSizePolicy(sizePolicy)
        self.pushButton_6.setMinimumSize(QSize(115, 115))
        self.pushButton_6.setMaximumSize(QSize(300, 300))

        self.gridLayout_2.addWidget(self.pushButton_6, 0, 0, 1, 1)

        self.pushButton_10 = QPushButton(self.tab_6)
        self.pushButton_10.setObjectName(u"pushButton_10")
        self.pushButton_10.setMinimumSize(QSize(115, 115))
        self.pushButton_10.setMaximumSize(QSize(300, 300))

        self.gridLayout_2.addWidget(self.pushButton_10, 0, 1, 1, 1)

        self.pushButton_11 = QPushButton(self.tab_6)
        self.pushButton_11.setObjectName(u"pushButton_11")
        sizePolicy.setHeightForWidth(self.pushButton_11.sizePolicy().hasHeightForWidth())
        self.pushButton_11.setSizePolicy(sizePolicy)
        self.pushButton_11.setMinimumSize(QSize(115, 115))
        self.pushButton_11.setMaximumSize(QSize(300, 300))

        self.gridLayout_2.addWidget(self.pushButton_11, 0, 2, 1, 1)

        self.pushButton_12 = QPushButton(self.tab_6)
        self.pushButton_12.setObjectName(u"pushButton_12")
        self.pushButton_12.setMinimumSize(QSize(115, 115))
        self.pushButton_12.setMaximumSize(QSize(300, 300))

        self.gridLayout_2.addWidget(self.pushButton_12, 1, 0, 1, 1)

        self.pushButton_13 = QPushButton(self.tab_6)
        self.pushButton_13.setObjectName(u"pushButton_13")
        self.pushButton_13.setMinimumSize(QSize(115, 115))
        self.pushButton_13.setMaximumSize(QSize(300, 300))

        self.gridLayout_2.addWidget(self.pushButton_13, 1, 1, 1, 1)

        self.pushButton_14 = QPushButton(self.tab_6)
        self.pushButton_14.setObjectName(u"pushButton_14")
        self.pushButton_14.setMinimumSize(QSize(115, 115))
        self.pushButton_14.setMaximumSize(QSize(300, 300))

        self.gridLayout_2.addWidget(self.pushButton_14, 1, 2, 1, 1)

        self.pushButton_15 = QPushButton(self.tab_6)
        self.pushButton_15.setObjectName(u"pushButton_15")
        self.pushButton_15.setMinimumSize(QSize(115, 115))
        self.pushButton_15.setMaximumSize(QSize(300, 300))

        self.gridLayout_2.addWidget(self.pushButton_15, 2, 0, 1, 1)

        self.pushButton_16 = QPushButton(self.tab_6)
        self.pushButton_16.setObjectName(u"pushButton_16")
        self.pushButton_16.setMinimumSize(QSize(115, 115))
        self.pushButton_16.setMaximumSize(QSize(300, 300))

        self.gridLayout_2.addWidget(self.pushButton_16, 2, 1, 1, 1)

        self.pushButton_17 = QPushButton(self.tab_6)
        self.pushButton_17.setObjectName(u"pushButton_17")
        self.pushButton_17.setMinimumSize(QSize(115, 115))
        self.pushButton_17.setMaximumSize(QSize(300, 300))

        self.gridLayout_2.addWidget(self.pushButton_17, 2, 2, 1, 1)


        self.horizontalLayout_41.addLayout(self.gridLayout_2)

        self.formLayout_4 = QFormLayout()
        self.formLayout_4.setObjectName(u"formLayout_4")
        self.formLayout_4.setLabelAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)
        self.formLayout_4.setFormAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignTop)
        self.formLayout_4.setHorizontalSpacing(6)
        self.label_71 = QLabel(self.tab_6)
        self.label_71.setObjectName(u"label_71")

        self.formLayout_4.setWidget(0, QFormLayout.LabelRole, self.label_71)

        self.label_55 = QLabel(self.tab_6)
        self.label_55.setObjectName(u"label_55")
        self.label_55.setMinimumSize(QSize(0, 0))

        self.formLayout_4.setWidget(1, QFormLayout.LabelRole, self.label_55)

        self.verticalSpacer_28 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.formLayout_4.setItem(2, QFormLayout.LabelRole, self.verticalSpacer_28)

        self.label_56 = QLabel(self.tab_6)
        self.label_56.setObjectName(u"label_56")

        self.formLayout_4.setWidget(3, QFormLayout.LabelRole, self.label_56)

        self.verticalSpacer_29 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.formLayout_4.setItem(4, QFormLayout.LabelRole, self.verticalSpacer_29)

        self.label_57 = QLabel(self.tab_6)
        self.label_57.setObjectName(u"label_57")

        self.formLayout_4.setWidget(5, QFormLayout.LabelRole, self.label_57)

        self.verticalSpacer_30 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.formLayout_4.setItem(6, QFormLayout.LabelRole, self.verticalSpacer_30)

        self.label_58 = QLabel(self.tab_6)
        self.label_58.setObjectName(u"label_58")

        self.formLayout_4.setWidget(7, QFormLayout.LabelRole, self.label_58)

        self.verticalSpacer_31 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.formLayout_4.setItem(8, QFormLayout.LabelRole, self.verticalSpacer_31)

        self.label_59 = QLabel(self.tab_6)
        self.label_59.setObjectName(u"label_59")

        self.formLayout_4.setWidget(9, QFormLayout.LabelRole, self.label_59)

        self.verticalSpacer_32 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.formLayout_4.setItem(10, QFormLayout.LabelRole, self.verticalSpacer_32)

        self.label_60 = QLabel(self.tab_6)
        self.label_60.setObjectName(u"label_60")

        self.formLayout_4.setWidget(11, QFormLayout.LabelRole, self.label_60)

        self.verticalSpacer_33 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.formLayout_4.setItem(12, QFormLayout.LabelRole, self.verticalSpacer_33)

        self.label_61 = QLabel(self.tab_6)
        self.label_61.setObjectName(u"label_61")

        self.formLayout_4.setWidget(13, QFormLayout.LabelRole, self.label_61)

        self.verticalSpacer_34 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.formLayout_4.setItem(14, QFormLayout.LabelRole, self.verticalSpacer_34)

        self.label_62 = QLabel(self.tab_6)
        self.label_62.setObjectName(u"label_62")

        self.formLayout_4.setWidget(15, QFormLayout.LabelRole, self.label_62)

        self.label_63 = QLabel(self.tab_6)
        self.label_63.setObjectName(u"label_63")
        self.label_63.setAlignment(Qt.AlignCenter)

        self.formLayout_4.setWidget(1, QFormLayout.FieldRole, self.label_63)

        self.label_64 = QLabel(self.tab_6)
        self.label_64.setObjectName(u"label_64")
        self.label_64.setAlignment(Qt.AlignCenter)

        self.formLayout_4.setWidget(3, QFormLayout.FieldRole, self.label_64)

        self.label_65 = QLabel(self.tab_6)
        self.label_65.setObjectName(u"label_65")
        self.label_65.setAlignment(Qt.AlignCenter)

        self.formLayout_4.setWidget(5, QFormLayout.FieldRole, self.label_65)

        self.label_66 = QLabel(self.tab_6)
        self.label_66.setObjectName(u"label_66")
        self.label_66.setAlignment(Qt.AlignCenter)

        self.formLayout_4.setWidget(7, QFormLayout.FieldRole, self.label_66)

        self.label_67 = QLabel(self.tab_6)
        self.label_67.setObjectName(u"label_67")
        self.label_67.setAlignment(Qt.AlignCenter)

        self.formLayout_4.setWidget(9, QFormLayout.FieldRole, self.label_67)

        self.label_68 = QLabel(self.tab_6)
        self.label_68.setObjectName(u"label_68")
        self.label_68.setAlignment(Qt.AlignCenter)

        self.formLayout_4.setWidget(11, QFormLayout.FieldRole, self.label_68)

        self.label_69 = QLabel(self.tab_6)
        self.label_69.setObjectName(u"label_69")
        self.label_69.setAlignment(Qt.AlignCenter)

        self.formLayout_4.setWidget(13, QFormLayout.FieldRole, self.label_69)

        self.label_70 = QLabel(self.tab_6)
        self.label_70.setObjectName(u"label_70")
        self.label_70.setAlignment(Qt.AlignCenter)

        self.formLayout_4.setWidget(15, QFormLayout.FieldRole, self.label_70)


        self.horizontalLayout_41.addLayout(self.formLayout_4)


        self.verticalLayout_10.addLayout(self.horizontalLayout_41)

        self.verticalSpacer_35 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_10.addItem(self.verticalSpacer_35)

        self.line_14 = QFrame(self.tab_6)
        self.line_14.setObjectName(u"line_14")
        self.line_14.setFrameShape(QFrame.HLine)
        self.line_14.setFrameShadow(QFrame.Sunken)

        self.verticalLayout_10.addWidget(self.line_14)

        self.verticalLayout_9 = QVBoxLayout()
        self.verticalLayout_9.setObjectName(u"verticalLayout_9")
        self.verticalSpacer_36 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_9.addItem(self.verticalSpacer_36)

        self.horizontalLayout_37 = QHBoxLayout()
        self.horizontalLayout_37.setObjectName(u"horizontalLayout_37")
        self.radioButton = QRadioButton(self.tab_6)
        self.radioButton.setObjectName(u"radioButton")

        self.horizontalLayout_37.addWidget(self.radioButton)

        self.horizontalSpacer_9 = QSpacerItem(338, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_37.addItem(self.horizontalSpacer_9)


        self.verticalLayout_9.addLayout(self.horizontalLayout_37)

        self.horizontalLayout_38 = QHBoxLayout()
        self.horizontalLayout_38.setObjectName(u"horizontalLayout_38")
        self.label_53 = QLabel(self.tab_6)
        self.label_53.setObjectName(u"label_53")

        self.horizontalLayout_38.addWidget(self.label_53)

        self.horizontalSlider = QSlider(self.tab_6)
        self.horizontalSlider.setObjectName(u"horizontalSlider")
        self.horizontalSlider.setOrientation(Qt.Horizontal)

        self.horizontalLayout_38.addWidget(self.horizontalSlider)

        self.label_54 = QLabel(self.tab_6)
        self.label_54.setObjectName(u"label_54")

        self.horizontalLayout_38.addWidget(self.label_54)


        self.verticalLayout_9.addLayout(self.horizontalLayout_38)

        self.horizontalLayout_39 = QHBoxLayout()
        self.horizontalLayout_39.setObjectName(u"horizontalLayout_39")
        self.label_74 = QLabel(self.tab_6)
        self.label_74.setObjectName(u"label_74")

        self.horizontalLayout_39.addWidget(self.label_74)

        self.horizontalSlider_12 = QSlider(self.tab_6)
        self.horizontalSlider_12.setObjectName(u"horizontalSlider_12")
        self.horizontalSlider_12.setOrientation(Qt.Horizontal)

        self.horizontalLayout_39.addWidget(self.horizontalSlider_12)

        self.label_75 = QLabel(self.tab_6)
        self.label_75.setObjectName(u"label_75")

        self.horizontalLayout_39.addWidget(self.label_75)


        self.verticalLayout_9.addLayout(self.horizontalLayout_39)

        self.horizontalLayout_40 = QHBoxLayout()
        self.horizontalLayout_40.setObjectName(u"horizontalLayout_40")
        self.label_72 = QLabel(self.tab_6)
        self.label_72.setObjectName(u"label_72")

        self.horizontalLayout_40.addWidget(self.label_72)

        self.lineEdit_14 = QLineEdit(self.tab_6)
        self.lineEdit_14.setObjectName(u"lineEdit_14")
        self.lineEdit_14.setMaximumSize(QSize(40, 16777215))

        self.horizontalLayout_40.addWidget(self.lineEdit_14)

        self.label_73 = QLabel(self.tab_6)
        self.label_73.setObjectName(u"label_73")

        self.horizontalLayout_40.addWidget(self.label_73)

        self.horizontalSpacer_11 = QSpacerItem(208, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_40.addItem(self.horizontalSpacer_11)


        self.verticalLayout_9.addLayout(self.horizontalLayout_40)

        self.pushButton_18 = QPushButton(self.tab_6)
        self.pushButton_18.setObjectName(u"pushButton_18")

        self.verticalLayout_9.addWidget(self.pushButton_18)


        self.verticalLayout_10.addLayout(self.verticalLayout_9)

        self.tabWidget_2.addTab(self.tab_6, "")

        self.horizontalLayout_33.addWidget(self.tabWidget_2)

        self.line = QFrame(self.tab_2)
        self.line.setObjectName(u"line")
        self.line.setFrameShape(QFrame.VLine)
        self.line.setFrameShadow(QFrame.Sunken)

        self.horizontalLayout_33.addWidget(self.line)

        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.label_48 = QLabel(self.tab_2)
        self.label_48.setObjectName(u"label_48")

        self.verticalLayout.addWidget(self.label_48)

        self.gridLayout = QGridLayout()
        self.gridLayout.setObjectName(u"gridLayout")
        self.label_23 = QLabel(self.tab_2)
        self.label_23.setObjectName(u"label_23")

        self.gridLayout.addWidget(self.label_23, 0, 2, 1, 1)

        self.line_5 = QFrame(self.tab_2)
        self.line_5.setObjectName(u"line_5")
        self.line_5.setFrameShape(QFrame.HLine)
        self.line_5.setFrameShadow(QFrame.Sunken)

        self.gridLayout.addWidget(self.line_5, 3, 2, 1, 1)

        self.label_40 = QLabel(self.tab_2)
        self.label_40.setObjectName(u"label_40")

        self.gridLayout.addWidget(self.label_40, 6, 1, 1, 1)

        self.verticalSpacer_21 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.gridLayout.addItem(self.verticalSpacer_21, 11, 0, 1, 1)

        self.verticalSpacer_18 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.gridLayout.addItem(self.verticalSpacer_18, 5, 0, 1, 1)

        self.label_44 = QLabel(self.tab_2)
        self.label_44.setObjectName(u"label_44")

        self.gridLayout.addWidget(self.label_44, 8, 2, 1, 1)

        self.label_41 = QLabel(self.tab_2)
        self.label_41.setObjectName(u"label_41")

        self.gridLayout.addWidget(self.label_41, 6, 2, 1, 1)

        self.line_8 = QFrame(self.tab_2)
        self.line_8.setObjectName(u"line_8")
        self.line_8.setFrameShape(QFrame.HLine)
        self.line_8.setFrameShadow(QFrame.Sunken)

        self.gridLayout.addWidget(self.line_8, 7, 1, 1, 1)

        self.label_49 = QLabel(self.tab_2)
        self.label_49.setObjectName(u"label_49")

        self.gridLayout.addWidget(self.label_49, 12, 0, 1, 1)

        self.line_7 = QFrame(self.tab_2)
        self.line_7.setObjectName(u"line_7")
        self.line_7.setFrameShape(QFrame.HLine)
        self.line_7.setFrameShadow(QFrame.Sunken)

        self.gridLayout.addWidget(self.line_7, 5, 2, 1, 1)

        self.label_39 = QLabel(self.tab_2)
        self.label_39.setObjectName(u"label_39")

        self.gridLayout.addWidget(self.label_39, 6, 0, 1, 1)

        self.line_2 = QFrame(self.tab_2)
        self.line_2.setObjectName(u"line_2")
        self.line_2.setFrameShape(QFrame.HLine)
        self.line_2.setFrameShadow(QFrame.Sunken)

        self.gridLayout.addWidget(self.line_2, 1, 1, 1, 1)

        self.label_50 = QLabel(self.tab_2)
        self.label_50.setObjectName(u"label_50")

        self.gridLayout.addWidget(self.label_50, 12, 1, 1, 1)

        self.label_36 = QLabel(self.tab_2)
        self.label_36.setObjectName(u"label_36")

        self.gridLayout.addWidget(self.label_36, 4, 0, 1, 1)

        self.line_10 = QFrame(self.tab_2)
        self.line_10.setObjectName(u"line_10")
        self.line_10.setFrameShape(QFrame.HLine)
        self.line_10.setFrameShadow(QFrame.Sunken)

        self.gridLayout.addWidget(self.line_10, 9, 1, 1, 1)

        self.label_51 = QLabel(self.tab_2)
        self.label_51.setObjectName(u"label_51")

        self.gridLayout.addWidget(self.label_51, 12, 2, 1, 1)

        self.verticalSpacer_17 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.gridLayout.addItem(self.verticalSpacer_17, 3, 0, 1, 1)

        self.label_38 = QLabel(self.tab_2)
        self.label_38.setObjectName(u"label_38")

        self.gridLayout.addWidget(self.label_38, 4, 2, 1, 1)

        self.label_46 = QLabel(self.tab_2)
        self.label_46.setObjectName(u"label_46")

        self.gridLayout.addWidget(self.label_46, 10, 1, 1, 1)

        self.label_33 = QLabel(self.tab_2)
        self.label_33.setObjectName(u"label_33")

        self.gridLayout.addWidget(self.label_33, 2, 0, 1, 1)

        self.label_21 = QLabel(self.tab_2)
        self.label_21.setObjectName(u"label_21")

        self.gridLayout.addWidget(self.label_21, 0, 0, 1, 1)

        self.label_37 = QLabel(self.tab_2)
        self.label_37.setObjectName(u"label_37")

        self.gridLayout.addWidget(self.label_37, 4, 1, 1, 1)

        self.verticalSpacer_16 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.gridLayout.addItem(self.verticalSpacer_16, 1, 0, 1, 1)

        self.verticalSpacer_20 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.gridLayout.addItem(self.verticalSpacer_20, 9, 0, 1, 1)

        self.line_4 = QFrame(self.tab_2)
        self.line_4.setObjectName(u"line_4")
        self.line_4.setFrameShape(QFrame.HLine)
        self.line_4.setFrameShadow(QFrame.Sunken)

        self.gridLayout.addWidget(self.line_4, 3, 1, 1, 1)

        self.label_47 = QLabel(self.tab_2)
        self.label_47.setObjectName(u"label_47")

        self.gridLayout.addWidget(self.label_47, 10, 2, 1, 1)

        self.line_6 = QFrame(self.tab_2)
        self.line_6.setObjectName(u"line_6")
        self.line_6.setFrameShape(QFrame.HLine)
        self.line_6.setFrameShadow(QFrame.Sunken)

        self.gridLayout.addWidget(self.line_6, 5, 1, 1, 1)

        self.label_34 = QLabel(self.tab_2)
        self.label_34.setObjectName(u"label_34")

        self.gridLayout.addWidget(self.label_34, 2, 1, 1, 1)

        self.verticalSpacer_19 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.gridLayout.addItem(self.verticalSpacer_19, 7, 0, 1, 1)

        self.line_9 = QFrame(self.tab_2)
        self.line_9.setObjectName(u"line_9")
        self.line_9.setFrameShape(QFrame.HLine)
        self.line_9.setFrameShadow(QFrame.Sunken)

        self.gridLayout.addWidget(self.line_9, 7, 2, 1, 1)

        self.line_3 = QFrame(self.tab_2)
        self.line_3.setObjectName(u"line_3")
        self.line_3.setFrameShape(QFrame.HLine)
        self.line_3.setFrameShadow(QFrame.Sunken)

        self.gridLayout.addWidget(self.line_3, 1, 2, 1, 1)

        self.line_12 = QFrame(self.tab_2)
        self.line_12.setObjectName(u"line_12")
        self.line_12.setFrameShape(QFrame.HLine)
        self.line_12.setFrameShadow(QFrame.Sunken)

        self.gridLayout.addWidget(self.line_12, 11, 1, 1, 1)

        self.label_35 = QLabel(self.tab_2)
        self.label_35.setObjectName(u"label_35")

        self.gridLayout.addWidget(self.label_35, 2, 2, 1, 1)

        self.label_42 = QLabel(self.tab_2)
        self.label_42.setObjectName(u"label_42")

        self.gridLayout.addWidget(self.label_42, 8, 0, 1, 1)

        self.label_43 = QLabel(self.tab_2)
        self.label_43.setObjectName(u"label_43")

        self.gridLayout.addWidget(self.label_43, 8, 1, 1, 1)

        self.line_11 = QFrame(self.tab_2)
        self.line_11.setObjectName(u"line_11")
        self.line_11.setFrameShape(QFrame.HLine)
        self.line_11.setFrameShadow(QFrame.Sunken)

        self.gridLayout.addWidget(self.line_11, 9, 2, 1, 1)

        self.line_13 = QFrame(self.tab_2)
        self.line_13.setObjectName(u"line_13")
        self.line_13.setFrameShape(QFrame.HLine)
        self.line_13.setFrameShadow(QFrame.Sunken)

        self.gridLayout.addWidget(self.line_13, 11, 2, 1, 1)

        self.label_22 = QLabel(self.tab_2)
        self.label_22.setObjectName(u"label_22")

        self.gridLayout.addWidget(self.label_22, 0, 1, 1, 1)

        self.label_45 = QLabel(self.tab_2)
        self.label_45.setObjectName(u"label_45")

        self.gridLayout.addWidget(self.label_45, 10, 0, 1, 1)


        self.verticalLayout.addLayout(self.gridLayout)

        self.label_24 = QLabel(self.tab_2)
        self.label_24.setObjectName(u"label_24")

        self.verticalLayout.addWidget(self.label_24)

        self.textEdit = QTextEdit(self.tab_2)
        self.textEdit.setObjectName(u"textEdit")

        self.verticalLayout.addWidget(self.textEdit)

        self.horizontalLayout_29 = QHBoxLayout()
        self.horizontalLayout_29.setObjectName(u"horizontalLayout_29")
        self.pushButton_4 = QPushButton(self.tab_2)
        self.pushButton_4.setObjectName(u"pushButton_4")

        self.horizontalLayout_29.addWidget(self.pushButton_4)

        self.pushButton_7 = QPushButton(self.tab_2)
        self.pushButton_7.setObjectName(u"pushButton_7")

        self.horizontalLayout_29.addWidget(self.pushButton_7)

        self.pushButton_5 = QPushButton(self.tab_2)
        self.pushButton_5.setObjectName(u"pushButton_5")

        self.horizontalLayout_29.addWidget(self.pushButton_5)

        self.pushButton_8 = QPushButton(self.tab_2)
        self.pushButton_8.setObjectName(u"pushButton_8")

        self.horizontalLayout_29.addWidget(self.pushButton_8)

        self.horizontalSpacer_8 = QSpacerItem(88, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_29.addItem(self.horizontalSpacer_8)


        self.verticalLayout.addLayout(self.horizontalLayout_29)


        self.horizontalLayout_33.addLayout(self.verticalLayout)

        self.tabWidget.addTab(self.tab_2, "")
        self.tab = QWidget()
        self.tab.setObjectName(u"tab")
        self.horizontalLayout_24 = QHBoxLayout(self.tab)
        self.horizontalLayout_24.setObjectName(u"horizontalLayout_24")
        self.verticalLayout_7 = QVBoxLayout()
        self.verticalLayout_7.setObjectName(u"verticalLayout_7")
        self.formLayout_3 = QFormLayout()
        self.formLayout_3.setObjectName(u"formLayout_3")
        self.label_5 = QLabel(self.tab)
        self.label_5.setObjectName(u"label_5")

        self.formLayout_3.setWidget(0, QFormLayout.LabelRole, self.label_5)

        self.horizontalLayout_8 = QHBoxLayout()
        self.horizontalLayout_8.setObjectName(u"horizontalLayout_8")
        self.lineEdit_1 = QLineEdit(self.tab)
        self.lineEdit_1.setObjectName(u"lineEdit_1")

        self.horizontalLayout_8.addWidget(self.lineEdit_1)

        self.label_8 = QLabel(self.tab)
        self.label_8.setObjectName(u"label_8")

        self.horizontalLayout_8.addWidget(self.label_8)


        self.formLayout_3.setLayout(0, QFormLayout.FieldRole, self.horizontalLayout_8)

        self.verticalSpacer_10 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.formLayout_3.setItem(1, QFormLayout.LabelRole, self.verticalSpacer_10)

        self.label_6 = QLabel(self.tab)
        self.label_6.setObjectName(u"label_6")

        self.formLayout_3.setWidget(2, QFormLayout.LabelRole, self.label_6)

        self.horizontalLayout_19 = QHBoxLayout()
        self.horizontalLayout_19.setObjectName(u"horizontalLayout_19")
        self.lineEdit_2 = QLineEdit(self.tab)
        self.lineEdit_2.setObjectName(u"lineEdit_2")

        self.horizontalLayout_19.addWidget(self.lineEdit_2)

        self.label_9 = QLabel(self.tab)
        self.label_9.setObjectName(u"label_9")

        self.horizontalLayout_19.addWidget(self.label_9)


        self.formLayout_3.setLayout(2, QFormLayout.FieldRole, self.horizontalLayout_19)

        self.verticalSpacer_11 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.formLayout_3.setItem(3, QFormLayout.LabelRole, self.verticalSpacer_11)

        self.label_7 = QLabel(self.tab)
        self.label_7.setObjectName(u"label_7")

        self.formLayout_3.setWidget(4, QFormLayout.LabelRole, self.label_7)

        self.horizontalLayout_20 = QHBoxLayout()
        self.horizontalLayout_20.setObjectName(u"horizontalLayout_20")
        self.lineEdit_3 = QLineEdit(self.tab)
        self.lineEdit_3.setObjectName(u"lineEdit_3")

        self.horizontalLayout_20.addWidget(self.lineEdit_3)

        self.label_10 = QLabel(self.tab)
        self.label_10.setObjectName(u"label_10")

        self.horizontalLayout_20.addWidget(self.label_10)


        self.formLayout_3.setLayout(4, QFormLayout.FieldRole, self.horizontalLayout_20)

        self.verticalSpacer_12 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.formLayout_3.setItem(5, QFormLayout.LabelRole, self.verticalSpacer_12)

        self.label_11 = QLabel(self.tab)
        self.label_11.setObjectName(u"label_11")

        self.formLayout_3.setWidget(8, QFormLayout.LabelRole, self.label_11)

        self.horizontalLayout_21 = QHBoxLayout()
        self.horizontalLayout_21.setObjectName(u"horizontalLayout_21")
        self.lineEdit_4 = QLineEdit(self.tab)
        self.lineEdit_4.setObjectName(u"lineEdit_4")

        self.horizontalLayout_21.addWidget(self.lineEdit_4)

        self.label_12 = QLabel(self.tab)
        self.label_12.setObjectName(u"label_12")

        self.horizontalLayout_21.addWidget(self.label_12)


        self.formLayout_3.setLayout(8, QFormLayout.FieldRole, self.horizontalLayout_21)

        self.verticalSpacer_13 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.formLayout_3.setItem(9, QFormLayout.LabelRole, self.verticalSpacer_13)

        self.label_14 = QLabel(self.tab)
        self.label_14.setObjectName(u"label_14")

        self.formLayout_3.setWidget(10, QFormLayout.LabelRole, self.label_14)

        self.horizontalLayout_25 = QHBoxLayout()
        self.horizontalLayout_25.setObjectName(u"horizontalLayout_25")
        self.lineEdit = QLineEdit(self.tab)
        self.lineEdit.setObjectName(u"lineEdit")

        self.horizontalLayout_25.addWidget(self.lineEdit)

        self.label_15 = QLabel(self.tab)
        self.label_15.setObjectName(u"label_15")

        self.horizontalLayout_25.addWidget(self.label_15)


        self.formLayout_3.setLayout(10, QFormLayout.FieldRole, self.horizontalLayout_25)

        self.verticalSpacer_14 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.formLayout_3.setItem(11, QFormLayout.LabelRole, self.verticalSpacer_14)

        self.label_16 = QLabel(self.tab)
        self.label_16.setObjectName(u"label_16")

        self.formLayout_3.setWidget(12, QFormLayout.LabelRole, self.label_16)

        self.horizontalLayout_26 = QHBoxLayout()
        self.horizontalLayout_26.setObjectName(u"horizontalLayout_26")
        self.lineEdit_5 = QLineEdit(self.tab)
        self.lineEdit_5.setObjectName(u"lineEdit_5")

        self.horizontalLayout_26.addWidget(self.lineEdit_5)

        self.label_17 = QLabel(self.tab)
        self.label_17.setObjectName(u"label_17")

        self.horizontalLayout_26.addWidget(self.label_17)


        self.formLayout_3.setLayout(12, QFormLayout.FieldRole, self.horizontalLayout_26)

        self.verticalSpacer_15 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.formLayout_3.setItem(7, QFormLayout.LabelRole, self.verticalSpacer_15)

        self.label_18 = QLabel(self.tab)
        self.label_18.setObjectName(u"label_18")

        self.formLayout_3.setWidget(6, QFormLayout.LabelRole, self.label_18)

        self.horizontalLayout_27 = QHBoxLayout()
        self.horizontalLayout_27.setObjectName(u"horizontalLayout_27")
        self.lineEdit_6 = QLineEdit(self.tab)
        self.lineEdit_6.setObjectName(u"lineEdit_6")

        self.horizontalLayout_27.addWidget(self.lineEdit_6)

        self.label_19 = QLabel(self.tab)
        self.label_19.setObjectName(u"label_19")

        self.horizontalLayout_27.addWidget(self.label_19)


        self.formLayout_3.setLayout(6, QFormLayout.FieldRole, self.horizontalLayout_27)

        self.verticalSpacer_22 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.formLayout_3.setItem(13, QFormLayout.LabelRole, self.verticalSpacer_22)

        self.label_25 = QLabel(self.tab)
        self.label_25.setObjectName(u"label_25")

        self.formLayout_3.setWidget(14, QFormLayout.LabelRole, self.label_25)

        self.horizontalLayout_31 = QHBoxLayout()
        self.horizontalLayout_31.setObjectName(u"horizontalLayout_31")
        self.lineEdit_9 = QLineEdit(self.tab)
        self.lineEdit_9.setObjectName(u"lineEdit_9")

        self.horizontalLayout_31.addWidget(self.lineEdit_9)

        self.pushButton_9 = QPushButton(self.tab)
        self.pushButton_9.setObjectName(u"pushButton_9")

        self.horizontalLayout_31.addWidget(self.pushButton_9)


        self.formLayout_3.setLayout(14, QFormLayout.FieldRole, self.horizontalLayout_31)

        self.verticalSpacer_23 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.formLayout_3.setItem(15, QFormLayout.LabelRole, self.verticalSpacer_23)

        self.label_26 = QLabel(self.tab)
        self.label_26.setObjectName(u"label_26")

        self.formLayout_3.setWidget(16, QFormLayout.LabelRole, self.label_26)

        self.lineEdit_10 = QLineEdit(self.tab)
        self.lineEdit_10.setObjectName(u"lineEdit_10")

        self.formLayout_3.setWidget(16, QFormLayout.FieldRole, self.lineEdit_10)

        self.verticalSpacer_24 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.formLayout_3.setItem(17, QFormLayout.LabelRole, self.verticalSpacer_24)

        self.label_27 = QLabel(self.tab)
        self.label_27.setObjectName(u"label_27")

        self.formLayout_3.setWidget(18, QFormLayout.LabelRole, self.label_27)

        self.lineEdit_11 = QLineEdit(self.tab)
        self.lineEdit_11.setObjectName(u"lineEdit_11")

        self.formLayout_3.setWidget(18, QFormLayout.FieldRole, self.lineEdit_11)

        self.verticalSpacer_25 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.formLayout_3.setItem(19, QFormLayout.LabelRole, self.verticalSpacer_25)

        self.label_28 = QLabel(self.tab)
        self.label_28.setObjectName(u"label_28")

        self.formLayout_3.setWidget(20, QFormLayout.LabelRole, self.label_28)

        self.lineEdit_12 = QLineEdit(self.tab)
        self.lineEdit_12.setObjectName(u"lineEdit_12")

        self.formLayout_3.setWidget(20, QFormLayout.FieldRole, self.lineEdit_12)

        self.verticalSpacer_26 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.formLayout_3.setItem(21, QFormLayout.LabelRole, self.verticalSpacer_26)

        self.label_29 = QLabel(self.tab)
        self.label_29.setObjectName(u"label_29")

        self.formLayout_3.setWidget(22, QFormLayout.LabelRole, self.label_29)

        self.horizontalLayout_32 = QHBoxLayout()
        self.horizontalLayout_32.setObjectName(u"horizontalLayout_32")
        self.lineEdit_13 = QLineEdit(self.tab)
        self.lineEdit_13.setObjectName(u"lineEdit_13")

        self.horizontalLayout_32.addWidget(self.lineEdit_13)

        self.label_30 = QLabel(self.tab)
        self.label_30.setObjectName(u"label_30")

        self.horizontalLayout_32.addWidget(self.label_30)


        self.formLayout_3.setLayout(22, QFormLayout.FieldRole, self.horizontalLayout_32)

        self.verticalSpacer_27 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.formLayout_3.setItem(23, QFormLayout.LabelRole, self.verticalSpacer_27)

        self.label_31 = QLabel(self.tab)
        self.label_31.setObjectName(u"label_31")

        self.formLayout_3.setWidget(24, QFormLayout.LabelRole, self.label_31)

        self.horizontalLayout_34 = QHBoxLayout()
        self.horizontalLayout_34.setObjectName(u"horizontalLayout_34")
        self.lineEdit_7 = QLineEdit(self.tab)
        self.lineEdit_7.setObjectName(u"lineEdit_7")

        self.horizontalLayout_34.addWidget(self.lineEdit_7)

        self.label_32 = QLabel(self.tab)
        self.label_32.setObjectName(u"label_32")

        self.horizontalLayout_34.addWidget(self.label_32)


        self.formLayout_3.setLayout(24, QFormLayout.FieldRole, self.horizontalLayout_34)


        self.verticalLayout_7.addLayout(self.formLayout_3)

        self.horizontalLayout_23 = QHBoxLayout()
        self.horizontalLayout_23.setObjectName(u"horizontalLayout_23")
        self.pushButton_3 = QPushButton(self.tab)
        self.pushButton_3.setObjectName(u"pushButton_3")

        self.horizontalLayout_23.addWidget(self.pushButton_3)

        self.pushButton_19 = QPushButton(self.tab)
        self.pushButton_19.setObjectName(u"pushButton_19")

        self.horizontalLayout_23.addWidget(self.pushButton_19)

        self.pushButton_20 = QPushButton(self.tab)
        self.pushButton_20.setObjectName(u"pushButton_20")

        self.horizontalLayout_23.addWidget(self.pushButton_20)

        self.horizontalSpacer_7 = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_23.addItem(self.horizontalSpacer_7)


        self.verticalLayout_7.addLayout(self.horizontalLayout_23)


        self.horizontalLayout_24.addLayout(self.verticalLayout_7)

        self.verticalLayout_6 = QVBoxLayout()
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.horizontalLayout_22 = QHBoxLayout()
        self.horizontalLayout_22.setObjectName(u"horizontalLayout_22")
        self.label_13 = QLabel(self.tab)
        self.label_13.setObjectName(u"label_13")

        self.horizontalLayout_22.addWidget(self.label_13)

        self.horizontalSpacer_6 = QSpacerItem(408, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_22.addItem(self.horizontalSpacer_6)


        self.verticalLayout_6.addLayout(self.horizontalLayout_22)

        self.log_2 = QTextEdit(self.tab)
        self.log_2.setObjectName(u"log_2")

        self.verticalLayout_6.addWidget(self.log_2)


        self.horizontalLayout_24.addLayout(self.verticalLayout_6)

        self.tabWidget.addTab(self.tab, "")

        self.verticalLayout_8.addWidget(self.tabWidget)


        self.retranslateUi(widget)

        self.tabWidget.setCurrentIndex(0)
        self.tabWidget_2.setCurrentIndex(0)

        # 定义滑块和对应标签的映射
        self.slider_label_map = {
            self.horizontalSlider_1: self.label_rain_1,
            self.horizontalSlider_2: self.label_rain_2,
            self.horizontalSlider_3: self.label_rain_3,
            self.horizontalSlider_4: self.label_rain_4,
            self.horizontalSlider_5: self.label_rain_5,
            self.horizontalSlider_6: self.label_rain_6,
            self.horizontalSlider_7: self.label_rain_7,
            self.horizontalSlider_8: self.label_rain_8,
                                }
        self.slider_label_map_1 = {
            self.horizontalSlider: self.label_54,
            self.horizontalSlider_12: self.label_75,
                                }

        for slider, label in self.slider_label_map.items():
            slider.setRange(0, 100)  # 设置范围为0-100
            slider.setValue(0)       # 初始值设为0
            slider.valueChanged.connect(lambda value, l=label: self.update_slider_label(value, l))
        
        for slider, label in self.slider_label_map_1.items():
            slider.setRange(0, 10)  # 设置范围为0-10
            slider.setValue(0)       # 初始值设为0
            slider.valueChanged.connect(lambda value, l=label: self.update_slider_label_1(value, l))
        
        self.slider_label_map_wind = {
            self.horizontalSlider_9: self.label_rain_9,
            self.horizontalSlider_10: self.label_rain_10,
            self.horizontalSlider_11: self.label_rain_11,
                                }
        
        for slider, label in self.slider_label_map_wind.items():
            slider.setRange(-20, 20)  #风速范围设定
            slider.setValue(0)
            slider.valueChanged.connect(lambda value, l=label: self.update_slider_label_wind(value, l)) 

        self.lineEdit_1.setText(str(X_MAX))  # X_MAX 默认值
        self.lineEdit_2.setText(str(Y_MAX))  # Y_MAX 默认值
        self.lineEdit_3.setText(str(- Z_MAX))  # Z_MAX 默认值
        self.lineEdit_4.setText(str(l))  # l 默认值
        self.lineEdit.setText(str(angle)) # angle 默认值
        self.lineEdit_5.setText(str(tip)) # tip 默认值
        self.lineEdit_6.setText(str(speed)) # speed 默认值
        self.lineEdit_9.setText(str(R_diag_global))
        self.lineEdit_10.setText(str(r_pixel))
        self.lineEdit_11.setText(str(q_a_global))
        self.lineEdit_12.setText(str(q_alpha_global))
        self.lineEdit_13.setText(str(manual_delay)) # manual_delay 默认值
        self.lineEdit_7.setText(str(manual_delay_1)) # manual_delay 默认值
        #self.lineEdit_8.setText(str(zaosheng)) # 噪声默认值
        #self.lineEdit_15.setText(str(zaosheng_1)) # 突发噪声默认值

        self.camera_buttons = {
            0: self.pushButton_16,
            1: self.pushButton_12,
            2: self.pushButton_10,
            3: self.pushButton_14,
            4: self.pushButton_15,
            5: self.pushButton_6,
            6: self.pushButton_11,
            7: self.pushButton_17
        }
        
        # label 与 camera 索引的对应关系
        self.camera_labels = {
            0: self.label_63,
            1: self.label_64,
            2: self.label_65,
            3: self.label_66,
            4: self.label_67,
            5: self.label_68,
            6: self.label_69,
            7: self.label_70
        }
    
        # 为每个按钮绑定点击事件
        for idx, button in self.camera_buttons.items():
            button.clicked.connect(lambda *args, i=idx: self.toggle_camera_channel(i))
            # 设置初始文本
            button.setText("未开启")
        
        # 更新所有 label 的初始状态
        self.update_camera_labels()

        QMetaObject.connectSlotsByName(widget)
    # 设置UI界面

    def retranslateUi(self, widget):
        widget.setWindowTitle(QCoreApplication.translate("widget", u"Shining\u4eff\u771f\u5e73\u53f0\u63a7\u5236\u7aef", None))
        self.button1.setText(QCoreApplication.translate("widget", u"\u542f\u52a8\u6570\u636e\u6536\u96c6\u4eff\u771f\u7a97\u53e3", None))
        self.button2.setText(QCoreApplication.translate("widget", u"\u65e0\u4eba\u673a\u4e00\u952e\u6362\u88c5", None))
        self.boolweather.setText(QCoreApplication.translate("widget", u"\u662f\u5426\u542f\u7528\u5929\u6c14\u53d8\u5316", None))
        self.label1.setText(QCoreApplication.translate("widget", u"\u96e8", None))
        self.label_rain_1.setText(QCoreApplication.translate("widget", u"0%", None))
        self.label2.setText(QCoreApplication.translate("widget", u"\u9053\u8def\u6e7f\u5ea6", None))
        self.label_rain_2.setText(QCoreApplication.translate("widget", u"0%", None))
        self.label3.setText(QCoreApplication.translate("widget", u"\u96ea", None))
        self.label_rain_3.setText(QCoreApplication.translate("widget", u"0%", None))
        self.label4.setText(QCoreApplication.translate("widget", u"\u9053\u8def\u79ef\u96ea", None))
        self.label_rain_4.setText(QCoreApplication.translate("widget", u"0%", None))
        self.label5.setText(QCoreApplication.translate("widget", u"\u843d\u53f6", None))
        self.label_rain_5.setText(QCoreApplication.translate("widget", u"0%", None))
        self.label6.setText(QCoreApplication.translate("widget", u"\u9053\u8def\u79ef\u53f6", None))
        self.label_rain_6.setText(QCoreApplication.translate("widget", u"0%", None))
        self.label7.setText(QCoreApplication.translate("widget", u"\u7070\u5ea6", None))
        self.label8.setText(QCoreApplication.translate("widget", u"\u96fe\u5ea6", None))
        self.label_rain_7.setText(QCoreApplication.translate("widget", u"0%", None))
        self.label_rain_8.setText(QCoreApplication.translate("widget", u"0%", None))
        self.label.setText(QCoreApplication.translate("widget", u"\u98ce\u5411\u53d8\u5316", None))
        self.X_lable.setText(QCoreApplication.translate("widget", u"X", None))
        self.label_rain_9.setText(QCoreApplication.translate("widget", u"0m/s", None))
        self.Y_lable.setText(QCoreApplication.translate("widget", u"Y", None))
        self.label_rain_10.setText(QCoreApplication.translate("widget", u"0m/s", None))
        self.Z_lable.setText(QCoreApplication.translate("widget", u"Z", None))
        self.label_rain_11.setText(QCoreApplication.translate("widget", u"0m/s", None))
        self.label_2.setText(QCoreApplication.translate("widget", u"\u8c03\u8bd5\u4fe1\u606f", None))
        self.apply.setText(QCoreApplication.translate("widget", u"\u5e94\u7528\u73af\u5883", None))
        self.start.setText(QCoreApplication.translate("widget", u"\u5f00\u59cb\u6536\u96c6\u6570\u636e", None))
        self.pushButton.setText(QCoreApplication.translate("widget", u"\u505c\u6b62\u6570\u636e\u6536\u96c6", None))
        self.pushButton_2.setText(QCoreApplication.translate("widget", u"\u6e05\u9664\u65e7\u6570\u636e\u96c6", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_1), QCoreApplication.translate("widget", u"\u751f\u6210\u4eff\u771f\u6570\u636e", None))
        self.label_3.setText(QCoreApplication.translate("widget", u"<html><head/><body><p align=\"center\">\u8bf7\u6253\u5f00\u6570\u636e\u6536\u96c6\u4eff\u771f\u7a97\u53e3</p></body></html>", None))
        self.tabWidget_2.setTabText(self.tabWidget_2.indexOf(self.tab_4), QCoreApplication.translate("widget", u"\u771f\u5b9e\u753b\u9762", None))
        self.label_20.setText(QCoreApplication.translate("widget", u"<html><head/><body><p align=\"center\">\u8fde\u63a5\u4e2d</p></body></html>", None))
        self.tabWidget_2.setTabText(self.tabWidget_2.indexOf(self.tab_5), QCoreApplication.translate("widget", u"\u7a7a\u95f43D\u9aa8\u67b6", None))
        self.label_4.setText(QCoreApplication.translate("widget", u"<html><head/><body><p align=\"center\">\u6682\u65e0\u98de\u884c\u8bb0\u5f55</p></body></html>", None))
        self.tabWidget_2.setTabText(self.tabWidget_2.indexOf(self.tab_3), QCoreApplication.translate("widget", u"\u5dee\u503c\u6570\u636e\u6c47\u603b", None))
        self.label_52.setText(QCoreApplication.translate("widget", u"\u6444\u50cf\u5934\u8fc7\u6ee4\u7f51", None))
        self.pushButton_6.setText(QCoreApplication.translate("widget", u"\u672a\u5f00\u542f", None))
        self.pushButton_10.setText(QCoreApplication.translate("widget", u"\u672a\u5f00\u542f", None))
        self.pushButton_11.setText(QCoreApplication.translate("widget", u"\u672a\u5f00\u542f", None))
        self.pushButton_12.setText(QCoreApplication.translate("widget", u"\u672a\u5f00\u542f", None))
        self.pushButton_13.setText(QCoreApplication.translate("widget", u"\u65e0\u4eba\u673a", None))
        self.pushButton_14.setText(QCoreApplication.translate("widget", u"\u672a\u5f00\u542f", None))
        self.pushButton_15.setText(QCoreApplication.translate("widget", u"\u672a\u5f00\u542f", None))
        self.pushButton_16.setText(QCoreApplication.translate("widget", u"\u672a\u5f00\u542f", None))
        self.pushButton_17.setText(QCoreApplication.translate("widget", u"\u672a\u5f00\u542f", None))
        self.label_71.setText(QCoreApplication.translate("widget", u"\u72b6\u6001", None))
        self.label_55.setText(QCoreApplication.translate("widget", u"\u6b63\u540e\u65b9\uff1a", None))
        self.label_56.setText(QCoreApplication.translate("widget", u"\u6b63\u5de6\u65b9\uff1a", None))
        self.label_57.setText(QCoreApplication.translate("widget", u"\u6b63\u524d\u65b9\uff1a", None))
        self.label_58.setText(QCoreApplication.translate("widget", u"\u6b63\u53f3\u65b9\uff1a", None))
        self.label_59.setText(QCoreApplication.translate("widget", u"\u5de6\u540e\u65b9\uff1a", None))
        self.label_60.setText(QCoreApplication.translate("widget", u"\u5de6\u524d\u65b9\uff1a", None))
        self.label_61.setText(QCoreApplication.translate("widget", u"\u53f3\u524d\u65b9\uff1a", None))
        self.label_62.setText(QCoreApplication.translate("widget", u"\u53f3\u540e\u65b9\uff1a", None))
        self.label_63.setText(QCoreApplication.translate("widget", u"\u901a\u9053\u5f00\u653e", None))
        self.label_64.setText(QCoreApplication.translate("widget", u"\u901a\u9053\u5f00\u653e", None))
        self.label_65.setText(QCoreApplication.translate("widget", u"\u901a\u9053\u5f00\u653e", None))
        self.label_66.setText(QCoreApplication.translate("widget", u"\u901a\u9053\u5f00\u653e", None))
        self.label_67.setText(QCoreApplication.translate("widget", u"\u901a\u9053\u5f00\u653e", None))
        self.label_68.setText(QCoreApplication.translate("widget", u"\u901a\u9053\u5f00\u653e", None))
        self.label_69.setText(QCoreApplication.translate("widget", u"\u901a\u9053\u5f00\u653e", None))
        self.label_70.setText(QCoreApplication.translate("widget", u"\u901a\u9053\u5f00\u653e", None))
        self.radioButton.setText(QCoreApplication.translate("widget", u"\u542f\u7528\u968f\u673a\u6bdb\u523a", None))
        self.label_53.setText(QCoreApplication.translate("widget", u"\u6574\u4f53\u566a\u58f0\u5927\u5c0f", None))
        self.label_54.setText(QCoreApplication.translate("widget", u"0 pixel", None))
        self.label_74.setText(QCoreApplication.translate("widget", u"\u7a81\u53d1\u566a\u58f0\u5927\u5c0f", None))
        self.label_75.setText(QCoreApplication.translate("widget", u"0 pixel", None))
        self.label_72.setText(QCoreApplication.translate("widget", u"\u6570\u636e\u6270\u52a8\u9891\u7387\uff1a", None))
        self.label_73.setText(QCoreApplication.translate("widget", u"\u5e27", None))
        self.pushButton_18.setText(QCoreApplication.translate("widget", u"\u5e94\u7528\u53c2\u6570", None))
        self.tabWidget_2.setTabText(self.tabWidget_2.indexOf(self.tab_6), QCoreApplication.translate("widget", u"\u6d88\u878d\u5b9e\u9a8c", None))
        self.label_48.setText(QCoreApplication.translate("widget", u"\u76d1\u6d4b\u6570\u636e", None))
        self.label_23.setText(QCoreApplication.translate("widget", u"cm", None))
        self.label_40.setText(QCoreApplication.translate("widget", u"0", None))
        self.label_44.setText(QCoreApplication.translate("widget", u"\u5ea6", None))
        self.label_41.setText(QCoreApplication.translate("widget", u"\u5ea6", None))
        self.label_49.setText(QCoreApplication.translate("widget", u"\u5e27\u7387\uff1a", None))
        self.label_39.setText(QCoreApplication.translate("widget", u"\u4fef\u4ef0\uff1a", None))
        self.label_50.setText(QCoreApplication.translate("widget", u"0", None))
        self.label_36.setText(QCoreApplication.translate("widget", u"z:", None))
        self.label_51.setText(QCoreApplication.translate("widget", u"fps", None))
        self.label_38.setText(QCoreApplication.translate("widget", u"cm", None))
        self.label_46.setText(QCoreApplication.translate("widget", u"0", None))
        self.label_33.setText(QCoreApplication.translate("widget", u"y:", None))
        self.label_21.setText(QCoreApplication.translate("widget", u"x:", None))
        self.label_37.setText(QCoreApplication.translate("widget", u"0", None))
        self.label_47.setText(QCoreApplication.translate("widget", u"\u5ea6", None))
        self.label_34.setText(QCoreApplication.translate("widget", u"0", None))
        self.label_35.setText(QCoreApplication.translate("widget", u"cm", None))
        self.label_42.setText(QCoreApplication.translate("widget", u"\u6a2a\u6447\uff1a", None))
        self.label_43.setText(QCoreApplication.translate("widget", u"0", None))
        self.label_22.setText(QCoreApplication.translate("widget", u"0", None))
        self.label_45.setText(QCoreApplication.translate("widget", u"\u504f\u822a\uff1a", None))
        self.label_24.setText(QCoreApplication.translate("widget", u"\u8c03\u8bd5\u4fe1\u606f", None))
        self.pushButton_4.setText(QCoreApplication.translate("widget", u"\u5f00\u59cb/\u505c\u6b62\u98de\u884c", None))
        self.pushButton_7.setText(QCoreApplication.translate("widget", u"\u6253\u5f00/\u5173\u95ed\u771f\u5b9e\u753b\u9762\u76d1\u89c6\u5668", None))
        self.pushButton_5.setText(QCoreApplication.translate("widget", u"\u5bfc\u5165\u65b0\u6846\u67b6", None))
        self.pushButton_8.setText(QCoreApplication.translate("widget", u"\u7ed3\u679c\u67e5\u770b", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), QCoreApplication.translate("widget", u"\u7b97\u6cd5\u7ed3\u679c\u68c0\u9a8c", None))
        self.label_5.setText(QCoreApplication.translate("widget", u"\u65e0\u4eba\u673a\u98de\u884cX\u8303\u56f4\uff1a", None))
        self.label_8.setText(QCoreApplication.translate("widget", u"m", None))
        self.label_6.setText(QCoreApplication.translate("widget", u"\u65e0\u4eba\u673a\u98de\u884cY\u8303\u56f4\uff1a", None))
        self.label_9.setText(QCoreApplication.translate("widget", u"m", None))
        self.label_7.setText(QCoreApplication.translate("widget", u"\u65e0\u4eba\u673a\u98de\u884c\u9ad8\u5ea6\u8303\u56f4\uff1a", None))
        self.label_10.setText(QCoreApplication.translate("widget", u"m", None))
        self.label_11.setText(QCoreApplication.translate("widget", u"\u76f8\u673a\u8ddd\u79bb\u4e16\u754c\u539f\u70b9\u4f4d\u7f6e\uff1a", None))
        self.label_12.setText(QCoreApplication.translate("widget", u"cm", None))
        self.label_14.setText(QCoreApplication.translate("widget", u"\u6444\u50cf\u673a\u4fef\u4ef0\u89d2", None))
        self.label_15.setText(QCoreApplication.translate("widget", u"\u5ea6", None))
        self.label_16.setText(QCoreApplication.translate("widget", u"\u62cd\u7167\u95f4\u9694", None))
        self.label_17.setText(QCoreApplication.translate("widget", u"s", None))
        self.label_18.setText(QCoreApplication.translate("widget", u"\u65e0\u4eba\u673a\u8fd0\u884c\u901f\u5ea6", None))
        self.label_19.setText(QCoreApplication.translate("widget", u"m/s", None))
        self.label_25.setText(QCoreApplication.translate("widget", u"<html><head/><body><p>\u7a7a\u95f4\u6d4b\u91cf\u566a\u58f0\u65b9\u5dee\uff08cm<span style=\" vertical-align:super;\">2</span>\uff09</p></body></html>", None))
        self.pushButton_9.setText(QCoreApplication.translate("widget", u"\u5e94\u7528\u4e0a\u6b21\u63a8\u8350\u503c", None))
        self.label_26.setText(QCoreApplication.translate("widget", u"<html><head/><body><p>\u56fe\u50cf\u6d4b\u91cf\u566a\u58f0\u6807\u51c6\u5dee\uff08pixel\uff09</p></body></html>", None))
        self.label_27.setText(QCoreApplication.translate("widget", u"<html><head/><body><p>\u52a0\u901f\u5ea6\u5bc6\u5ea6\u566a\u58f0\u8c31q<span style=\" vertical-align:sub;\">a</span></p></body></html>", None))
        self.label_28.setText(QCoreApplication.translate("widget", u"<html><head/><body><p>\u89d2\u52a0\u901f\u5ea6\u5bc6\u5ea6\u566a\u58f0\u8c31q<span style=\" vertical-align:sub;\">\u03b1</span></p></body></html>", None))
        self.label_29.setText(QCoreApplication.translate("widget", u"\u56fa\u6709\u5ef6\u8fdf\uff08LS+EKF\uff09", None))
        self.label_30.setText(QCoreApplication.translate("widget", u"s", None))
        self.label_31.setText(QCoreApplication.translate("widget", u"\u56fa\u6709\u5ef6\u8fdf\uff08EKF\uff09", None))
        self.label_32.setText(QCoreApplication.translate("widget", u"s", None))
        self.pushButton_3.setText(QCoreApplication.translate("widget", u"\u5e94\u7528\u53c2\u6570", None))
        self.pushButton_19.setText(QCoreApplication.translate("widget", u"\u7ed3\u679c\u67e5\u770b", None))
        self.pushButton_20.setText(QCoreApplication.translate("widget", u"\u6570\u636e\u5bfc\u51fa", None))
        self.label_13.setText(QCoreApplication.translate("widget", u"\u8c03\u8bd5\u4fe1\u606f", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), QCoreApplication.translate("widget", u"\u76f8\u673a\u53c2\u6570\u8c03\u6574", None))
    # 标签映射

    def open_collect_UE(self):
        # 定义可执行文件的完整路径
        exe_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'WindowsNoEditor', 'try2.exe')
        # 定义工作目录
        working_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'WindowsNoEditor')

        try:
            # 使用 subprocess.Popen 启动进程
            process = subprocess.Popen(exe_path, cwd=working_dir)
            print(f"成功启动 {exe_path}，进程 ID: {process.pid}")
        except FileNotFoundError:
            print(f"错误：找不到文件 {exe_path}")
        except Exception as e:
            print(f"启动失败，原因：{e}")
    # 打开UE进行数据收集

    def collect_data(self):
        """启动数据收集线程"""
        if self.check_existing_data():  # 检查是否存在数据文件
            reply = QMessageBox.question(self.window, "数据库存在旧数据", "是否要删除已有数据？", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

            if reply == QMessageBox.Yes:
                self.clear_old_data()
            else:
                return

        print("正在启动数据收集任务...")
        # 禁用按钮，防止线程多次启动
        self.start.setEnabled(False)
        self.apply.setEnabled(False)
        self.pushButton_2.setEnabled(False)
        self.pushButton_3.setEnabled(False)
        self.pushButton_4.setEnabled(False)
        self.pushButton_5.setEnabled(False)
        #self.pushButton_6.setEnabled(False)
        self.pushButton_7.setEnabled(False)

        # 检查是否已有线程在运行
        if hasattr(self, 'data_thread') and self.data_thread.isRunning():
            print("数据收集任务已在运行中")
            # 保持按钮禁用状态
            return

        # 检查截图功能是否已经开启
        file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'WindowsNoEditor', 'try2', 'connect', 'judge.json')
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                data = json.load(file)
                boolrun = data.get('judge', False)
        except Exception as e:
            print(f"读取文件失败: {e}")
            self.start.setEnabled(True)
            self.apply.setEnabled(True)
            self.pushButton_2.setEnabled(True)
            self.pushButton_3.setEnabled(True)
            self.pushButton_4.setEnabled(True)
            self.pushButton_5.setEnabled(True)
            #self.pushButton_6.setEnabled(True)
            self.pushButton_7.setEnabled(True)
            return

        if boolrun == True:
            print("数据收集任务已启动")
            # 创建并启动数据收集线程
            self.data_thread = DataCollectionThread()
            self.data_thread.finished.connect(self.on_data_collection_finished)
            self.data_thread.start()
            self.pushButton.clicked.connect(self.stop_data_collection)
        else:
            print("截图功能未启用！\n请在UE中打开截图功能！\n数据收集已暂停！")
            self.start.setEnabled(True)
            self.apply.setEnabled(True)
            self.pushButton_2.setEnabled(True)
            self.pushButton_3.setEnabled(True)
            self.pushButton_4.setEnabled(True)
            self.pushButton_5.setEnabled(True)
            #self.pushButton_6.setEnabled(True)
            self.pushButton_7.setEnabled(True)
    # 开启数据收集线程

    def stop_data_collection(self):
        """停止数据收集"""
        if hasattr(self, 'data_thread') and self.data_thread.isRunning():
            print("正在停止数据收集...")
            self.data_thread.stop()
            self.data_thread.quit()
            self.data_thread.wait()
            print("数据收集已停止")
    # 停止数据收集

    def on_data_collection_finished(self):
        """数据收集完成后的处理"""
        print("数据收集任务已完成")
        # 重新启用开始按钮
        self.start.setEnabled(True)
        self.apply.setEnabled(True)
        self.pushButton_2.setEnabled(True)
        self.pushButton_3.setEnabled(True)
        self.pushButton_4.setEnabled(True)
        self.pushButton_5.setEnabled(True)
        #self.pushButton_6.setEnabled(True)
        self.pushButton_7.setEnabled(True)

        # 断开可能存在的停止按钮连接，避免重复连接
        try:
            self.pushButton.clicked.disconnect(self.stop_data_collection)
        except RuntimeError:
            # 如果没有连接则忽略
            pass
    # 数据收集完成处理

    def update_slider_label(self, value, label):
        label.setText(f"{value}%")
    # 更新滑块标签

    def update_slider_label_1(self, value, label):
        label.setText(f"{value} pixel")
    # 更新滑块标签（消融实验部分）

    def update_slider_label_wind(self, value, label):
        label.setText(f"{value}m/s")
    # 更新风速标签😅，这破玩意就放主线程吧占不了多少算力

    def apply_weather(self):
        try:
            client = airsim.MultirotorClient()
            client.enableApiControl(True)
        except Exception as e:
            print(f"错误：无法连接到AirSim服务器，请检查仿真平台是否已启动并运行。\n{e}")
            return
        
        if self.boolweather.isChecked() == True:
            client.simEnableWeather(True)
            # 定义天气参数字典
            weather_params = {
                airsim.WeatherParameter.Rain: self.horizontalSlider_1.value() / 100.0,
                airsim.WeatherParameter.Roadwetness: self.horizontalSlider_2.value() / 100.0,
                airsim.WeatherParameter.Snow: self.horizontalSlider_3.value() / 100.0,
                airsim.WeatherParameter.RoadSnow: self.horizontalSlider_4.value() / 100.0,
                airsim.WeatherParameter.MapleLeaf: self.horizontalSlider_5.value() / 100.0,
                airsim.WeatherParameter.RoadLeaf: self.horizontalSlider_6.value() / 100.0,
                airsim.WeatherParameter.Dust: self.horizontalSlider_7.value() / 100.0,
                airsim.WeatherParameter.Fog: self.horizontalSlider_8.value() / 100.0,
                            }

            # 批量设置天气参数
            for param, value in weather_params.items():
                client.simSetWeatherParameter(param, value)

            # 设置风速
            client.simSetWind(airsim.Vector3r(self.horizontalSlider_9.value(), self.horizontalSlider_10.value(), self.horizontalSlider_11.value()))
            print("天气控制已应用！")
        else:
            print("天气控制未启用！")
            client.simEnableWeather(False)
    # 应用天气控制

    def clear_old_data(self):
        bugit_path = os.path.join(self.current_dir, 'WindowsNoEditor', 'try2', 'Saved', 'BugIt', 'WindowsNoEditor')
        connect_path = os.path.join(self.current_dir, 'WindowsNoEditor', 'try2', 'connect')
        recorder_path = os.path.join(self.current_dir, 'recorder')
    
        # 记录删除的文件数量
        deleted_files = 0
    
        try:
            # 检查并删除 BugIt 目录中的图片文件
            if os.path.exists(bugit_path):
                # 查找常见图片格式
                image_patterns = ['*.png', '*.jpg', '*.jpeg', '*.bmp', '*.tiff', '*.gif']
                for pattern in image_patterns:
                    image_files = glob.glob(os.path.join(bugit_path, pattern))
                    for file_path in image_files:
                        try:
                            os.remove(file_path)
                            deleted_files += 1
                            print(f"已删除图片文件: {file_path}")
                        except Exception as e:
                            print(f"删除图片文件失败 {file_path}: {e}")
        
            # 检查并删除 connect 目录中以 trigger 开头的文件
            if os.path.exists(connect_path):
                trigger_files = glob.glob(os.path.join(connect_path, 'trigger*'))
                for file_path in trigger_files:
                    try:
                        if os.path.isfile(file_path):  # 确保是文件而不是目录
                            os.remove(file_path)
                            deleted_files += 1
                            print(f"已删除trigger文件: {file_path}")
                    except Exception as e:
                        print(f"删除trigger文件失败 {file_path}: {e}")
        
            # 检查并删除 recorder 目录中以 record 开头的文件
            if os.path.exists(recorder_path):
                record_files = glob.glob(os.path.join(recorder_path, 'record*'))
                for file_path in record_files:
                    try:
                        if os.path.isfile(file_path):  # 确保是文件而不是目录
                            os.remove(file_path)
                            deleted_files += 1
                            print(f"已删除record文件: {file_path}")
                    except Exception as e:
                        print(f"删除record文件失败 {file_path}: {e}")
        
            # 根据删除结果弹出相应提示
            if deleted_files > 0:
                QMessageBox.information(self.window, "清理完成", f"已成功删除 {deleted_files} 个旧文件。")
                print(f"清理完成，共删除 {deleted_files} 个文件。")
            else:
                QMessageBox.information(self.window, "无需清理", "仓库干净，可以进行数据收集。")
                print("仓库干净，可以进行数据收集。")
            
        except Exception as e:
            error_msg = f"清理过程中出现错误: {e}"
            print(error_msg)
            QMessageBox.warning(self.window, "清理失败", error_msg)
    # 删除旧数据

    def check_existing_data(self):
        """
        检查是否存在旧数据文件，但不删除
        返回True表示存在旧数据，返回False表示无旧数据
        """
        bugit_path = os.path.join(self.current_dir, 'WindowsNoEditor', 'try2', 'Saved', 'BugIt', 'WindowsNoEditor')
        connect_path = os.path.join(self.current_dir, 'WindowsNoEditor', 'try2', 'connect')
        recorder_path = os.path.join(self.current_dir, 'recorder')
    
        # 检查 BugIt 目录中的图片文件
        if os.path.exists(bugit_path):
            image_patterns = ['*.png', '*.jpg', '*.jpeg', '*.bmp', '*.tiff', '*.gif']
            for pattern in image_patterns:
                if glob.glob(os.path.join(bugit_path, pattern)):
                    return True  # 发现图片文件
    
        # 检查 connect 目录中以 trigger 开头的文件
        if os.path.exists(connect_path):
            if glob.glob(os.path.join(connect_path, 'trigger*')):
                return True  # 发现trigger文件
    
        # 检查 recorder 目录中以 record 开头的文件
        if os.path.exists(recorder_path):
            if glob.glob(os.path.join(recorder_path, 'record*')):
                return True  # 发现record文件
    
        return False  # 未发现任何旧数据文件
    # 检查旧数据是否存在

    def camera_param(self):
        # 检查截图功能是否已经开启
        file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'WindowsNoEditor', 'try2', 'connect', 'judge.json')
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                data = json.load(file)
                boolrun = data.get('judge', False)
        except Exception as e:
            print(f"读取文件失败: {e}")
            return

        if boolrun == True:
            print("截图功能已启用！\n相机参数已锁死！")
            return
        
        global_import.X_MAX = float(self.lineEdit_1.text())
        global_import.X_MIN = -float(self.lineEdit_1.text())
        global_import.Y_MAX = float(self.lineEdit_2.text())
        global_import.Y_MIN = -float(self.lineEdit_2.text())
        global_import.Z_MAX = - float(self.lineEdit_3.text())
        global_import.l = float(self.lineEdit_4.text())
        global_import.angle = float(self.lineEdit.text())
        global_import.tip = self.lineEdit_5.text()
        global_import.speed = float(self.lineEdit_6.text())
        global_import.R_diag_global = float(self.lineEdit_9.text())
        global_import.r_pixel = float(self.lineEdit_10.text())
        global_import.q_a_global = float(self.lineEdit_11.text())
        global_import.q_alpha_global = float(self.lineEdit_12.text())
        global_import.manual_delay = float(self.lineEdit_13.text())
        global_import.manual_delay_1 = float(self.lineEdit_7.text())

        # 创建统一的通信目录
        comm_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'WindowsNoEditor', "try2" , "connect")
        #comm_dir = "D:\\Unreal Projects\\try2\\connect"
        if not os.path.exists(comm_dir):
            os.makedirs(comm_dir)
    
        # 创建触发文件
        trigger_file = os.path.join(comm_dir, "camera.json")
    
        data = {
                "l":  f" -{self.lineEdit_4.text()}",
                "angle": f" -{self.lineEdit.text()}",
                "tip": f" {self.lineEdit_5.text()}",
                "speed": f" {self.lineEdit_6.text()}"
            }
    
        with open(trigger_file, 'w') as f:
            json.dump(data, f, indent=2)
    
        print(f"相机距离文件已覆写: {trigger_file}")

        create_camera_trigger_file()
    # 进行相机参数调整

    def toggle_camera_channel(self, index):
        if 0 <= index < 8:
            # 切换状态
            camera[index] = 1 - camera[index]
            
            # 更新按钮文本
            button = self.camera_buttons[index]
            if camera[index] == 1:
                button.setText("开启")
            else:
                button.setText("未开启")
            
            # 更新对应 label 状态
            self.update_camera_labels()
            
            print(f"相机通道 {index} 已{'开启' if camera[index] == 1 else '关闭'}")
    # 消融实验按钮切换控制

    def update_camera_labels(self):
        for idx, label in self.camera_labels.items():
            if camera[idx] == 1:
                label.setText("通道关闭")
            else:
                label.setText("通道开放")
    # 消融实验标签切换

    def change_clothes(self):
        file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'WindowsNoEditor')
        if not os.path.exists(file_path):
            os.makedirs(file_path)

        if self.clothes == 0:
            trigger_file = os.path.join(file_path, "settings.json")
    
            data = {
                "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md",
                "SettingsVersion": 1.2,
                "SimMode":"Multirotor",
                "PawnPaths":
			    {"DefaultQuadrotor" : {"PawnBP": "Class'/Game/StarterContent/Blueprints/UI_way/BP_FlyingPawn.BP_FlyingPawn_C'"}
			    }
                    }
            with open(trigger_file, 'w') as f:
                json.dump(data, f, indent=2)
            
            self.clothes += 1
            return

        if self.clothes % 2 == 0:
            trigger_file = os.path.join(file_path, "settings.json")
    
            data = {
                "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md",
                "SettingsVersion": 1.2,
                "SimMode":"Multirotor",
                "PawnPaths":
			    {"DefaultQuadrotor" : {"PawnBP": "Class'/Game/StarterContent/Blueprints/UI_way/BP_FlyingPawn.BP_FlyingPawn_C'"}
			    }
                    }
            QMessageBox.information(self.window, "切换成功", "已切换外形为“DJI MINI4 Pro”\n切换地图以实现模型刷新")
            print("已切换外形为“DJI MINI4 Pro\n切换地图以实现模型刷新”")
        else:
            trigger_file = os.path.join(file_path, "settings.json")
    
            data = {
                "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md",
                "SettingsVersion": 1.2,
                "SimMode":"Multirotor"
                    }
            QMessageBox.information(self.window, "切换成功", "已切换外形为“默认”\n切换地图以实现模型刷新")
            print("已切换外形为“默认”\n切换地图以实现模型刷新")
    
        with open(trigger_file, 'w') as f:
            json.dump(data, f, indent=2)
        
        self.clothes += 1
    # 换无人机外形😭其实改的是配置文件，皮肤都在UE4的包里

    def vedio_param(self):
        if self.video_thread and self.video_thread.isRunning():
            self.stop_video_capture()
            self.video_thread = None # 停止线程
        else:
            self.start_video_capture()
    # 用于捕获UE4仿真窗口画面实现一个实时播放，控制开始和结束

    def initial_judge(self):
        file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'WindowsNoEditor', 'try2', 'connect', 'judge.json')
        if not os.path.exists(file_path):
            os.makedirs(os.path.dirname(file_path), exist_ok=True)
            default_data = {"judge": False}  # 默认设为 False
        
            try:
                with open(file_path, 'w', encoding='utf-8') as file:
                    json.dump(default_data, file, indent=2)
                print(f"已创建默认的 judge.json 文件: {file_path}")
            except Exception as e:
                print(f"创建默认文件失败: {e}")
    # 初始化judge文件，修复初始状态参数锁死问题😊随手关门好习惯

    def setup_video_display(self):
        # 设置label_3的初始状态
        self.label_3.setText("视频未启动")
        self.label_3.setAlignment(Qt.AlignCenter)
        self.label_3.setStyleSheet("background-color: black; color: white;")
    # 设置视频显示初始化

    def stop_video_capture(self):
        self.start.setEnabled(True)
        
        # 1. 先停止视频采集线程（源头）
        # 这步很重要，不再产生新的积压信号
        if self.video_thread and self.video_thread.isRunning():
            self.video_thread.running = False
            self.video_thread.quit()
            if not self.video_thread.wait(2000): # 等待视频线程安全退出
                print("警告：视频线程停止超时")
            self.video_thread = None

        # 2. 再停止 EKF 线程（处理端）
        if self.EKF_thread and self.EKF_thread.isRunning():
            self.EKF_thread.quit()
            
            # 【关键修改】给足时间！
            # 因为有积压，这里可能需要等几秒让它把队列里的旧数据算完
            # 如果不想等，可以加一个清空队列的逻辑（见下文）
            if not self.EKF_thread.wait(5000): # 给它 5 秒时间
                print("警告：EKF线程停止超时")
            self.EKF_thread = None
            
        print("所有线程已停止")
    # 停止捕获线程

    def start_video_capture(self):
        """启动视频捕获"""
        # 【安全检查】：防止重复启动或启动未完全清理的线程
        if self.video_thread and self.video_thread.isRunning():
            print("检测到上一个线程仍在运行，正在强制清理...")
            self.stop_video_capture()
            self.video_thread = None
            # 强制清理后稍作等待
            QThread.msleep(200)

        # 创建新线程
        self.video_thread = vedioThread()
        local_coords = np.vstack(([0,0,0], rotor_position))
        self.EKF_thread = TightlyCoupledEKF(local_coords=local_coords, 
                                            qa=global_import.q_a_global, 
                                            qalpha=global_import.q_alpha_global, 
                                            r_pixel=global_import.r_pixel
                                            )
        try:
            self.setup_matplotlib_3d_view()
            angle_val = float(self.lineEdit.text())
            l_val = float(self.lineEdit_4.text())
            #self.video_thread.initialize_3d_reconstruction()
            self.initialize_3d_reconstruction(angle_val, l_val, K)
            self.EKF_thread.initialize_3d_reconstruction(angle_val, l_val, K)
            camera_params_list = []
            for i in range(8): # 你有8个相机
                param = {
                    'R_wc': self.R_wc_matrices[i],    # 直接取预计算的旋转矩阵
                    't_wc': self.camera_positions[i], # 直接取预计算的位置
                    'K': K                            # 内参矩阵
                }
                camera_params_list.append(param)
            self.EKF_thread.set_camera_params(camera_params_list, width, height)
        except Exception as e:
            print(f"初始化3D参数失败: {e}")
            return
        
        self.start.setEnabled(False)
        
        # 连接信号
        self.video_thread.frame_ready.connect(self.display_video_frame)
        self.video_thread.error_occurred.connect(self.handle_video_error)
        self.video_thread.yolo_thread.points_3d_ready.connect(self.update_3d_points)
        self.video_thread.yolo_thread.points_3d_ready.connect(self.EKF_thread.process_step)
        self.EKF_thread.data.connect(self.save_EKF_data)
        
        # 启动线程
        self.video_thread.start()
        self.EKF_thread.start()
        
        # 初始化 EKF
        self.EKF = RigidBodyEKF(qa=global_import.q_a_global, 
                                qalpha=global_import.q_alpha_global, 
                                R_diag=global_import.R_diag_global
                                )
        self.EKF.set_local_coords(local_coords)

        print("视频捕获已启动")
    # 开启展示线程(有很多线程的启动都在这里，目光看过来❤️❤️❤️❤️❤️)

    def update_3d_points(self, points_3d, current_time):
        """更新3D点显示"""
        points_3d = self.reconstruct_3d_points_fast(points_3d) # 3D点重建
        self.reconstructed_points = points_3d
        self.label_22.setText(f"{points_3d[0][0]}")
        self.label_34.setText(f"{points_3d[0][1]}")
        self.label_37.setText(f"{points_3d[0][2]}")
        # 位姿结算
        R = self.attitude_determination(points_3d[0], points_3d[1:], rotor_position, current_time)
        # EKF滤波
        self.attitude_determination_EKF(points_3d[0], points_3d[1:], rotor_position, current_time, R)
        self.plot_3d_points()
    # 绘制更新

    def plot_3d_points(self):
        """在3D视图中绘制点 (高性能版)"""
        # 检查散点对象是否存在
        if not hasattr(self, 'scatter_item') or not self.reconstructed_points:
            return
                
        try:
            # 1. 整理数据：将列表转换为 (N, 3) 的 numpy 数组
            valid_points = []
            for point in self.reconstructed_points:
                if point is not None and not np.isnan(point).any():
                    valid_points.append(point)
            
            if not valid_points:
                return

            points_array = np.array(valid_points)
            points_array[:, 1] = -points_array[:, 1]  # Y轴取反，符合UE4坐标系
            
            # 2. 核心优化：只更新数据，不重绘场景！
            # 这行代码非常快，直接把新的坐标丢给显卡渲染
            self.scatter_item.setData(pos=points_array)
                
        except Exception as e:
            print(f"绘制3D点时出错: {e}")
    # 绘制

    def display_video_frame(self, frame, current_time):
        try:
            # 获取label_3的大小
            #label_size = self.label_3.size()
            #max_width = label_size.width()
            max_width = 1000
            #max_width = label_size.width()
            max_height = 1000
        
            if max_width <= 0 or max_height <= 0:
                return
            
            height, width = frame.shape[:2]
            
            scale_x = max_width / width
            scale_y = max_height / height
            scale = min(scale_x, scale_y)

            new_width = int(width * scale)
            new_height = int(height * scale)
        
            if new_width > 0 and new_height > 0:
                # 调整图像大小
                resized_frame = cv2.resize(frame, (new_width, new_height))
            
                # 转换为Qt图像格式
                height, width, channel = resized_frame.shape
                bytesPerLine = 3 * width
                qImg = QImage(resized_frame.data, width, height, bytesPerLine, QImage.Format_BGR888)
            
                # 显示图像
                pixmap = QPixmap.fromImage(qImg)
                self.label_3.setPixmap(pixmap)
                self.label_3.setAlignment(Qt.AlignCenter)
                self.label_50.setText(f"{1/(current_time - self.current_time)}")
                self.current_time = current_time
        
        except Exception as e:
            print(f"显示视频帧失败: {e}")
    # 显示视频帧

    def handle_video_error(self, error_msg):
        print(error_msg)
        self.label_3.setText(f"{error_msg}")
        self.label_3.setAlignment(Qt.AlignCenter)
        self.start.setEnabled(True)
    # 处理视频错误

    def setup_matplotlib_3d_view(self):   
        try:
            # 1. 创建 GLViewWidget (OpenGL 视图)
            self.view_3d = gl.GLViewWidget()
            self.view_3d.setBackgroundColor('k')  # 黑色背景，符合你之前的风格
            
            # 设置相机距离，确保能看到整个盒子
            limit = l * 1.1
            
            # 2. 【新增】绘制立体坐标轴盒子
            # 这将绘制一个类似 Matplotlib 的正方体框架
            self.draw_axis_box(limit)

            # 3. 【新增】添加网格平面
            # 在 Z=0 平面添加网格，方便观察高度
            grid = gl.GLGridItem()
            grid.setSize(limit*2, limit*2) # 设置网格大小
            grid.setSpacing(limit/5, limit/5) # 设置网格间距
            # 将网格稍微下沉一点点，避免遮挡 Z=0 的点
            grid.translate(0, 0, -0.01) 
            self.view_3d.addItem(grid)

            # 4. 初始化散点图对象
            self.scatter_item = gl.GLScatterPlotItem(pos=np.zeros((1,3)), color=(1,1,0,1), size=10)
            self.view_3d.addItem(self.scatter_item)

            # 5. 设置相机视角
            self.view_3d.setCameraPosition(distance=limit*4, elevation=20, azimuth=225)

            # 6. 嵌入到 label_20 控件中
            if hasattr(self, 'label_20') and self.label_20:
                layout = self.label_20.layout()
                if layout:
                    while layout.count():
                        child = layout.takeAt(0)
                        if child.widget():
                            child.widget().deleteLater()
                else:
                    layout = QVBoxLayout(self.label_20)
                    self.label_20.setLayout(layout)
                
                layout.addWidget(self.view_3d)
                layout.setContentsMargins(0, 0, 0, 0)
                print("PyQtGraph 3D视图（含立体坐标盒）已成功嵌入！")
                
        except Exception as e:
            print(f"PyQtGraph 3D视图设置失败: {e}")
            traceback.print_exc()
    # 相机设置

    def draw_axis_box(self, limit):
        """
        手动绘制类似 Matplotlib 的 3D 坐标轴盒子
        包括：12条边框线 + 3条坐标轴线 + 原点
        """
        # 定义正方体的8个顶点 (根据你的 limit 设置范围)
        # 注意：Matplotlib 通常范围是 [-limit, limit]
        s = limit # side length
        
        # 盒子顶点坐标
        verts = np.array([
            [-s, -s, -s], [s, -s, -s], [s, s, -s], [-s, s, -s], # 底面
            [-s, -s, s], [s, -s, s], [s, s, s], [-s, s, s]      # 顶面
        ])
        
        # 定义盒子的12条边 (连接哪两个顶点)
        edges = [
            (0,1), (1,2), (2,3), (3,0), # 底面边
            (4,5), (5,6), (6,7), (7,4), # 顶面边
            (0,4), (1,5), (2,6), (3,7)  # 立柱
        ]
        
        # 准备绘制数据：构建一个 (12*2, 3) 的数组
        lines = []
        for e in edges:
            lines.append(verts[e[0]])
            lines.append(verts[e[1]])
        
        lines = np.array(lines)
        
        # 绘制盒子边框 (白色，半透明，稍微细一点，作为背景参考)
        box_plot = gl.GLLinePlotItem(pos=lines, color=(1, 1, 1, 0.3), width=1, mode='lines')
        self.view_3d.addItem(box_plot)
        
        # 【重点】绘制加粗的彩色坐标轴线 (穿过中心或从原点出发)
        # Matplotlib 风格通常是轴线穿过中心，或者从最小面引出
        # 这里我们画三条从原点出发的粗线，更直观
        axis_len = limit # 轴长
        
        # X轴 (红色)
        x_axis = gl.GLLinePlotItem(pos=np.array([[0,0,0], [axis_len,0,0]]), color=(1,0,0,1), width=2)
        self.view_3d.addItem(x_axis)
        
        # Y轴 (绿色)
        y_axis = gl.GLLinePlotItem(pos=np.array([[0,0,0], [0,-axis_len,0]]), color=(0,1,0,1), width=2)
        self.view_3d.addItem(y_axis)
        
        # Z轴 (蓝色)
        z_axis = gl.GLLinePlotItem(pos=np.array([[0,0,0], [0,0,axis_len]]), color=(0,0,1,1), width=2)
        self.view_3d.addItem(z_axis)
    # 使用openGL创建一个可交互的3D视图并嵌入到骨架信息中

    def start_flight(self):
        if hasattr(self, 'fly_thread') and self.fly_thread is not None and self.fly_thread.isRunning():
            print("正在停止飞行...")
            self.fly_thread.stop()
            self.fly_thread.quit()
            self.fly_thread.wait()
            #self.start.setEnabled(True)
            self.apply.setEnabled(True)
            self.pushButton_2.setEnabled(True)
            self.pushButton_3.setEnabled(True)
            self.pushButton.setEnabled(True)
            self.pushButton_5.setEnabled(True)
            self.pushButton_9.setEnabled(True)
            self.pushButton_7.setEnabled(True)
            print("飞行展示已停止")
            self.stop_video_capture()
            self.video_thread = None

            #🧐这里加载一个绘图展示结算模块
            self.create_comparison_plot()
            self.suggset_R_diag()
            self.EKF = None
            self.init_R = None

        else:
            if self.video_thread is not None and self.video_thread.isRunning():
                mode_dialog = QMessageBox(self.parent() if hasattr(self, 'parent') else None)
                mode_dialog.setWindowTitle("选择飞行模式")
                mode_dialog.setText("请选择飞行模式:")
                mode_dialog.setIcon(QMessageBox.Question)

                # 动态添加按钮 - 可自由增减选项
                flight_modes = {
                    "random_waypoint": "随机航点飞行",
                    "circle_flight": "圆形环绕飞行", 
                    "figure_eight": "8 字轨迹飞行",
                    "hover_mode": "定点悬停",
                    "line_scan": "正方形飞行"
                }

                # 存储按钮引用以便后续识别
                self.mode_buttons = {}
                for mode_key, mode_name in flight_modes.items():
                    button = mode_dialog.addButton(mode_name, QMessageBox.ActionRole)
                    self.mode_buttons[button] = mode_key

                # 添加取消按钮
                cancel_button = mode_dialog.addButton("取消", QMessageBox.RejectRole)

                # 显示对话框并等待用户选择
                mode_dialog.exec()
                clicked_button = mode_dialog.clickedButton()

                # 获取用户选择的模式
                if clicked_button == cancel_button:
                    print("用户取消飞行任务")
                    return
                else:
                    flight_mode = self.mode_buttons.get(clicked_button, "random_waypoint")
                    print(f"已选择飞行模式：{flight_mode}")
                # ==================================================
                #self.EKF = RigidBodyEKF(qa=1.0, qalpha=1.0, R_diag=0.02**2)
                #local_coords = np.vstack(([0,0,0], rotor_position))
                #self.EKF.set_local_coords(local_coords)
                global plot_real_data
                global plot_yolo_data
                global plot_EKF_data
                global plot_finial_data
                plot_real_data = []
                plot_yolo_data = []
                plot_EKF_data = []
                plot_finial_data = []
                self.fly_thread = fly(self.window, flight_mode=flight_mode)
                self.fly_thread.start()
                #self.start.setEnabled(False)
                self.apply.setEnabled(False)
                self.pushButton_2.setEnabled(False)
                self.pushButton_3.setEnabled(False)
                self.pushButton.setEnabled(False)
                self.pushButton_5.setEnabled(False)
                self.pushButton_9.setEnabled(False)
                self.pushButton_7.setEnabled(False)
            else:
                wbbb = QMessageBox.information(self.window, "提示", "监视器窗口未打开！")
    # 控制UE4窗口开始飞行，控制开始和结束

    def attitude_determination(self, drone_center_world, rotor_world_positions, rotor_local_coords, yolo_time):
        # 1. 准备数据，转换为numpy数组并转置为 (3, N) 格式
        drone_center_np = np.array(drone_center_world)
        rotor_world_np = np.array(rotor_world_positions).T  # Shape: (3, N)
        rotor_local_np = np.array(rotor_local_coords).T    # Shape: (3, N)

        # 2. 计算旋翼相对于无人机中心的世界坐标向量
        rotor_world_relative_np = rotor_world_np - drone_center_np.reshape(3, 1)

        # 3. 使用Kabsch算法计算最佳旋转矩阵 R
        # 我们需要找到 R，使得 R @ P_local ≈ P_world_relative
        H = rotor_world_relative_np @ rotor_local_np.T
        
        U, S, Vt = np.linalg.svd(H)
        
        R = U @ Vt
        
        # 处理反射情况（确保行列式为1）
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = U @ Vt

        # 4. 从旋转矩阵 R (ZYX顺序) 中提取欧拉角
        pitch = np.arcsin(np.clip(R[2, 0], -1.0, 1.0))
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])

        # 存储带时间戳的YOLO姿态数据到 plot_yolo_data
        timestamped_yolo_data = {
            "timestamp": yolo_time,
            "drone_center": drone_center_world.tolist() if hasattr(drone_center_world, 'tolist') else drone_center_world,
            "rotor_world_positions": [pos.tolist() for pos in rotor_world_positions],  # 转换为列表以便JSON序列化
            "pitch": float(-pitch),
            "roll": float(roll),
            "yaw": float(yaw)
        }
        plot_yolo_data.append(timestamped_yolo_data)

        self.label_40.setText(f"{math.degrees(pitch)}")
        self.label_43.setText(f"{math.degrees(roll)}")
        self.label_46.setText(f"{math.degrees(yaw)}")

        return R
    # 反向结算姿态信息并显示(基础LS版本)

    def attitude_determination_EKF(self, drone_center_world, rotor_world_positions, rotor_local_coords, yolo_time, R = None):
        if self.EKF is None:
            print("错误：EKF 对象未创建,或飞行模块已开始结算")
            return
        if not hasattr(self.EKF, 'initialized') or not self.EKF.initialized:
            print("EKF 未初始化，正在执行初始化...")
            
            # 如果 R 为 None，使用单位矩阵作为初始旋转
            if R is None or not isinstance(R, np.ndarray):
                print("警告：R 矩阵为空，使用默认初始化")
                R = rotor_local_coords
            
            # 确保 drone_center_world 是有效的 numpy 数组
            if drone_center_world is None or len(drone_center_world) < 3:
                print("警告：无人机中心位置无效，使用默认值")
                drone_center_world = np.array([0.0, 0.0, 0.0])
            else:
                drone_center_world = np.array(drone_center_world[:3])
            
            try:
                # 执行 EKF 初始化
                self.EKF.initialize(center=drone_center_world, rotmat=R)
                self.init_R = R.copy() if hasattr(R, 'copy') else R
                print(f"EKF 初始化成功！初始位置：{drone_center_world}, 初始旋转矩阵:\n{R}")
            except Exception as e:
                print(f"EKF 初始化失败：{e}")
                traceback.print_exc()
                return
        # 第一帧初始化跟踪目标

        current_time = yolo_time
        center_np = np.array(drone_center_world).flatten()
        rotors_np = np.array(rotor_world_positions).flatten()
        z = np.hstack((center_np, rotors_np))
        filtered_pos, (roll, pitch, yaw) = self.EKF.step(z, current_time)

        q_filtered = self.EKF.get_filtered_quaternion()
        R_filtered = self.EKF._quat_to_rotmat(q_filtered)
        local_coords = self.EKF.local_coords  # (3,5)
        world_points = filtered_pos[:, None] + R_filtered @ local_coords  # (3,5)
        filtered_rotors = world_points[:, 1:].T  # (4,3) 四个旋翼

        timestamped_ekf_data = {
            "timestamp": current_time,
            "drone_center": filtered_pos.tolist(),
            "rotor_world_positions": [pos.tolist() for pos in filtered_rotors],  # 滤波后的旋翼位置
            "pitch": float(pitch),
            "roll": float(roll),
            "yaw": float(yaw)
        }
        plot_EKF_data.append(timestamped_ekf_data)    
    # LS处理多相机信息，EKF后端做平稳

    def upload_pt(self):
        # 打开文件选择对话框
        file_path, _ = QFileDialog.getOpenFileName(
            self.window,
            "选择PT文件",
            "",
            "PT Files (*.pt);;All Files (*)"
        )
        
        if file_path:
            try:
                # 获取文件名
                filename = os.path.basename(file_path)
                
                # 检查文件扩展名
                if not filename.endswith('.pt'):
                    QMessageBox.warning(self.window, "文件类型错误", "请选择.pt文件")
                    return
                
                # 目标路径
                target_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'WindowsNoEditor')
                target_path = os.path.join(target_dir, filename)
                
                # 确保目标目录存在
                if not os.path.exists(target_dir):
                    os.makedirs(target_dir)
                
                # 复制文件
                shutil.copy2(file_path, target_path)
                
                # 更新全局变量
                global pt_name
                pt_name = filename
                self.current_pt_name = filename
                
                # 显示成功消息
                QMessageBox.information(self.window, "上传成功", f"文件已上传:\n{filename}")
                print(f"PT文件已更新为: {filename}")
                
            except Exception as e:
                error_msg = f"上传文件失败: {str(e)}"
                print(error_msg)
                QMessageBox.critical(self.window, "上传失败", error_msg)
        else:
            print("文件选择已取消")
    # 提交新pt文件 

    def create_comparison_plot(self):
        try:
            # 清除 label_4 的现有内容
            self.clear_label_content(self.label_4)
            
            # 准备数据
            real_times, real_x, real_y, real_z, real_pitch, real_roll, real_yaw = self.extract_real_data()
            yolo_times, yolo_x, yolo_y, yolo_z, yolo_pitch, yolo_roll, yolo_yaw = self.extract_yolo_data()
            EKF_times, EKF_x, EKF_y, EKF_z, EKF_pitch, EKF_roll, EKF_yaw = self.extract_EKF_data()
            f_times, f_x, f_y, f_z, f_pitch, f_roll, f_yaw = self.extract_f_data()
            
            # 创建图表
            fig = Figure(figsize=(12, 8), dpi=100)
            fig.suptitle('真实数据 vs YOLO 估计数据对比', fontsize=16, fontproperties=self.get_chinese_font())
            
            # 创建 6 个子图（位置 + 姿态）
            axes = []
            for i in range(6):
                ax = fig.add_subplot(2, 3, i + 1)
                ax.set_facecolor("#ffffff")
                ax.tick_params(colors='black')
                ax.xaxis.label.set_color('black')
                ax.yaxis.label.set_color('black')
                for spine in ax.spines.values():
                    spine.set_color('black')
                axes.append(ax)
            
            # 绘制数据（位置对比）
            titles = ['X 位置对比', 'Y 位置对比', 'Z 位置对比', 
                    'Pitch 对比', 'Roll 对比', 'Yaw 对比']
            real_data = [real_x, real_y, real_z, real_pitch, real_roll, real_yaw]
            yolo_data = [yolo_x, yolo_y, yolo_z, yolo_pitch, yolo_roll, yolo_yaw]
            EKF_data = [EKF_x, EKF_y, EKF_z, EKF_pitch, EKF_roll, EKF_yaw]
            f_data = [f_x, f_y, f_z, f_pitch, f_roll, f_yaw]
            ylabels = ['X 位置', 'Y 位置', 'Z 位置', 
                    'Pitch (弧度)', 'Roll (弧度)', 'Yaw (弧度)']
            
            for i, ax in enumerate(axes):
                if real_times and real_data[i]:
                    ax.plot(real_times, real_data[i], color='#FF6B6B', linestyle='-', 
                        label='真实', linewidth=1)
                if yolo_times and yolo_data[i]:
                    ax.plot(yolo_times, yolo_data[i], color='#4ECDC4', linestyle='--', 
                        label='LS', linewidth=1)
                if EKF_times and EKF_data[i]:
                    ax.plot(EKF_times, EKF_data[i], color='#FFE66D', linestyle='-.', 
                        label='LS+EKF', linewidth=1)
                if f_times and f_data[i]:
                    ax.plot(f_times, f_data[i], color="#660FE8", linestyle='-.', 
                        label='EKF', linewidth=1)
                
                ax.set_title(titles[i], fontproperties=self.get_chinese_font())
                ax.set_xlabel('时间 (s)', fontproperties=self.get_chinese_font())
                ax.set_ylabel(ylabels[i], fontproperties=self.get_chinese_font())
                ax.legend(prop=self.get_chinese_font())
                ax.grid(True, alpha=0.3)
            
            fig.tight_layout()
            
            # 【关键】保存 Figure 引用供按钮调用
            self.comparison_fig = fig
            
            # 嵌入到 label_4
            canvas = FigureCanvas(fig)
            layout = self.label_4.layout()
            if layout:
                while layout.count():
                    child = layout.takeAt(0)
                    if child.widget():
                        child.widget().deleteLater()
            else:
                layout = QVBoxLayout(self.label_4)
                self.label_4.setLayout(layout)
            
            layout.addWidget(canvas)
            layout.setContentsMargins(0, 0, 0, 0)
            canvas.draw()
            
            print("对比图表已生成，点击按钮可查看大图")
            
        except Exception as e:
            print(f"创建对比图表时出错：{e}")
            traceback.print_exc()
    # 创建对比图表，显示真实数据和YOLO估计数据的对比
    
    def extract_real_data(self):
        """
        从plot_real_data中提取真实数据
        """
        times = []
        x_positions = []
        y_positions = []
        z_positions = []
        pitches = []
        rolls = []
        yaws = []
        
        try:
            for data in plot_real_data:
                if 'timestamp' in data and 'drone_pose' in data:
                    times.append(data['timestamp'])
                    
                    pose = data['drone_pose']
                    if 'position' in pose:
                        x_positions.append(pose['position'].get('x', 0) * 100)
                        y_positions.append(pose['position'].get('y', 0) * 100)
                        z_positions.append(pose['position'].get('z', 0) * 100)
                    
                    if 'orientation' in pose:
                        pitches.append(pose['orientation'].get('pitch', 0))
                        rolls.append(pose['orientation'].get('roll', 0))
                        yaws.append(pose['orientation'].get('yaw', 0))
        except Exception as e:
            print(f"提取真实数据时出错: {e}")
        
        return times, x_positions, y_positions, z_positions, pitches, rolls, yaws
    # 读取真实位姿与时间
    
    def extract_yolo_data(self):
        """
        从plot_yolo_data中提取YOLO数据
        注意：YOLO数据中没有位置信息，只有姿态信息
        """
        times = []
        x_positions = []  # YOLO数据中没有位置信息
        y_positions = []
        z_positions = []
        pitches = []
        rolls = []
        yaws = []
        
        try:
            for data in plot_yolo_data:
                if 'timestamp' in data:
                    times.append(data['timestamp'])
                    
                    # 提取位置信息
                    if 'drone_center' in data:
                        drone_center = data['drone_center']
                        if isinstance(drone_center, (list, tuple)) and len(drone_center) >= 3:
                            x_positions.append(drone_center[0])
                            y_positions.append(drone_center[1])
                            z_positions.append(drone_center[2])
                        else:
                            # 如果数据格式不正确，使用默认值
                            x_positions.append(0)
                            y_positions.append(0)
                            z_positions.append(0)
                    else:
                        # 如果没有位置信息，使用默认值
                        x_positions.append(0)
                        y_positions.append(0)
                        z_positions.append(0)
                    
                    # 提取姿态信息
                    pitches.append(data.get('pitch', 0))
                    rolls.append(data.get('roll', 0))
                    yaws.append(data.get('yaw', 0))
        except Exception as e:
            print(f"提取YOLO数据时出错: {e}")
        
        return times, x_positions, y_positions, z_positions, pitches, rolls, yaws
    # 读取估计位姿与时间

    def extract_EKF_data(self):
        """
        从plot_yolo_data中提取YOLO数据
        注意：YOLO数据中没有位置信息，只有姿态信息
        """
        times = []
        x_positions = []  # YOLO数据中没有位置信息
        y_positions = []
        z_positions = []
        pitches = []
        rolls = []
        yaws = []
        
        try:
            for data in plot_EKF_data:
                if 'timestamp' in data:
                    times.append(data['timestamp'])
                    
                    # 提取位置信息
                    if 'drone_center' in data:
                        drone_center = data['drone_center']
                        if isinstance(drone_center, (list, tuple)) and len(drone_center) >= 3:
                            x_positions.append(drone_center[0])
                            y_positions.append(drone_center[1])
                            z_positions.append(drone_center[2])
                        else:
                            # 如果数据格式不正确，使用默认值
                            x_positions.append(0)
                            y_positions.append(0)
                            z_positions.append(0)
                    else:
                        # 如果没有位置信息，使用默认值
                        x_positions.append(0)
                        y_positions.append(0)
                        z_positions.append(0)
                    
                    # 提取姿态信息
                    pitches.append(data.get('pitch', 0))
                    rolls.append(data.get('roll', 0))
                    yaws.append(data.get('yaw', 0))
        except Exception as e:
            print(f"提取YOLO数据时出错: {e}")
        
        return times, x_positions, y_positions, z_positions, pitches, rolls, yaws
    # 读取估计位姿与时间

    def extract_f_data(self):
        """
        从plot_yolo_data中提取YOLO数据
        注意：YOLO数据中没有位置信息，只有姿态信息
        """
        times = []
        x_positions = []  # YOLO数据中没有位置信息
        y_positions = []
        z_positions = []
        pitches = []
        rolls = []
        yaws = []
        
        try:
            for data in plot_finial_data:
                if 'timestamp' in data:
                    times.append(data['timestamp'])
                    
                    # 提取位置信息
                    if 'drone_center' in data:
                        drone_center = data['drone_center']
                        if isinstance(drone_center, (list, tuple)) and len(drone_center) >= 3:
                            x_positions.append(drone_center[0])
                            y_positions.append(drone_center[1])
                            z_positions.append(drone_center[2])
                        else:
                            # 如果数据格式不正确，使用默认值
                            x_positions.append(0)
                            y_positions.append(0)
                            z_positions.append(0)
                    else:
                        # 如果没有位置信息，使用默认值
                        x_positions.append(0)
                        y_positions.append(0)
                        z_positions.append(0)
                    
                    # 提取姿态信息
                    pitches.append(data.get('pitch', 0))
                    rolls.append(data.get('roll', 0))
                    yaws.append(data.get('yaw', 0))
        except Exception as e:
            print(f"提取YOLO数据时出错: {e}")
        
        return times, x_positions, y_positions, z_positions, pitches, rolls, yaws
    # 读取估计位姿与时间
    
    def clear_label_content(self, label):
        """
        清除label中的现有内容
        """
        try:
            layout = label.layout()
            if layout:
                while layout.count():
                    child = layout.takeAt(0)
                    if child.widget():
                        child.widget().deleteLater()
        except Exception as e:
            print(f"清除label内容时出错: {e}")
    # 报错信息label_4特供版

    def setup_matplotlib_chinese_support(self):
        try:
            # 尝试使用系统中文字体
            # Windows系统常用中文字体
            chinese_fonts = [
                'SimHei',      # 黑体
                'Microsoft YaHei',  # 微软雅黑
                'STHeiti',     # 华文黑体
                'SimSun',      # 宋体
                'FangSong',    # 仿宋
            ]
            
            # 查找系统可用字体
            available_fonts = [f.name for f in font_manager.fontManager.ttflist]
            
            # 寻找可用的中文字体
            found_chinese_font = None
            for font_name in chinese_fonts:
                if font_name in available_fonts:
                    found_chinese_font = font_name
                    break
            
            # 如果没找到预定义的中文字体，尝试查找任何包含中文字体名称的字体
            if found_chinese_font is None:
                chinese_keywords = ['黑体', '微软雅黑', '宋体', '仿宋', '楷体', '华文']
                for font_name in available_fonts:
                    if any(keyword in font_name for keyword in chinese_keywords):
                        found_chinese_font = font_name
                        break
            
            # 设置字体
            if found_chinese_font:
                plt.rcParams['font.sans-serif'] = [found_chinese_font]
                plt.rcParams['axes.unicode_minus'] = False  # 解决负号'-'显示为方块的问题
                print(f"已设置中文字体: {found_chinese_font}")
            else:
                # 如果找不到中文字体，使用默认字体但确保可以显示中文
                plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Bitstream Vera Sans', 'sans-serif']
                plt.rcParams['axes.unicode_minus'] = False
                print("未找到合适的中文字体，使用默认字体")
                
        except Exception as e:
            print(f"设置中文字体时出错: {e}")
            # 设置备选方案
            plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Bitstream Vera Sans', 'sans-serif']
            plt.rcParams['axes.unicode_minus'] = False
    # 设置Matplotlib中文字体支持

    def get_chinese_font(self):
        try:
            # 尝试获取中文字体
            chinese_fonts = ['SimHei', 'Microsoft YaHei', 'STHeiti', 'SimSun']
            for font_name in chinese_fonts:
                try:
                    return font_manager.FontProperties(fname=font_manager.findfont(font_manager.FontProperties(family=font_name)))
                except:
                    continue
            # 如果找不到中文字体，返回默认字体
            return font_manager.FontProperties()
        except:
            return font_manager.FontProperties()
    # 获取中文字体属性

    def suggset_R_diag(self):
        a = global_import.manual_delay
        b = global_import.manual_delay_1
        # 完整版本（带详细输出）
        #print(f"当前的“手动延迟”为：{manual_delay}s")
        #self.R_diag =
        self.calculate_R_diag(a, b)#manual_delay=0.06
        
        # 简化版本
        # R_diag_global = calculate_R_diag_summary()

        #print(f"R_diag 计算完成，形状：{self.R_diag.shape}")
    # 通过上一次飞行真值对比给出建议“建议噪声方差”
    
    def show_large_comparison_plot(self):
        self.suggset_R_diag()
        try:
            # 检查是否有保存的图表数据
            if self.comparison_fig is None:
                QMessageBox.warning(self.window, "提示", "请先生成对比图表")
                return
            
            import matplotlib.pyplot as plt
            delay = global_import.manual_delay
            delay_1 = global_import.manual_delay_1
            
            # 创建新的大尺寸图表
            fig_large = plt.figure(figsize=(16, 10), dpi=100)
            fig_large.canvas.manager.window.setWindowTitle("数据对比 - 详细视图（可缩放/保存）")
            
            # 从保存的 Figure 中获取数据重新绘制
            real_times, real_x, real_y, real_z, real_pitch, real_roll, real_yaw = self.extract_real_data()
            yolo_times, yolo_x, yolo_y, yolo_z, yolo_pitch, yolo_roll, yolo_yaw = self.extract_yolo_data()
            EKF_times, EKF_x, EKF_y, EKF_z, EKF_pitch, EKF_roll, EKF_yaw = self.extract_EKF_data()
            f_times, f_x, f_y, f_z, f_pitch, f_roll, f_yaw = self.extract_f_data()
            
            # 时间戳校正：减去系统延迟
            yolo_times = [t - delay for t in yolo_times]
            EKF_times = [t - delay for t in EKF_times]
            f_times = [t - delay_1 for t in f_times]

            # 创建 6 个子图
            axes = []
            for i in range(6):
                ax = fig_large.add_subplot(2, 3, i + 1)
                ax.set_facecolor("#ffffff")
                ax.tick_params(colors='black')
                ax.xaxis.label.set_color('black')
                ax.yaxis.label.set_color('black')
                for spine in ax.spines.values():
                    spine.set_color('black')
                axes.append(ax)
            
            titles = ['X 位置对比', 'Y 位置对比', 'Z 位置对比', 
                    'Pitch 对比', 'Roll 对比', 'Yaw 对比']
            real_data = [real_x, real_y, real_z, real_pitch, real_roll, real_yaw]
            yolo_data = [yolo_x, yolo_y, yolo_z, yolo_pitch, yolo_roll, yolo_yaw]
            EKF_data = [EKF_x, EKF_y, EKF_z, EKF_pitch, EKF_roll, EKF_yaw]
            f_data = [f_x, f_y, f_z, f_pitch, f_roll, f_yaw]
            ylabels = ['X 位置', 'Y 位置', 'Z 位置', 
                    'Pitch (弧度)', 'Roll (弧度)', 'Yaw (弧度)']
            
            for i, ax in enumerate(axes):
                if real_times and real_data[i]:
                    ax.plot(real_times, real_data[i], color='#FF6B6B', linestyle='-', 
                        label='真实', linewidth=2)
                if yolo_times and yolo_data[i]:
                    ax.plot(yolo_times, yolo_data[i], color='#4ECDC4', linestyle='--', 
                        label='LS', linewidth=2)
                if EKF_times and EKF_data[i]:
                    ax.plot(EKF_times, EKF_data[i], color='#FFE66D', linestyle='-.', 
                        label='EKF+LS', linewidth=2)
                if f_times and f_data[i]:
                    ax.plot(f_times, f_data[i], color="#660FE800", linestyle='-.', 
                        label='EKF', linewidth=1)
                
                ax.set_title(titles[i], fontproperties=self.get_chinese_font(), fontsize=14)
                ax.set_xlabel('时间 (s)', fontproperties=self.get_chinese_font())
                ax.set_ylabel(ylabels[i], fontproperties=self.get_chinese_font())
                ax.legend(prop=self.get_chinese_font())
                ax.grid(True, alpha=0.3)
            
            fig_large.suptitle('真实数据 vs YOLO 估计数据 vs EKF 滤波数据', 
                            fontsize=16, fontproperties=self.get_chinese_font())
            fig_large.tight_layout()
            
            # 非阻塞显示独立窗口
            plt.show(block=False)
            
            print("已打开对比图表大窗口")
            
        except Exception as e:
            print(f"显示大图时出错：{e}")
            import traceback
            traceback.print_exc()
    # 在新窗口中显示大尺寸对比图表

    def apply_R_diag(self):
        if self.R_diag is not None:
            global_import.R_diag_global = self.R_diag
            self.lineEdit_9.setText(str(self.R_diag))
            print(f"已应用 R_diag ：{self.R_diag}")
        else:
            print("未找到 R_diag ！\n请先进行飞行获取 R_diag ！")
    # 将 R_diag 应用到全局变量中

    def initialize_3d_reconstruction(self, angle_val, l_val, K_matrix):
        try:
            # 1. 预计算相机内参矩阵的逆
            self.K_inv = np.linalg.inv(K_matrix)
            
            # 2. 预计算相机位置（基于固定的l和angle）
            camera_positions_normalized = calculate_camera_position_bias(angle_val)
            # 缩放到实际位置 - 修复：确保每个位置都是numpy数组后再进行标量乘法
            self.camera_positions = [np.array(pos) * l_val for pos in camera_positions_normalized]
            
            # 3. 预计算每个相机的旋转矩阵
            self.R_wc_matrices = []
            for camera_pos in self.camera_positions:
                forward_vec = (-camera_pos) / np.linalg.norm(camera_pos)
                world_up_vec = np.array([0, 0, 1])
                right_vec = np.cross(forward_vec, world_up_vec)
                if np.linalg.norm(right_vec) < 1e-6:
                    right_vec = np.array([1, 0, 0])
                right_vec /= np.linalg.norm(right_vec)
                up_vec = np.cross(right_vec, forward_vec)
                R_wc = np.vstack((right_vec, up_vec, forward_vec))  # R_wc: World to Camera
                #down_vec = np.cross(forward_vec, right_vec)  # 让 Y 轴向下，完美匹配图像坐标系
                #R_wc = np.vstack((right_vec, down_vec, forward_vec))
                self.R_wc_matrices.append(R_wc)
            
            print("3D重建参数预计算完成")
            
        except Exception as e:
            print(f"初始化3D重建参数时出错: {e}")
            traceback.print_exc()
    # 在开始工作前预计算所有不变量

    def reconstruct_3d_points_fast(self, all_points):
        """
        优化的3D点重建方法，使用预计算的参数
        """
        if self.camera_positions is None or self.K_inv is None or self.R_wc_matrices is None:
            print("3D重建参数未初始化")
            return [np.array([np.nan, np.nan, np.nan]) for _ in range(5)]
        
        reconstructed_points = []

        # 2. 遍历每一个物体（共5个）
        for j in range(5):
            A_matrix = []
            b_vector = []

            # 3. 遍历每一个相机（共8个），收集观测到该物体的光线
            for i in range(8):
                point_2d = all_points[i][j]
                
                # 假设 (0, 0) 是无效点，跳过
                if point_2d is None or (isinstance(point_2d, (list, tuple, np.ndarray)) and point_2d[0] == 0 and point_2d[1] == 0):
                    continue

                # --- 使用预计算的参数进行逆向投影 ---

                # a. 获取相机位置 (已预计算)
                camera_pos = self.camera_positions[i]
                
                # b. 逆向图像坐标翻转
                u_original = width - point_2d[0]
                v_original = height - point_2d[1]
                #u_original = point_2d[0]
                #v_original = point_2d[1]

                # c. 逆向投影：从像素坐标得到相机坐标系下的射线方向
                ray_cam = self.K_inv @ np.array([u_original, v_original, 1])
                
                # d. 使用预计算的旋转矩阵将射线从相机坐标系转换到世界坐标系
                ray_world = self.R_wc_matrices[i].T @ ray_cam
                ray_world = ray_world / np.linalg.norm(ray_world)

                # e. 构建线性系统 (使用叉积方程)
                dx, dy, dz = ray_world
                A_i = np.array([
                    [0, -dz, dy],
                    [dz, 0, -dx],
                    [-dy, dx, 0]
                ])
                b_i = np.cross(camera_pos, ray_world)
                
                A_matrix.append(A_i)
                b_vector.append(b_i)

            # 4. 如果收集到足够的光线，则计算最优交点
            if len(A_matrix) >= 2:  # 至少需要两个相机
                A = np.vstack(A_matrix)
                b = np.hstack(b_vector)
                
                try:
                    # 使用最小二乘法求解超定方程组 A * P = b
                    point_3d, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
                    reconstructed_points.append(point_3d * -1)
                    #reconstructed_points.append(point_3d)
                except np.linalg.LinAlgError:
                    print(f"物体 {j+1} 的线性方程求解失败。")
                    reconstructed_points.append(np.array([np.nan, np.nan, np.nan]))
            else:
                print(f"物体 {j+1} 的有效观测点不足，无法重建。")
                reconstructed_points.append(np.array([np.nan, np.nan, np.nan]))
                
        return reconstructed_points
    # 重建3D点

    def save_EKF_data(self, z, current_time):
        filtered_pos = z[:3]
        q = z[6:10]
        roll, pitch, yaw = TightlyCoupledEKF._quat_to_euler(q)
        R_filtered = TightlyCoupledEKF._quat_to_rotmat(q)
        local_coords = self.EKF_thread.local_coords
        world_points = filtered_pos[:, np.newaxis] + R_filtered @ local_coords
        filtered_rotors = world_points[:, 1:].T

        timestamped_ekf_data = {
            "timestamp": current_time,
            "drone_center": filtered_pos.tolist(),
            "rotor_world_positions": [pos.tolist() for pos in filtered_rotors],  # 滤波后的旋翼位置
            "pitch": float(pitch),
            "roll": float(roll),
            "yaw": float(yaw)
        }
        plot_finial_data.append(timestamped_ekf_data)
    # 保存紧耦合EKF数据

    def calculate_R_diag(self, manual_delay, manual_delay_1):
        """
        手动设置延迟补偿量，并计算像素误差 (RMSE, Mean, Std)
        """
        print(f"=== 开始计算：手动补偿 LS+EKF 延迟 {manual_delay}s，紧耦合 EKF 延迟 {manual_delay_1}s ===")
        
        # ==================== 1. 数据提取 ====================
        # (保持你原有的数据提取代码不变)
        camera_params_list = []
        for i in range(8): 
            param = {
                        'R_wc': self.R_wc_matrices[i],
                        't_wc': self.camera_positions[i],
                        'K': global_import.K
                        }
            camera_params_list.append(param)

        t_real = np.array([d['timestamp'] for d in plot_real_data])
        real_positions = np.array([
            [d['drone_pose']['position']['x'] * 100,
            d['drone_pose']['position']['y'] * 100,
            d['drone_pose']['position']['z'] * 100]
            for d in plot_real_data
        ])
        real_attitudes = np.array([
            [d['drone_pose']['orientation']['pitch'],
            d['drone_pose']['orientation']['roll'],
            d['drone_pose']['orientation']['yaw']]
            for d in plot_real_data
        ])
        real_attitudes = np.unwrap(real_attitudes, axis=0)  # 解开姿态的周期性

        t_pred = np.array([d['timestamp'] for d in plot_yolo_data])
        pred_points_all = []
        pred_points_all_EKF = []
        pred_points_all_EKF_f = []
        for d in plot_yolo_data:
            center = np.array(d['drone_center'])
            rotors = np.array(d['rotor_world_positions'])
            pred_points_all.append(np.vstack([center, rotors]))
        for d in plot_EKF_data:    
            center_EKF = np.array(d['drone_center'])
            rotors_EKF = np.array(d['rotor_world_positions'])
            pred_points_all_EKF.append(np.vstack([center_EKF, rotors_EKF]))
        for d in plot_finial_data:    
            center_EKF_f = np.array(d['drone_center'])
            rotors_EKF_f = np.array(d['rotor_world_positions'])
            pred_points_all_EKF_f.append(np.vstack([center_EKF_f, rotors_EKF_f]))
        pred_points_all = np.array(pred_points_all)
        pred_points_all_EKF = np.array(pred_points_all_EKF)
        pred_points_all_EKF_f = np.array(pred_points_all_EKF_f)

        #t_real = t_real - t_real[0]
        #t_pred = t_pred - t_pred[0]

        # ==================== 2. 应用补偿 ====================
        t_pred_corrected = t_pred - manual_delay      
        t_pred_corrected_1 = t_pred - manual_delay_1  
        
        # ==================== 3. 插值准备 ====================
        interp_x = interp1d(t_real, real_positions[:, 0], kind='linear', bounds_error=False, fill_value="extrapolate")
        interp_y = interp1d(t_real, real_positions[:, 1], kind='linear', bounds_error=False, fill_value="extrapolate")
        interp_z = interp1d(t_real, real_positions[:, 2], kind='linear', bounds_error=False, fill_value="extrapolate")
        interp_p = interp1d(t_real, real_attitudes[:, 0], kind='linear', bounds_error=False, fill_value="extrapolate")
        interp_r = interp1d(t_real, real_attitudes[:, 1], kind='linear', bounds_error=False, fill_value="extrapolate")
        interp_yw = interp1d(t_real, real_attitudes[:, 2], kind='linear', bounds_error=False, fill_value="extrapolate")

        local_coords = np.vstack([[0, 0, 0], rotor_position])
        
        # 只保留像素误差列表: [u_err, v_err, point_idx]
        errors_pixl = []
        errors_EKF_pixl = []
        errors_EKF_f_pixl = []
        
        plot_pred = []
        plot_pred_EKF = []
        plot_pred_EKF_f = []
        plot_true = []

        valid_start = max(t_real[0], t_pred_corrected[0])
        valid_end = min(t_real[-1], t_pred_corrected[-1])
        valid_start_1 = max(t_real[0], t_pred_corrected_1[0])
        valid_end_1 = min(t_real[-1], t_pred_corrected_1[-1])

        # ==================== 4. 误差计算 ====================
        # (这里的循环逻辑不变，为了完整性简写)
        for i, t_corr in enumerate(t_pred_corrected):
            if t_corr < valid_start or t_corr > valid_end: continue
            
            pred_pts = pred_points_all[i]
            pred_pts_EKF = pred_points_all_EKF[i]
            
            # 真值计算
            true_x, true_y, true_z = interp_x(t_corr), interp_y(t_corr), interp_z(t_corr)
            true_center = np.array([true_x, true_y, true_z])
            pitch, roll, yaw = interp_p(t_corr), interp_r(t_corr), interp_yw(t_corr)
            R = euler_to_rotmat(pitch, roll, yaw)
            true_points = true_center + (R @ local_coords.T).T
            #true_points[:, 2] *= -1
            
            # 用于画图
            plot_pred.append(pred_pts[0])
            plot_pred_EKF.append(pred_pts_EKF[0])
            plot_true.append(true_points[0])
            
            # 像素误差计算
            for cam in camera_params_list:
                R_wc, t_wc, K = cam['R_wc'], cam['t_wc'], cam['K']
                for k in range(5):
                    # 真值投影
                    p_c_true = R_wc @ (true_points[k] - t_wc)
                    if p_c_true[2] < 0.1: continue
                    u_true = width - (K[0,0] * (p_c_true[0] / p_c_true[2]) + K[0,2])
                    #u_true = (K[0,0] * (p_c_true[0] / p_c_true[2]) + K[0,2])
                    #v_true = (K[1,1] * (p_c_true[1] / p_c_true[2]) + K[1,2])
                    v_true = height - (K[1,1] * (p_c_true[1] / p_c_true[2]) + K[1,2])
                    
                    # LS 投影
                    p_c_pred = R_wc @ (pred_pts[k] - t_wc)
                    if p_c_pred[2] < 0.1: continue
                    u_pred = width - (K[0,0] * (p_c_pred[0] / p_c_pred[2]) + K[0,2])
                    v_pred = height - (K[1,1] * (p_c_pred[1] / p_c_pred[2]) + K[1,2])
                    errors_pixl.append([u_true - u_pred, v_true - v_pred, k])
                    
                    # LS+EKF 投影
                    p_c_ekf = R_wc @ (pred_pts_EKF[k] - t_wc)
                    if p_c_ekf[2] < 0.1: continue
                    u_ekf = width - (K[0,0] * (p_c_ekf[0] / p_c_ekf[2]) + K[0,2])
                    v_ekf = height - (K[1,1] * (p_c_ekf[1] / p_c_ekf[2]) + K[1,2])
                    errors_EKF_pixl.append([u_true - u_ekf, v_true - v_ekf, k])

        # ==================== 5. 紧耦合 EKF 误差计算 ====================
        for i, t_corr_1 in enumerate(t_pred_corrected_1):
            if t_corr_1 < valid_start_1 or t_corr_1 > valid_end_1: continue
            
            pred_pts_EKF_f = pred_points_all_EKF_f[i]
            
            # 真值计算
            true_x, true_y, true_z = interp_x(t_corr_1), interp_y(t_corr_1), interp_z(t_corr_1)
            true_center = np.array([true_x, true_y, true_z])
            pitch, roll, yaw = interp_p(t_corr_1), interp_r(t_corr_1), interp_yw(t_corr_1)
            R = euler_to_rotmat(pitch, roll, yaw)
            true_points = true_center + (R @ local_coords.T).T
            #true_points[:, 2] *= -1
            
            plot_pred_EKF_f.append(pred_pts_EKF_f[0])
            
            # 像素误差计算
            for cam in camera_params_list:
                R_wc, t_wc, K = cam['R_wc'], cam['t_wc'], cam['K']
                for k in range(5):
                    p_c_true = R_wc @ (true_points[k] - t_wc)
                    if p_c_true[2] < 0.1: continue
                    u_true = width - (K[0,0] * (p_c_true[0] / p_c_true[2]) + K[0,2])
                    v_true = height - (K[1,1] * (p_c_true[1] / p_c_true[2]) + K[1,2])
                    #u_true = (K[0,0] * (p_c_true[0] / p_c_true[2]) + K[0,2])
                    #v_true = (K[1,1] * (p_c_true[1] / p_c_true[2]) + K[1,2])
                    
                    p_c_f = R_wc @ (pred_pts_EKF_f[k] - t_wc)
                    if p_c_f[2] < 0.1: continue
                    u_f = width - (K[0,0] * (p_c_f[0] / p_c_f[2]) + K[0,2])
                    v_f = height - (K[1,1] * (p_c_f[1] / p_c_f[2]) + K[1,2])
                    errors_EKF_f_pixl.append([u_true - u_f, v_true - v_f, k])

        # ==================== 6. 结果统计与输出 ====================
        print(f"\n有效样本数：LS={len(errors_pixl)//5}, LS+EKF={len(errors_EKF_pixl)//5}, 紧耦合={len(errors_EKF_f_pixl)//5}")
        
        def process_and_print(name, data_list):
            if not data_list: return
            data = np.array(data_list)
            print(f"\n{'='*15} {name} 像素误差报告 {'='*15}")
            # 修改表头：去掉 U/V 偏差，换成 MAE 和 距离
            print(f"{'点类型':<8} {'U平均偏移':<8} {'V平均偏移':<8} {'U标准差':<8} {'V标准差':<8} {'均方根误差':<8} {'平均欧式距离':<8}")
            print("-" * 70)
            
            labels = ["中心点", "旋翼 1", "旋翼 2", "旋翼 3", "旋翼 4"]
            
            for k in range(5):
                subset = data[data[:, 2] == k]
                if len(subset) == 0: continue
                
                errs = subset[:, 0:2]
                
                # 1. 计算 MAE (平均绝对误差) - 揭示被抵消的偏差
                mae_u = np.mean(np.abs(errs[:, 0]))
                mae_v = np.mean(np.abs(errs[:, 1]))
                
                # 2. 计算 Std (标准差) - 反映抖动
                std_u = np.std(errs[:, 0])
                std_v = np.std(errs[:, 1])
                
                # 3. 计算 RMSE (均方根误差) - 综合指标
                rmse = np.sqrt(np.mean(np.sum(errs**2, axis=1)))
                
                # 4. 计算 Mean Distance (平均欧氏距离) - 最直观的精度
                # 这里是先算每一帧的距离，再求平均，和 RMSE 略有不同，但更直观
                distances = np.sqrt(np.sum(errs**2, axis=1))
                mean_dist = np.mean(distances)
                
                print(f"{labels[k]:<8} {mae_u:>6.2f}   {mae_v:>6.2f}   {std_u:>6.2f}   {std_v:>6.2f}   {rmse:>6.2f}   {mean_dist:>6.2f}")
            
            # 计算总体指标
            all_errs = data[:, 0:2]
            total_mae = np.mean(np.abs(all_errs))
            total_rmse = np.sqrt(np.mean(np.sum(all_errs**2, axis=1)))
            total_dist = np.mean(np.sqrt(np.sum(all_errs**2, axis=1)))
            print("-" * 70)
            print(f"总体 -> MAE: {total_mae:.2f} px | RMSE: {total_rmse:.2f} px | Mean Dist: {total_dist:.2f} px")

        process_and_print("LS", errors_pixl)
        process_and_print("LS+EKF", errors_EKF_pixl)
        process_and_print("紧耦合 EKF", errors_EKF_f_pixl)
        print("="*50 + "\n")
        
        # ==================== 7. 绘图 ====================
        try:
            plot_pred = np.array(plot_pred)
            plot_true = np.array(plot_true)
            plot_pred_EKF = np.array(plot_pred_EKF)
            plot_pred_EKF_f = np.array(plot_pred_EKF_f) if plot_pred_EKF_f else np.array([])
            
            fig = plt.figure(figsize=(12, 9))
            ax = fig.add_subplot(111, projection='3d')
            
            ax.plot(plot_true[:, 0], plot_true[:, 1], plot_true[:, 2], 
                    color='#FF6B6B', linestyle='-', linewidth=2, label='Real')
            ax.plot(plot_pred[:, 0], plot_pred[:, 1], plot_pred[:, 2], 
                    color='#4ECDC4', linestyle='--', linewidth=2.5, label='LS')
            ax.plot(plot_pred_EKF[:, 0], plot_pred_EKF[:, 1], plot_pred_EKF[:, 2], 
                    color='#FFE66D', linestyle='-.', linewidth=2, label='LS+EKF')
            if len(plot_pred_EKF_f) > 0:
                ax.plot(plot_pred_EKF_f[:, 0], plot_pred_EKF_f[:, 1], plot_pred_EKF_f[:, 2], 
                    color="#660FE8", linestyle='-.', linewidth=2, label='EKF')
            
            for i in range(min(len(plot_pred), 50)):
                ax.plot([plot_pred[i, 0], plot_true[i, 0]],
                    [plot_pred[i, 1], plot_true[i, 1]],
                    [plot_pred[i, 2], plot_true[i, 2]], 
                    c='gray', alpha=0.5, linewidth=0.8)
                ax.plot([plot_pred_EKF[i, 0], plot_true[i, 0]],
                    [plot_pred_EKF[i, 1], plot_true[i, 1]],
                    [plot_pred_EKF[i, 2], plot_true[i, 2]], 
                    c='gray', alpha=0.5, linewidth=0.8)
                if len(plot_pred_EKF_f) > 0:
                    ax.plot([plot_pred_EKF_f[i, 0], plot_true[i, 0]],
                        [plot_pred_EKF_f[i, 1], plot_true[i, 1]],
                        [plot_pred_EKF_f[i, 2], plot_true[i, 2]], 
                        c='gray', alpha=0.5, linewidth=0.8)
            
            ax.set_xlabel('X (cm)')
            ax.set_ylabel('Y (cm)')
            ax.set_zlabel('Z (cm)')
            ax.set_title('Result with Fixed Delay Compensation')
            ax.legend()
            plt.tight_layout()
            plt.show()
        except Exception as e:
            print(f"画图失败：{e}")

    '''
    def calculate_R_diag(self, manual_delay, manual_delay_1):
        """
        手动设置延迟补偿量，并计算厘米误差和像素误差
        """
        print(f"=== 开始计算：手动补偿 LS+EKF 延迟 {manual_delay}s，紧耦合 EKF 延迟 {manual_delay_1}s ===")
        
        # ==================== 1. 数据提取 ====================
        camera_params_list = []
        for i in range(8): # 你有8个相机
            param = {
                        'R_wc': self.R_wc_matrices[i],    # 直接取预计算的旋转矩阵
                        't_wc': self.camera_positions[i], # 直接取预计算的位置
                        'K': global_import.K              # 内参矩阵
                        }
            camera_params_list.append(param)

        t_real = np.array([d['timestamp'] for d in plot_real_data])
        real_positions = np.array([
            [d['drone_pose']['position']['x'] * 100,
            d['drone_pose']['position']['y'] * 100,
            d['drone_pose']['position']['z'] * 100]
            for d in plot_real_data
        ])
        real_attitudes = np.array([
            [d['drone_pose']['orientation']['pitch'],
            d['drone_pose']['orientation']['roll'],
            d['drone_pose']['orientation']['yaw']]
            for d in plot_real_data
        ])

        t_pred = np.array([d['timestamp'] for d in plot_yolo_data])
        pred_points_all = []
        pred_points_all_EKF = []
        pred_points_all_EKF_f = []
        for d in plot_yolo_data:
            center = np.array(d['drone_center'])
            rotors = np.array(d['rotor_world_positions'])
            pred_points_all.append(np.vstack([center, rotors]))
        for d in plot_EKF_data:    
            center_EKF = np.array(d['drone_center'])
            rotors_EKF = np.array(d['rotor_world_positions'])
            pred_points_all_EKF.append(np.vstack([center_EKF, rotors_EKF]))
        for d in plot_finial_data:    
            center_EKF_f = np.array(d['drone_center'])
            rotors_EKF_f = np.array(d['rotor_world_positions'])
            pred_points_all_EKF_f.append(np.vstack([center_EKF_f, rotors_EKF_f]))
        pred_points_all = np.array(pred_points_all)
        pred_points_all_EKF = np.array(pred_points_all_EKF)
        pred_points_all_EKF_f = np.array(pred_points_all_EKF_f)

        # 时间归零
        t_real = t_real - t_real[0]
        t_pred = t_pred - t_pred[0]

        # ==================== 2. 应用补偿 ====================
        t_pred_corrected = t_pred - manual_delay      
        t_pred_corrected_1 = t_pred - manual_delay_1  
        
        # ==================== 3. 插值准备 ====================
        interp_x = interp1d(t_real, real_positions[:, 0], kind='linear', bounds_error=False, fill_value="extrapolate")
        interp_y = interp1d(t_real, real_positions[:, 1], kind='linear', bounds_error=False, fill_value="extrapolate")
        interp_z = interp1d(t_real, real_positions[:, 2], kind='linear', bounds_error=False, fill_value="extrapolate")
        interp_p = interp1d(t_real, real_attitudes[:, 0], kind='linear', bounds_error=False, fill_value="extrapolate")
        interp_r = interp1d(t_real, real_attitudes[:, 1], kind='linear', bounds_error=False, fill_value="extrapolate")
        interp_yw = interp1d(t_real, real_attitudes[:, 2], kind='linear', bounds_error=False, fill_value="extrapolate")

        local_coords = np.vstack([[0, 0, 0], rotor_position])
        
        # 厘米误差列表
        errors = []
        errors_EKF = []
        errors_EKF_f = []
        
        # 像素误差列表 (修改为存储: [u_err, v_err, 点索引k])
        errors_pixl = []
        errors_EKF_pixl = []
        errors_EKF_f_pixl = []
        
        plot_pred = []
        plot_pred_EKF = []
        plot_pred_EKF_f = []
        plot_true = []

        # ==================== 计算有效范围 ====================
        valid_start = max(t_real[0], t_pred_corrected[0])
        valid_end = min(t_real[-1], t_pred_corrected[-1])
        
        valid_start_1 = max(t_real[0], t_pred_corrected_1[0])
        valid_end_1 = min(t_real[-1], t_pred_corrected_1[-1])

        # ==================== 4. 误差计算 ====================
        for i, t_corr in enumerate(t_pred_corrected):
            if t_corr < valid_start or t_corr > valid_end:
                continue
            
            pred_pts = pred_points_all[i]
            pred_pts_EKF = pred_points_all_EKF[i]
            
            # 查询修正时刻的真值
            true_x = interp_x(t_corr)
            true_y = interp_y(t_corr)
            true_z = interp_z(t_corr)
            true_center = np.array([true_x, true_y, true_z])
            
            pitch = interp_p(t_corr)
            roll = interp_r(t_corr)
            yaw = interp_yw(t_corr)
            
            R = euler_to_rotmat(pitch, roll, yaw)
            true_points = true_center + (R @ local_coords.T).T
            true_points[:, 2] *= -1  # 翻转 z 轴
            
            # 计算 LS 和 LS+EKF 厘米误差
            error = pred_pts - true_points
            error_EKF = pred_pts_EKF - true_points
            errors.append(error.flatten())
            errors_EKF.append(error_EKF.flatten())
            
            plot_pred.append(pred_pts[0])
            plot_pred_EKF.append(pred_pts_EKF[0])
            plot_true.append(true_points[0])
            
            # --- 像素误差计算 (LS & LS+EKF) ---
            for cam in camera_params_list:
                R_wc = cam['R_wc']
                t_wc = cam['t_wc']
                K = cam['K']
                
                for k in range(5): # 0是中心点, 1-4是旋翼
                    # 1. 真值投影
                    p_w_true = true_points[k]
                    p_c_true = R_wc @ (p_w_true - t_wc)
                    if p_c_true[2] < 0.1: continue 
                    
                    u_proj_t = K[0,0] * (p_c_true[0] / p_c_true[2]) + K[0,2]
                    v_proj_t = K[1,1] * (p_c_true[1] / p_c_true[2]) + K[1,2]
                    u_true = width - u_proj_t 
                    v_true = height - v_proj_t
                    
                    # 2. LS 预测值投影
                    p_w_pred = pred_pts[k]
                    p_c_pred = R_wc @ (p_w_pred - t_wc)
                    if p_c_pred[2] < 0.1: continue
                    
                    u_proj_p = K[0,0] * (p_c_pred[0] / p_c_pred[2]) + K[0,2]
                    v_proj_p = K[1,1] * (p_c_pred[1] / p_c_pred[2]) + K[1,2]
                    u_pred = width - u_proj_p
                    v_pred = height - v_proj_p
                    
                    # 存入 LS 像素误差 (追加索引 k)
                    errors_pixl.append([u_true - u_pred, v_true - v_pred, k])
                    
                    # 3. LS+EKF 预测值投影
                    p_w_pred_ekf = pred_pts_EKF[k]
                    p_c_pred_ekf = R_wc @ (p_w_pred_ekf - t_wc)
                    if p_c_pred_ekf[2] < 0.1: continue
                    
                    u_proj_pe = K[0,0] * (p_c_pred_ekf[0] / p_c_pred_ekf[2]) + K[0,2]
                    v_proj_pe = K[1,1] * (p_c_pred_ekf[1] / p_c_pred_ekf[2]) + K[1,2]
                    u_pred_ekf = width - u_proj_pe
                    v_pred_ekf = height - v_proj_pe
                    
                    # 存入 LS+EKF 像素误差 (追加索引 k)
                    errors_EKF_pixl.append([u_true - u_pred_ekf, v_true - v_pred_ekf, k])

        # ==================== 5. 紧耦合 EKF 误差计算 ====================
        for i, t_corr_1 in enumerate(t_pred_corrected_1):
            if t_corr_1 < valid_start_1 or t_corr_1 > valid_end_1:
                continue
            
            pred_pts_EKF_f = pred_points_all_EKF_f[i]
            
            true_x = interp_x(t_corr_1)
            true_y = interp_y(t_corr_1)
            true_z = interp_z(t_corr_1)
            true_center = np.array([true_x, true_y, true_z])
            
            pitch = interp_p(t_corr_1)
            roll = interp_r(t_corr_1)
            yaw = interp_yw(t_corr_1)
            
            R = euler_to_rotmat(pitch, roll, yaw)
            true_points = true_center + (R @ local_coords.T).T
            true_points[:, 2] *= -1  
            
            error_EKF_f = pred_pts_EKF_f - true_points
            errors_EKF_f.append(error_EKF_f.flatten())
            plot_pred_EKF_f.append(pred_pts_EKF_f[0])
            
            # --- 像素误差计算 ---
            for cam in camera_params_list:
                R_wc = cam['R_wc']
                t_wc = cam['t_wc']
                K = cam['K']
                
                for k in range(5):
                    p_w_true = true_points[k]
                    p_c_true = R_wc @ (p_w_true - t_wc)
                    if p_c_true[2] < 0.1: continue
                    
                    u_proj_t = K[0,0] * (p_c_true[0] / p_c_true[2]) + K[0,2]
                    v_proj_t = K[1,1] * (p_c_true[1] / p_c_true[2]) + K[1,2]
                    u_true = width - u_proj_t
                    v_true = height - v_proj_t
                    
                    p_w_f = pred_pts_EKF_f[k]
                    p_c_f = R_wc @ (p_w_f - t_wc)
                    if p_c_f[2] < 0.1: continue
                    
                    u_proj_f = K[0,0] * (p_c_f[0] / p_c_f[2]) + K[0,2]
                    v_proj_f = K[1,1] * (p_c_f[1] / p_c_f[2]) + K[1,2]
                    u_f = width - u_proj_f
                    v_f = height - v_proj_f
                    
                    # 存入 紧耦合EKF 像素误差 (追加索引 k)
                    errors_EKF_f_pixl.append([u_true - u_f, v_true - v_f, k])

        # ==================== 6. 检查结果 ====================
        if not errors:
            print("错误：LS 补偿后没有时间重叠数据！")
            return None

        errors = np.array(errors)
        errors_EKF = np.array(errors_EKF)
        errors_EKF_f = np.array(errors_EKF_f) if errors_EKF_f else np.array([])
        
        R_diag = np.var(errors, axis=0)
        R_diag_EKF = np.var(errors_EKF, axis=0)
        R_diag_EKF_f = np.var(errors_EKF_f, axis=0) if len(errors_EKF_f) > 0 else np.array([])
        
        # 转换像素误差为数组
        errors_pixl = np.array(errors_pixl)
        errors_EKF_pixl = np.array(errors_EKF_pixl)
        errors_EKF_f_pixl = np.array(errors_EKF_f_pixl)
        # --- 像素误差输出 (使用切片逻辑) ---
        # 定义辅助函数计算方差
        def get_pix_stats(data, k_idx):
            # 筛选出索引为 k_idx 的数据
            subset = data[data[:, 2] == k_idx]
            if len(subset) == 0: return [0, 0]
            # 只取前两列 计算
            return np.var(subset[:, 0:2], axis=0)
        
        # ==================== 7. 结果输出 ====================
        print("========================\n")
        print(f"\n有效样本数：LS={len(errors)}, LS+EKF={len(errors_EKF)}, 紧耦合 EKF={len(errors_EKF_f)}")
        
        # --- 厘米误差输出 ---
        print("\n===== R_diag 计算结果 =====")
        print(f"中心点方差: {R_diag[0:3]}")
        print(f"旋翼 1 方差: {R_diag[3:6]}")
        print(f"旋翼 2 方差: {R_diag[6:9]}")
        print(f"旋翼 3 方差: {R_diag[9:12]}")
        print(f"旋翼 4 方差: {R_diag[12:15]}")
        print(f"平均方差：{np.mean(R_diag):.6f}")
        print(f"平均标准差：{np.mean(np.sqrt(R_diag)):.4f} cm")
        print("========================\n")
        print(f"\n=== 误差 EKF 输出 ===")
        print("\n===== R_diag 计算结果 =====")
        print(f"中心点方差: {R_diag_EKF[0:3]}")
        print(f"旋翼 1 方差: {R_diag_EKF[3:6]}")
        print(f"旋翼 2 方差: {R_diag_EKF[6:9]}")
        print(f"旋翼 3 方差: {R_diag_EKF[9:12]}")
        print(f"旋翼 4 方差: {R_diag_EKF[12:15]}")
        print(f"平均方差：{np.mean(R_diag_EKF):.6f}")
        print(f"平均标准差：{np.mean(np.sqrt(R_diag_EKF)):.4f} cm")
        print("========================\n")
        print(f"\n=== 误差 紧耦合EKF 输出 ===")
        print("\n===== R_diag 计算结果 =====")
        print(f"中心点方差: {R_diag_EKF_f[0:3]}")
        print(f"旋翼 1 方差: {R_diag_EKF_f[3:6]}")
        print(f"旋翼 2 方差: {R_diag_EKF_f[6:9]}")
        print(f"旋翼 3 方差: {R_diag_EKF_f[9:12]}")
        print(f"旋翼 4 方差: {R_diag_EKF_f[12:15]}")
        print(f"平均方差：{np.mean(R_diag_EKF_f):.6f}")
        print(f"平均标准差：{np.mean(np.sqrt(R_diag_EKF_f)):.4f} cm")
        print("========================\n")
        
        print(f"\n=== 像素误差统计 ===")
        
        # LS
        if len(errors_pixl) > 0:
            R_pix = np.array([get_pix_stats(errors_pixl, k) for k in range(5)])
            print("\n===== LS 像素方差 =====")
            print(f"中心点方差: {R_pix[0]}")
            print(f"旋翼 1 方差: {R_pix[1]}")
            print(f"旋翼 2 方差: {R_pix[2]}")
            print(f"旋翼 3 方差: {R_pix[3]}")
            print(f"旋翼 4 方差: {R_pix[4]}")
            print(f"平均方差：{np.mean(R_pix):.4f}")
            print(f"平均标准差：{np.mean(np.sqrt(R_pix)):.4f} px")

        # LS+EKF
        if len(errors_EKF_pixl) > 0:
            R_pix_ekf = np.array([get_pix_stats(errors_EKF_pixl, k) for k in range(5)])
            print("\n===== LS+EKF 像素方差 =====")
            print(f"中心点方差: {R_pix_ekf[0]}")
            print(f"旋翼 1 方差: {R_pix_ekf[1]}")
            print(f"旋翼 2 方差: {R_pix_ekf[2]}")
            print(f"旋翼 3 方差: {R_pix_ekf[3]}")
            print(f"旋翼 4 方差: {R_pix_ekf[4]}")
            print(f"平均方差：{np.mean(R_pix_ekf):.4f}")
            print(f"平均标准差：{np.mean(np.sqrt(R_pix_ekf)):.4f} px")

        # 紧耦合 EKF
        if len(errors_EKF_f_pixl) > 0:
            R_pix_f = np.array([get_pix_stats(errors_EKF_f_pixl, k) for k in range(5)])
            print("\n===== 紧耦合 EKF 像素方差 =====")
            print(f"中心点方差: {R_pix_f[0]}")
            print(f"旋翼 1 方差: {R_pix_f[1]}")
            print(f"旋翼 2 方差: {R_pix_f[2]}")
            print(f"旋翼 3 方差: {R_pix_f[3]}")
            print(f"旋翼 4 方差: {R_pix_f[4]}")
            print(f"平均方差：{np.mean(R_pix_f):.4f}")
            print(f"平均标准差：{np.mean(np.sqrt(R_pix_f)):.4f} px")
        
        print("========================\n") 
        
        # ==================== 8. 绘图 ====================
        try:
            plot_pred = np.array(plot_pred)
            plot_true = np.array(plot_true)
            plot_pred_EKF = np.array(plot_pred_EKF)
            plot_pred_EKF_f = np.array(plot_pred_EKF_f) if plot_pred_EKF_f else np.array([])
            
            fig = plt.figure(figsize=(12, 9))
            ax = fig.add_subplot(111, projection='3d')
            
            ax.plot(plot_true[:, 0], plot_true[:, 1], plot_true[:, 2], 
                    color='#FF6B6B', linestyle='-', linewidth=2, label='Real')
            ax.plot(plot_pred[:, 0], plot_pred[:, 1], plot_pred[:, 2], 
                    color='#4ECDC4', linestyle='--', linewidth=2.5, label='LS')
            ax.plot(plot_pred_EKF[:, 0], plot_pred_EKF[:, 1], plot_pred_EKF[:, 2], 
                    color='#FFE66D', linestyle='-.', linewidth=2, label='LS+EKF')
            if len(plot_pred_EKF_f) > 0:
                ax.plot(plot_pred_EKF_f[:, 0], plot_pred_EKF_f[:, 1], plot_pred_EKF_f[:, 2], 
                        color="#660FE8", linestyle='-.', linewidth=2, label='EKF')
            

            for i in range(min(len(plot_pred), 50)):
                ax.plot([plot_pred[i, 0], plot_true[i, 0]],
                    [plot_pred[i, 1], plot_true[i, 1]],
                    [plot_pred[i, 2], plot_true[i, 2]], 
                    c='gray', alpha=0.5, linewidth=0.8)
                ax.plot([plot_pred_EKF[i, 0], plot_true[i, 0]],
                    [plot_pred_EKF[i, 1], plot_true[i, 1]],
                    [plot_pred_EKF[i, 2], plot_true[i, 2]], 
                    c='gray', alpha=0.5, linewidth=0.8)
                ax.plot([plot_pred_EKF_f[i, 0], plot_true[i, 0]],
                    [plot_pred_EKF_f[i, 1], plot_true[i, 1]],
                    [plot_pred_EKF_f[i, 2], plot_true[i, 2]], 
                    c='gray', alpha=0.5, linewidth=0.8)
            
            ax.set_xlabel('X (cm)')
            ax.set_ylabel('Y (cm)')
            ax.set_zlabel('Z (cm)')
            ax.set_title('Result with Fixed Delay Compensation')
            ax.legend()
            plt.tight_layout()
            plt.show()
            

        except Exception as e:
            print(f"画图失败：{e}")
        return R_diag
    '''

    def output_data(self):
        """
        以 CSV 形式保存所有绘图数据
        共 92 列：
        - 公共时间戳 (1) + 公共真值 (18) + LS(18) + LS+EKF(18)
        - 紧耦合 EKF 时间戳 (1) + 紧耦合 EKF 对应真值 (18) + 紧耦合 EKF 数据 (18)
        
        每部分 18 列 = 3 位置 + 12 旋翼点 (4 个×3) + 3 姿态 (pitch,roll,yaw)
        """
        # ==================== 1. 数据准备 ====================
        # 真值时间戳归一化
        t_real = np.array([d['timestamp'] for d in plot_real_data])
        #t_real = t_real - t_real[0]
        
        # 预测时间戳归一化 (LS/LS+EKF/紧耦合 EKF 原始时间戳相同)
        t_pred = np.array([d['timestamp'] for d in plot_yolo_data])
        #t_pred = t_pred - t_pred[0]
        
        # 应用延迟补偿
        t_pred_corrected = t_pred - global_import.manual_delay       # LS 和 LS+EKF 的补偿后时间戳
        t_pred_corrected_final = t_pred - global_import.manual_delay_1  # 紧耦合 EKF 的补偿后时间戳

        real_positions = np.array([
            [d['drone_pose']['position']['x'] * 100,
            d['drone_pose']['position']['y'] * 100,
            d['drone_pose']['position']['z'] * 100]
            for d in plot_real_data
        ])
        real_attitudes = np.array([
            [d['drone_pose']['orientation']['pitch'],
            d['drone_pose']['orientation']['roll'],
            d['drone_pose']['orientation']['yaw']]
            for d in plot_real_data
        ])
        real_attitudes = np.unwrap(real_attitudes, axis=0)  # 解开姿态的周期性
        
        # ==================== 2. 真值插值函数 ====================
        interp_x = interp1d(t_real, real_positions[:, 0], kind='linear', bounds_error=False, fill_value="extrapolate")
        interp_y = interp1d(t_real, real_positions[:, 1], kind='linear', bounds_error=False, fill_value="extrapolate")
        interp_z = interp1d(t_real, real_positions[:, 2], kind='linear', bounds_error=False, fill_value="extrapolate")
        interp_pitch = interp1d(t_real, real_attitudes[:, 0], kind='linear', bounds_error=False, fill_value="extrapolate")
        interp_roll = interp1d(t_real, real_attitudes[:, 1], kind='linear', bounds_error=False, fill_value="extrapolate")
        interp_yaw = interp1d(t_real, real_attitudes[:, 2], kind='linear', bounds_error=False, fill_value="extrapolate")

        # 计算旋翼局部坐标
        local_coords = np.vstack([[0, 0, 0], rotor_position])
        
        # ==================== 3. 辅助函数 ====================
        def get_true_data(t_corr):
            """根据补偿后的时间戳获取对应的真值数据"""
            true_x = interp_x(t_corr)
            true_y = interp_y(t_corr)
            true_z = interp_z(t_corr)
            true_pitch = interp_pitch(t_corr)
            true_roll = interp_roll(t_corr)
            true_yaw = interp_yaw(t_corr)
            
            # 计算真值旋翼位置
            true_center = np.array([true_x, true_y, true_z])
            R_true = euler_to_rotmat(true_pitch, true_roll, true_yaw)
            true_points = true_center + (R_true @ local_coords.T).T
            #true_points[:, 2] *= -1  # Z 轴翻转
            #true_center[2] *= -1  # Z 轴翻转
            
            return true_center, true_points, true_pitch, true_roll, true_yaw
        
        def get_method_data(data_list, idx):
            """获取指定方法的数据"""
            if idx < len(data_list):
                d = data_list[idx]
                center = np.array(d['drone_center'])
                rotors = np.array(d['rotor_world_positions'])
                return center, rotors, d['pitch'], d['roll'], d['yaw']
            return None, None, None, None, None
        
        # ==================== 4. 构建输出数据 ====================
        output_data = []
        
        # 有效时间范围 (LS/LS+EKF)
        valid_start = max(t_real[0], t_pred_corrected[0])
        valid_end = min(t_real[-1], t_pred_corrected[-1])
        
        # 有效时间范围 (紧耦合 EKF)
        valid_start_final = max(t_real[0], t_pred_corrected_final[0])
        valid_end_final = min(t_real[-1], t_pred_corrected_final[-1])
        
        for i, t_corr in enumerate(t_pred_corrected):
            # 检查 LS/LS+EKF 是否在有效范围内
            if t_corr < valid_start or t_corr > valid_end:
                continue
            
            # --- 获取公共真值 (LS/LS+EKF 对应) ---
            true_center, true_points, true_pitch, true_roll, true_yaw = get_true_data(t_corr)
            
            # --- 获取 LS 数据 ---
            ls_center, ls_rotors, ls_pitch, ls_roll, ls_yaw = get_method_data(plot_yolo_data, i)
            if ls_center is None:
                continue
            
            # --- 获取 LS+EKF 数据 ---
            ekf_center, ekf_rotors, ekf_pitch, ekf_roll, ekf_yaw = get_method_data(plot_EKF_data, i)
            if ekf_center is None:
                ekf_center, ekf_rotors = ls_center, ls_rotors
                ekf_pitch, ekf_roll, ekf_yaw = ls_pitch, ls_roll, ls_yaw
            
            # --- 获取紧耦合 EKF 时间戳和数据 ---
            t_corr_final = t_pred_corrected_final[i]
            
            # 获取紧耦合 EKF 对应的真值
            if valid_start_final <= t_corr_final <= valid_end_final:
                final_true_center, final_true_points, final_true_pitch, final_true_roll, final_true_yaw = get_true_data(t_corr_final)
            else:
                # 如果超出范围，使用公共真值
                final_true_center, final_true_points = true_center, true_points
                final_true_pitch, final_true_roll, final_true_yaw = true_pitch, true_roll, true_yaw
            
            # 获取紧耦合 EKF 数据
            final_center, final_rotors, final_pitch, final_roll, final_yaw = get_method_data(plot_finial_data, i)
            if final_center is None:
                final_center, final_rotors = ls_center, ls_rotors
                final_pitch, final_roll, final_yaw = ls_pitch, ls_roll, ls_yaw
            
            # --- 构建单行数据 (92 列) ---
            row = []
            
            # 第 1 列：公共时间戳
            row.append(t_corr - valid_start)
            
            # 第 2-19 列：公共真值 (3 位置 + 12 旋翼点 + 3 姿态)
            row.extend(true_center.tolist())
            row.extend(true_points[1:].flatten())  # 4 个旋翼点，12 列
            row.extend([true_pitch, true_roll, true_yaw])
            
            # 第 20-37 列：LS 数据
            row.extend(ls_center.tolist())
            row.extend(ls_rotors.flatten())
            row.extend([ls_pitch, ls_roll, ls_yaw])
            
            # 第 38-55 列：LS+EKF 数据
            row.extend(ekf_center.tolist())
            row.extend(ekf_rotors.flatten())
            row.extend([ekf_pitch, ekf_roll, ekf_yaw])
            
            # 第 56 列：紧耦合 EKF 时间戳
            row.append(t_corr_final - valid_start_final)
            
            # 第 57-74 列：紧耦合 EKF 对应真值
            row.extend(final_true_center.tolist())
            row.extend(final_true_points[1:].flatten())
            row.extend([final_true_pitch, final_true_roll, final_true_yaw])
            
            # 第 75-92 列：紧耦合 EKF 数据
            row.extend(final_center.tolist())
            row.extend(final_rotors.flatten())
            row.extend([final_pitch, final_roll, final_yaw])
            
            output_data.append(row)
        
        # ==================== 5. 保存为 CSV ====================
        if len(output_data) == 0:
            print("⚠️ 没有有效数据可保存")
            return
        
        # 创建保存目录
        save_dir = os.path.join(self.current_dir, "output_data")
        os.makedirs(save_dir, exist_ok=True)
        
        # 生成文件名
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        save_path = os.path.join(save_dir, f"trajectory_data_{timestamp}.csv")
        
        # ==================== 6. 构建表头 ====================
        header = []
        
        # 公共时间戳
        header.append("时间戳")
        
        # 公共真值表头
        header.extend(["x真值", "y真值", "z真值"])
        for j in range(4):
            header.extend([f"旋翼{j+1}x真值", f"旋翼{j+1}y真值", f"旋翼{j+1}z真值"])
        header.extend(["pitch真值", "roll真值", "yaw真值"])
        
        # LS 表头
        header.extend(["ls中心x", "ls中心y", "ls中心z"])
        for j in range(4):
            header.extend([f"ls旋翼{j+1}x", f"ls旋翼{j+1}y", f"ls旋翼{j+1}z"])
        header.extend(["ls算法pitch", "ls算法roll", "ls算法yaw"])
        
        # LS+EKF 表头
        header.extend(["ekf中心x", "ekf中心y", "ekf中心z"])
        for j in range(4):
            header.extend([f"ekf旋翼{j+1}x", f"ekf旋翼{j+1}y", f"ekf旋翼{j+1}z"])
        header.extend(["ekf算法pitch", "ekf算法roll", "ekf算法yaw"])
        
        # 紧耦合 EKF 时间戳
        header.append("紧耦合时间戳")
        
        # 紧耦合 EKF 对应真值表头
        header.extend(["x真值", "y真值", "z真值"])
        for j in range(4):
            header.extend([f"旋翼{j+1}x真值", f"旋翼{j+1}y真值", f"旋翼{j+1}z真值"])
        header.extend(["pitch真值", "roll真值", "yaw真值"])
        
        # 紧耦合 EKF 数据表头
        header.extend(["紧耦合中心x", "紧耦合中心y", "紧耦合中心z"])
        for j in range(4):
            header.extend([f"紧耦合旋翼{j+1}x", f"紧耦合旋翼{j+1}y", f"紧耦合旋翼{j+1}z"])
        header.extend(["紧耦合算法pitch", "紧耦合算法roll", "紧耦合算法yaw"])
        
        # ==================== 7. 写入 CSV ====================
        with open(save_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(output_data)
        
        print(f"✅ 数据已保存至：{save_path}")
        print(f"   共 {len(output_data)} 行，{len(header)} 列")
        print(f"\n📊 数据结构说明:")
        print(f"   - 列 1: 公共时间戳 (LS/LS+EKF)")
        print(f"   - 列 2-19: 公共真值 (对应 LS/LS+EKF)")
        print(f"   - 列 20-37: LS 数据")
        print(f"   - 列 38-55: LS+EKF 数据")
        print(f"   - 列 56: 紧耦合 EKF 时间戳 (延迟补偿：{manual_delay_1}s)")
        print(f"   - 列 57-74: 紧耦合 EKF 对应真值")
        print(f"   - 列 75-92: 紧耦合 EKF 数据")
    # 以svg的形式保存所有绘图数据

#===============================神秘四小贩出餐口================================#
import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def euler_to_rotmat(pitch, roll, yaw):
    cp, cr, cy = np.cos(pitch), np.cos(roll), np.cos(yaw)
    sp, sr, sy = np.sin(pitch), np.sin(roll), np.sin(yaw)
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr]
    ])
    return R
# 欧拉角转旋转矩阵

'''
def calculate_R_diag(manual_delay, manual_delay_1):
    """
    手动设置延迟补偿量
    manual_delay: LS 和 LS+EKF 预测值滞后的时间 (秒)
    manual_delay_1: 紧耦合 EKF 预测值滞后的时间 (秒)
    """
    print(f"=== 开始计算：手动补偿 LS+EKF 延迟 {manual_delay}s，紧耦合 EKF 延迟 {manual_delay_1}s ===")
    
    # ==================== 1. 数据提取 ====================
    t_real = np.array([d['timestamp'] for d in plot_real_data])
    real_positions = np.array([
        [d['drone_pose']['position']['x'] * 100,
         d['drone_pose']['position']['y'] * 100,
         d['drone_pose']['position']['z'] * 100]
        for d in plot_real_data
    ])
    real_attitudes = np.array([
        [d['drone_pose']['orientation']['pitch'],
         d['drone_pose']['orientation']['roll'],
         d['drone_pose']['orientation']['yaw']]
        for d in plot_real_data
    ])

    t_pred = np.array([d['timestamp'] for d in plot_yolo_data])
    pred_points_all = []
    pred_points_all_EKF = []
    pred_points_all_EKF_f = []
    for d in plot_yolo_data:
        center = np.array(d['drone_center'])
        rotors = np.array(d['rotor_world_positions'])
        pred_points_all.append(np.vstack([center, rotors]))
    for d in plot_EKF_data:    
        center_EKF = np.array(d['drone_center'])
        rotors_EKF = np.array(d['rotor_world_positions'])
        pred_points_all_EKF.append(np.vstack([center_EKF, rotors_EKF]))
    for d in plot_finial_data:    
        center_EKF_f = np.array(d['drone_center'])
        rotors_EKF_f = np.array(d['rotor_world_positions'])
        pred_points_all_EKF_f.append(np.vstack([center_EKF_f, rotors_EKF_f]))
    pred_points_all = np.array(pred_points_all)
    pred_points_all_EKF = np.array(pred_points_all_EKF)
    pred_points_all_EKF_f = np.array(pred_points_all_EKF_f)

    # 时间归零
    t_real = t_real - t_real[0]
    t_pred = t_pred - t_pred[0]

    # ==================== 2. 应用补偿 ====================
    t_pred_corrected = t_pred - manual_delay      # LS 和 LS+EKF 使用同一延迟
    t_pred_corrected_1 = t_pred - manual_delay_1  # 紧耦合 EKF 使用独立延迟
    
    # ==================== 3. 插值准备 ====================
    interp_x = interp1d(t_real, real_positions[:, 0], kind='linear', bounds_error=False, fill_value="extrapolate")
    interp_y = interp1d(t_real, real_positions[:, 1], kind='linear', bounds_error=False, fill_value="extrapolate")
    interp_z = interp1d(t_real, real_positions[:, 2], kind='linear', bounds_error=False, fill_value="extrapolate")
    interp_p = interp1d(t_real, real_attitudes[:, 0], kind='linear', bounds_error=False, fill_value="extrapolate")
    interp_r = interp1d(t_real, real_attitudes[:, 1], kind='linear', bounds_error=False, fill_value="extrapolate")
    interp_yw = interp1d(t_real, real_attitudes[:, 2], kind='linear', bounds_error=False, fill_value="extrapolate")

    local_coords = np.vstack([[0, 0, 0], rotor_position])
    errors = []
    errors_EKF = []
    errors_EKF_f = []
    
    plot_pred = []
    plot_pred_EKF = []
    plot_pred_EKF_f = []
    plot_true = []

    # ==================== 计算有效范围 ====================
    # LS 和 LS+EKF 的有效范围
    valid_start = max(t_real[0], t_pred_corrected[0])
    valid_end = min(t_real[-1], t_pred_corrected[-1])
    
    # 紧耦合 EKF 的有效范围
    valid_start_1 = max(t_real[0], t_pred_corrected_1[0])
    valid_end_1 = min(t_real[-1], t_pred_corrected_1[-1])

    # ==================== 4. 误差计算 ====================
    for i, t_corr in enumerate(t_pred_corrected):
        # 检查是否在有效范围内
        if t_corr < valid_start or t_corr > valid_end:
            continue
        
        pred_pts = pred_points_all[i]
        pred_pts_EKF = pred_points_all_EKF[i]
        
        # 查询修正时刻的真值
        true_x = interp_x(t_corr)
        true_y = interp_y(t_corr)
        true_z = interp_z(t_corr)
        true_center = np.array([true_x, true_y, true_z])
        
        pitch = interp_p(t_corr)
        roll = interp_r(t_corr)
        yaw = interp_yw(t_corr)
        
        R = euler_to_rotmat(pitch, roll, yaw)
        true_points = true_center + (R @ local_coords.T).T
        true_points[:, 2] *= -1  # 翻转 z 轴
        
        # 计算 LS 和 LS+EKF 误差
        error = pred_pts - true_points
        error_EKF = pred_pts_EKF - true_points
        errors.append(error.flatten())
        errors_EKF.append(error_EKF.flatten())
        
        plot_pred.append(pred_pts[0])
        plot_pred_EKF.append(pred_pts_EKF[0])
        plot_true.append(true_points[0])
    
    # ==================== 5. 紧耦合 EKF 误差计算（独立延迟） ====================
    for i, t_corr_1 in enumerate(t_pred_corrected_1):
        # 检查是否在紧耦合 EKF 的有效范围内
        if t_corr_1 < valid_start_1 or t_corr_1 > valid_end_1:
            continue
        
        pred_pts_EKF_f = pred_points_all_EKF_f[i]
        
        # 查询修正时刻的真值（使用紧耦合 EKF 的修正时间）
        true_x = interp_x(t_corr_1)
        true_y = interp_y(t_corr_1)
        true_z = interp_z(t_corr_1)
        true_center = np.array([true_x, true_y, true_z])
        
        pitch = interp_p(t_corr_1)
        roll = interp_r(t_corr_1)
        yaw = interp_yw(t_corr_1)
        
        R = euler_to_rotmat(pitch, roll, yaw)
        true_points = true_center + (R @ local_coords.T).T
        true_points[:, 2] *= -1  # 翻转 z 轴
        
        # 计算紧耦合 EKF 误差
        error_EKF_f = pred_pts_EKF_f - true_points
        errors_EKF_f.append(error_EKF_f.flatten())
        plot_pred_EKF_f.append(pred_pts_EKF_f[0])

    # ==================== 6. 检查结果 ====================
    if not errors:
        print("错误：LS 补偿后没有时间重叠数据！请减小 manual_delay。")
        return None
    if not errors_EKF_f:
        print("警告：紧耦合 EKF 补偿后没有时间重叠数据！请减小 manual_delay_1。")

    errors = np.array(errors)
    errors_EKF = np.array(errors_EKF)
    errors_EKF_f = np.array(errors_EKF_f) if errors_EKF_f else np.array([])
    
    R_diag = np.var(errors, axis=0)
    R_diag_EKF = np.var(errors_EKF, axis=0)
    R_diag_EKF_f = np.var(errors_EKF_f, axis=0) if len(errors_EKF_f) > 0 else np.array([])
    
    # ==================== 7. 结果输出 ====================
    print("========================\n")
    print(f"\n有效样本数：LS={len(errors)}, LS+EKF={len(errors_EKF)}, 紧耦合 EKF={len(errors_EKF_f)}")
    print("\n===== R_diag 计算结果 =====")
    print(f"中心点方差: {R_diag[0:3]}")
    print(f"旋翼 1 方差: {R_diag[3:6]}")
    print(f"旋翼 2 方差: {R_diag[6:9]}")
    print(f"旋翼 3 方差: {R_diag[9:12]}")
    print(f"旋翼 4 方差: {R_diag[12:15]}")
    print(f"平均方差：{np.mean(R_diag):.6f}")
    print(f"平均标准差：{np.mean(np.sqrt(R_diag)):.4f} cm")
    print("========================\n")
    print(f"\n=== 误差 EKF 输出 ===")
    print("\n===== R_diag 计算结果 =====")
    print(f"中心点方差: {R_diag_EKF[0:3]}")
    print(f"旋翼 1 方差: {R_diag_EKF[3:6]}")
    print(f"旋翼 2 方差: {R_diag_EKF[6:9]}")
    print(f"旋翼 3 方差: {R_diag_EKF[9:12]}")
    print(f"旋翼 4 方差: {R_diag_EKF[12:15]}")
    print(f"平均方差：{np.mean(R_diag_EKF):.6f}")
    print(f"平均标准差：{np.mean(np.sqrt(R_diag_EKF)):.4f} cm")
    print("========================\n")
    print(f"\n=== 误差 紧耦合EKF 输出 ===")
    print("\n===== R_diag 计算结果 =====")
    print(f"中心点方差: {R_diag_EKF_f[0:3]}")
    print(f"旋翼 1 方差: {R_diag_EKF_f[3:6]}")
    print(f"旋翼 2 方差: {R_diag_EKF_f[6:9]}")
    print(f"旋翼 3 方差: {R_diag_EKF_f[9:12]}")
    print(f"旋翼 4 方差: {R_diag_EKF_f[12:15]}")
    print(f"平均方差：{np.mean(R_diag_EKF_f):.6f}")
    print(f"平均标准差：{np.mean(np.sqrt(R_diag_EKF_f)):.4f} cm")
    print("========================\n")
    
    # ==================== 8. 绘图 ====================
    try:
        plot_pred = np.array(plot_pred)
        plot_true = np.array(plot_true)
        plot_pred_EKF = np.array(plot_pred_EKF)
        plot_pred_EKF_f = np.array(plot_pred_EKF_f) if plot_pred_EKF_f else np.array([])
        
        fig = plt.figure(figsize=(12, 9))
        ax = fig.add_subplot(111, projection='3d')
        
        ax.plot(plot_true[:, 0], plot_true[:, 1], plot_true[:, 2], 
                color='#FF6B6B', linestyle='-', linewidth=2, label='Real')
        ax.plot(plot_pred[:, 0], plot_pred[:, 1], plot_pred[:, 2], 
                color='#4ECDC4', linestyle='--', linewidth=2.5, label='LS')
        ax.plot(plot_pred_EKF[:, 0], plot_pred_EKF[:, 1], plot_pred_EKF[:, 2], 
                color='#FFE66D', linestyle='-.', linewidth=2, label='LS+EKF')
        if len(plot_pred_EKF_f) > 0:
            ax.plot(plot_pred_EKF_f[:, 0], plot_pred_EKF_f[:, 1], plot_pred_EKF_f[:, 2], 
                    color="#660FE8", linestyle='-.', linewidth=2, label='EKF')
        

        # 画连线（仅前 50 个点）
        for i in range(min(len(plot_pred), 50)):
            ax.plot([plot_pred[i, 0], plot_true[i, 0]],
                   [plot_pred[i, 1], plot_true[i, 1]],
                   [plot_pred[i, 2], plot_true[i, 2]], 
                   c='gray', alpha=0.5, linewidth=0.8)
            ax.plot([plot_pred_EKF[i, 0], plot_true[i, 0]],
                   [plot_pred_EKF[i, 1], plot_true[i, 1]],
                   [plot_pred_EKF[i, 2], plot_true[i, 2]], 
                   c='gray', alpha=0.5, linewidth=0.8)
            ax.plot([plot_pred_EKF_f[i, 0], plot_true[i, 0]],
                   [plot_pred_EKF_f[i, 1], plot_true[i, 1]],
                   [plot_pred_EKF_f[i, 2], plot_true[i, 2]], 
                   c='gray', alpha=0.5, linewidth=0.8)
        
        #ax.scatter(plot_pred[:, 0], plot_pred[:, 1], plot_pred[:, 2], 
        #          c='red', marker='o', s=30, label='Real')
        #ax.scatter(plot_true[:, 0], plot_true[:, 1], plot_true[:, 2], 
        #          c='blue', marker='^', s=30, label='LS')
        #ax.scatter(plot_pred_EKF[:, 0], plot_pred_EKF[:, 1], plot_pred_EKF[:, 2], 
        #          c='green', marker='x', s=30, label='LS+EKF')
        
        ax.set_xlabel('X (cm)')
        ax.set_ylabel('Y (cm)')
        ax.set_zlabel('Z (cm)')
        ax.set_title('Result with Fixed Delay Compensation')
        ax.legend()
        plt.tight_layout()
        plt.show()
        

    except Exception as e:
        print(f"画图失败：{e}")
    return R_diag
# 评价模块
'''
