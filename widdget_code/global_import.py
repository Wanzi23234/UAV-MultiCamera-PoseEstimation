# 用以存储所有的全局变量
# 全局变量控制程序运行
import numpy as np
# 定义飞行空间范围（立方体边界）
X_MIN, X_MAX = -10, 10
Y_MIN, Y_MAX = -10, 10
Z_MIN, Z_MAX = 0, -4
l = 1600
angle = 45
tip = 3
speed = 5
# 相机内参矩阵K
K = np.array([
    [424.706578993305, 0, 424.096438220802],
    [0, 430.745625515531, 241.316436398605],
    [0, 0, 1]
    ])
rotor_position = np.array([
    [60.250, -110.650, 8.301],
    [-95.25, -80.550, -7.499],
    [60.250, 110.650, 8.301],
    [-95.25, 80.550, -7.499],
])
rotor_position_EKF = np.array([
    [0, 0, 0],
    [60.250, -110.650, 8.301],
    [-95.25, -80.550, -7.499],
    [60.250, 110.650, 8.301],
    [-95.25, 80.550, -7.499],
])
pt_name = 'best.pt'

# EKF调参变量
R_diag_global = 10
q_alpha_global = 0.2
q_a_global = 500
P_init_global = None
manual_delay = 0.1
manual_delay_1 = 0.1
r_pixel = 0.5
zaosheng = 0
zaosheng_1 = 0

# UE4图像切割参数
picture_split = [[852,2], [1704,2], [0,482], [852,482], [1704,482], [0,962], [852,962], [1704,962]]
def calculate_camera_position_bias(angle_deg):
    angle_rad = np.radians(angle_deg)
    cos_angle = np.cos(angle_rad)
    sin_angle = np.sin(angle_rad)
    
    # 8个相机的位置偏移（归一化向量）
    camera_position_bias = [
        [-cos_angle, 0, sin_angle],                    # 正后方向
        [0, -cos_angle, sin_angle],                    # 左方向
        [cos_angle, 0, sin_angle],                     # 前方向
        [0, cos_angle, sin_angle],                     # 右方向
        [-cos_angle/np.sqrt(2), -cos_angle/np.sqrt(2), sin_angle], # 45°仰角前左
        [cos_angle/np.sqrt(2), -cos_angle/np.sqrt(2), sin_angle],  # 45°仰角前右
        [cos_angle/np.sqrt(2), cos_angle/np.sqrt(2), sin_angle],   # 45°仰角后右
        [-cos_angle/np.sqrt(2), cos_angle/np.sqrt(2), sin_angle]   # 45°仰角后左
    ]
    
    return camera_position_bias    
#根据给定角度计算相机位置偏移
#angle_deg: 与水平面的夹角（度）
    
width, height = 853, 480

import airsim
import time
import json
import os
from datetime import datetime
import threading
import random
import cv2

running = True
# 运行状态标识

plot_real_data = []
plot_yolo_data = []
plot_EKF_data = []
plot_finial_data = []
#😭展示时很重要的数据存储地

def keyboard_listener(stop_event=None):
    """监听键盘输入，按下1键停止程序"""
    while stop_event is None or not stop_event.is_set():
        try:
            # 简单的键盘监听 - 在Windows上可以使用msvcrt
            import msvcrt
            if msvcrt.kbhit():
                key = msvcrt.getch().decode('utf-8')
                if key == '1':
                    print("检测到按键'1'，准备停止程序...")
                    if stop_event:
                        stop_event.set()
                    break
        except ImportError:
            # 非Windows系统可以使用其他方式
            import sys, select
            try:
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)
                    if key == '1':
                        print("检测到按键'1'，准备停止程序...")
                        if stop_event:
                            stop_event.set()
                        break
            except:
                pass
        time.sleep(0.1)
# 监听线程

def generate_random_waypoint():
    """在指定立方体内生成随机航点"""
    x = random.uniform(X_MIN, X_MAX)
    y = random.uniform(Y_MIN, Y_MAX)
    z = random.uniform(Z_MIN, Z_MAX)
    return x, y, z
# 生产随机路线

def create_trigger_file(record_name):
    """创建触发文件通知UE4拍照"""
    # 创建统一的通信目录
    comm_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'WindowsNoEditor', "try2" , "connect")
    #comm_dir = "C:\\Users\\whz\\Desktop\\WindowsNoEditor\\try2\\connect"
    #comm_dir = "D:\\Unreal Projects\\try2\\connect"
    if not os.path.exists(comm_dir):
        os.makedirs(comm_dir)
    
    # 创建触发文件
    trigger_file = os.path.join(comm_dir, f"trigger_{record_name}.json")
    
    data = {
        "record": f"record_{record_name}.json",
        "timestamp": datetime.now().isoformat(),
        "command": "TAKE_PHOTO"
    }
    
    with open(trigger_file, 'w') as f:
        json.dump(data, f, indent=2)
    
    print(f"触发文件已创建: {trigger_file}")
# 触发拍照文件

def create_record_file(record_name, drone_info):
    """创建记录文件保存无人机信息"""
    # 创建统一的保存目录
    comm_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'recorder')
    #comm_dir = "D:\\anaconda\\envs\\Shining\\shin\\recorder"
    if not os.path.exists(comm_dir):
        os.makedirs(comm_dir)
    
    # 创建记录文件
    record_file = os.path.join(comm_dir, f"record_{record_name}.json")
    
    data = {
        "record": record_name,
        "timestamp": datetime.now().isoformat(),
        "drone_info": drone_info,
    }
    
    with open(record_file, 'w') as f:
        json.dump(data, f, indent=2)
    
    print(f"记录文件已创建: {record_file}")
# 触发记录文件

def create_camera_trigger_file():
    # 创建统一的通信目录
    comm_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'WindowsNoEditor', "try2" , "connect")
    #comm_dir = "D:\\Unreal Projects\\try2\\connect"
    if not os.path.exists(comm_dir):
        os.makedirs(comm_dir)
    
    # 创建触发文件
    trigger_file = os.path.join(comm_dir, "camerat.json")
    
    data = {
        "camera": "apply"
    }
    
    with open(trigger_file, 'w') as f:
        json.dump(data, f, indent=2)
    
    print(f"触发文件已创建: {trigger_file}")
# 相机信息变更触发

def split_picture_folder(input_folder, output_folder, filename):
    # 创建输出文件夹（如果不存在）
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    
    # 检查是否为图片文件
    if filename.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tiff')):
        # 正确的文件路径拼接
        input_path = os.path.join(input_folder, filename)
        
        # 读取图片
        image = cv2.imread(input_path)
        
        # 检查图片是否成功读取
        if image is not None:
            # 按照 picture_split 中的8个点切割图片
            for i, (x, y) in enumerate(picture_split):
                # 裁剪图片
                cropped_image = image[y:y+height, x:x+width]
                
                # 生成新文件名（原文件名+索引）
                name, ext = os.path.splitext(filename)
                new_filename = f"{name}_{i + 1}{ext}"
                
                # 输出文件路径
                output_path = os.path.join(output_folder, new_filename)
                
                # 保存裁剪后的图片
                cv2.imwrite(output_path, cropped_image)
                print(f"Processed: {new_filename}")
        else:
            print(f"Failed to read: {filename}")
# 切割单张图片

import numpy as np
import time
from numdifftools import Jacobian  # 需安装：pip install numdifftools

class RigidBodyEKF:
    """
    刚体扩展卡尔曼滤波器，用于无人机位置与姿态的平滑估计。
    状态向量 (13维): [px, py, pz, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz]
        - p: 世界系下中心位置
        - v: 世界系下速度
        - q: 单位四元数 (从机体到世界)
        - w: 机体角速度 (机体坐标系)
    观测向量 (15维): 五个点的世界坐标 [中心(xyz), 旋翼1(xyz), ..., 旋翼4(xyz)]
    """
    def __init__(self, local_coords=None, qa=1.0, qalpha=1.0, R_diag=0.02**2):
        """
        参数:
            local_coords: 5个点在机体坐标系下的坐标，形状 (5,3) 或 (3,5)，默认 None（稍后设置）
            qa: 加速度过程噪声谱密度 (连续)
            qalpha: 角加速度过程噪声谱密度 (连续)
            R_diag: 测量噪声方差（每个坐标轴相同），可传入标量或15维向量
        """
        self.dim_x = 13
        self.dim_z = 15

        # 状态向量与协方差
        self.x = None          # 初始化时设置
        self.P = None

        # 过程噪声参数
        self.qa = qa
        self.qalpha = qalpha

        # 测量噪声协方差矩阵
        if np.isscalar(R_diag):
            self.R = np.eye(self.dim_z) * R_diag
        else:
            self.R = np.diag(R_diag)

        # 机体坐标系下五个点的坐标 (中心 + 四个旋翼)
        self.local_coords = None
        if local_coords is not None:
            self.set_local_coords(local_coords)

        # 上次时间戳
        self.last_time = None

        # 标志是否已初始化
        self.initialized = False

    def set_local_coords(self, local_coords):
        """
        设置机体坐标系下的五个点坐标。
        输入: local_coords 可以是 (5,3) 或 (3,5) 数组，其中第一个点必须是中心 (0,0,0)
        """
        arr = np.array(local_coords)
        if arr.shape == (5,3):
            self.local_coords = arr.T  # 转为 (3,5)
        elif arr.shape == (3,5):
            self.local_coords = arr
        else:
            raise ValueError("local_coords must be (5,3) or (3,5)")
        # 确保中心点为 (0,0,0)
        if not np.allclose(self.local_coords[:,0], 0):
            print("Warning: first point (center) should be (0,0,0)")

    def initialize(self, center, rotmat, v_init=None, omega_init=None, P_init=0.1):
        """
        用首次测量初始化状态。
        参数:
            center: 初始中心位置 (3,)
            rotmat: 初始旋转矩阵 (3,3) 或四元数 (4,)
            v_init: 初始速度，默认零
            omega_init: 初始角速度，默认零
            P_init: 初始协方差对角元素值（标量或13维对角）
        """
        if self.local_coords is None:
            raise RuntimeError("Local coordinates must be set before initialization.")

        # 处理姿态输入
        if len(rotmat) == 4:  # 四元数
            q = np.array(rotmat).flatten()
            q = q / np.linalg.norm(q)
        else:
            # 假设是旋转矩阵
            R = np.array(rotmat)
            q = self._rotmat_to_quat(R)

        # 状态向量
        p = np.array(center).flatten()
        v = np.zeros(3) if v_init is None else np.array(v_init).flatten()
        omega = np.zeros(3) if omega_init is None else np.array(omega_init).flatten()
        self.x = np.hstack((p, v, q, omega))

        # 协方差矩阵
        if np.isscalar(P_init):
            self.P = np.eye(self.dim_x) * P_init
        else:
            self.P = np.diag(P_init)

        self.last_time = time.time()
        self.initialized = True

    def predict(self, dt, current_time = None):
        """预测步 (数值雅可比)"""
        if not self.initialized:
            raise RuntimeError("EKF not initialized. Call initialize() first.")

        # 状态转移函数
        def f(state):
            p = state[0:3]
            v = state[3:6]
            q = state[6:10]
            omega = state[10:13]

            # 位置积分
            p_new = p + v * dt
            # 速度（恒速）
            v_new = v
            # 四元数积分
            dq = self._omega_to_quat(omega, dt)
            q_new = self._quat_multiply(q, dq)
            q_new /= np.linalg.norm(q_new)
            # 角速度（恒角速度）
            omega_new = omega
            return np.hstack((p_new, v_new, q_new, omega_new))

        # 计算雅可比 F (13x13)
        F = Jacobian(f)(self.x)

        # 预测状态
        self.x = f(self.x)

        # 离散过程噪声协方差 Qd
        Qd = np.zeros((self.dim_x, self.dim_x))
        dt3 = dt**3 / 3.0
        dt2 = dt**2 / 2.0
        # 位置-速度部分
        Qd[0:3, 0:3] = np.eye(3) * self.qa * dt3
        Qd[0:3, 3:6] = np.eye(3) * self.qa * dt2
        Qd[3:6, 0:3] = np.eye(3) * self.qa * dt2
        Qd[3:6, 3:6] = np.eye(3) * self.qa * dt
        # 角速度部分
        Qd[10:13, 10:13] = np.eye(3) * self.qalpha * dt

        # 预测协方差
        self.P = F @ self.P @ F.T + Qd

        # 保存时间
        self.last_time = current_time

    def update(self, z):
        """更新步 (数值雅可比)"""
        if not self.initialized:
            raise RuntimeError("EKF not initialized. Call initialize() first.")

        # 测量函数 h
        def h(state):
            p = state[0:3]
            q = state[6:10]
            R = self._quat_to_rotmat(q)
            # 计算五个点: p + R * local_coords
            points = p[:, None] + R @ self.local_coords  # (3,5)
            return points.T.flatten()  # 15维

        # 测量雅可比 H (15x13)
        H = Jacobian(h)(self.x)

        # 预测测量
        z_pred = h(self.x)

        # 残差
        y = z - z_pred

        # 残差协方差
        S = H @ self.P @ H.T + self.R

        # 卡尔曼增益
        K = self.P @ H.T @ np.linalg.inv(S)

        # 状态更新
        self.x = self.x + K @ y

        # 四元数归一化
        self.x[6:10] /= np.linalg.norm(self.x[6:10])

        # 协方差更新
        I = np.eye(self.dim_x)
        self.P = (I - K @ H) @ self.P

    def step(self, z, current_time=None):
        """
        执行一次完整的预测+更新步。
        参数:
            z: 测量向量 (15,)
            current_time: 当前时间戳，若为None则使用time.time()
        返回:
            滤波后的位置 (3,) 和欧拉角 (roll, pitch, yaw)
        """
        if current_time is None:
            current_time = time.time()

        if not self.initialized:
            raise RuntimeError("EKF not initialized. Call initialize() first.")

        dt = current_time - self.last_time
        if dt > 0:
            self.predict(dt, current_time)
            self.update(z)
        else:
            # dt <= 0 时只更新（不预测）
            self.update(z)

        return self.get_filtered_position(), self.get_filtered_euler()

    def get_filtered_position(self):
        """返回滤波后的位置 (3,)"""
        return self.x[0:3].copy()

    def get_filtered_velocity(self):
        """返回滤波后的速度 (3,)"""
        return self.x[3:6].copy()

    def get_filtered_quaternion(self):
        """返回滤波后的四元数 (4,)"""
        return self.x[6:10].copy()

    def get_filtered_euler(self):
        """返回滤波后的欧拉角 (roll, pitch, yaw) 弧度"""
        q = self.x[6:10]
        return self._quat_to_euler(q)

    def get_filtered_omega(self):
        """返回滤波后的角速度 (3,)"""
        return self.x[10:13].copy()

    # ------------------ 四元数辅助函数 ------------------
    @staticmethod
    def _quat_multiply(q, r):
        """四元数乘法 q * r (Hamilton)"""
        w1, x1, y1, z1 = q
        w2, x2, y2, z2 = r
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])

    @staticmethod
    def _quat_to_rotmat(q):
        """四元数转旋转矩阵 (3x3)"""
        w,x,y,z = q
        return np.array([
            [1-2*(y*y+z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1-2*(x*x+z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1-2*(x*x+y*y)]
        ])

    @staticmethod
    def _rotmat_to_quat(R):
        """旋转矩阵转四元数"""
        tr = np.trace(R)
        if tr > 0:
            S = np.sqrt(tr+1.0) * 2
            qw = 0.25 * S
            qx = (R[2,1] - R[1,2]) / S
            qy = (R[0,2] - R[2,0]) / S
            qz = (R[1,0] - R[0,1]) / S
        elif (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
            S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            qw = (R[2,1] - R[1,2]) / S
            qx = 0.25 * S
            qy = (R[0,1] + R[1,0]) / S
            qz = (R[0,2] + R[2,0]) / S
        elif R[1,1] > R[2,2]:
            S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            qw = (R[0,2] - R[2,0]) / S
            qx = (R[0,1] + R[1,0]) / S
            qy = 0.25 * S
            qz = (R[1,2] + R[2,1]) / S
        else:
            S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            qw = (R[1,0] - R[0,1]) / S
            qx = (R[0,2] + R[2,0]) / S
            qy = (R[1,2] + R[2,1]) / S
            qz = 0.25 * S
        return np.array([qw, qx, qy, qz])

    @staticmethod
    def _omega_to_quat(omega, dt):
        """角速度积分 -> 四元数增量"""
        theta = np.linalg.norm(omega) * dt
        if theta < 1e-12:
            return np.array([1.0, 0,0,0])
        axis = omega / np.linalg.norm(omega)
        return np.array([np.cos(theta/2),
                         axis[0]*np.sin(theta/2),
                         axis[1]*np.sin(theta/2),
                         axis[2]*np.sin(theta/2)])

    @staticmethod
    def _quat_to_euler(q):
        """四元数转欧拉角 (ZYX顺序: roll, pitch, yaw)"""
        w,x,y,z = q
        # 滚转 (roll)
        sinr_cosp = 2*(w*x + y*z)
        cosr_cosp = 1 - 2*(x*x + y*y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        # 俯仰 (pitch)
        sinp = 2*(w*y - z*x)
        if np.abs(sinp) >= 1:
            pitch = np.sign(sinp) * np.pi/2
        else:
            pitch = np.arcsin(sinp)
        # 偏航 (yaw)
        siny_cosp = 2*(w*z + x*y)
        cosy_cosp = 1 - 2*(y*y + z*z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return np.array([roll, pitch, yaw]) 
# 刚体EKF（LS方法特供版👍）

import numpy as np
from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt, QThread, Signal, QTimer)
import time
import numpy as np
import time

def numerical_jacobian(func, x, epsilon=1e-6):
    """
    计算函数 func 在 x 处的数值雅可比矩阵
    """
    n = len(x)
    f0 = func(x)
    m = len(f0)
    J = np.zeros((m, n))
    for i in range(n):
        x_plus = x.copy()
        x_plus[i] += epsilon
        f_plus = func(x_plus)
        J[:, i] = (f_plus - f0) / epsilon
    return J

import numpy as np
import time

class TightlyCoupledEKF(QThread):
    """
    紧耦合扩展卡尔曼滤波器
    状态向量 (13维): [px, py, pz, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz]
    观测向量: 所有可见关键点的 2D 像素坐标 [u1, v1, u2, v2, ...]
    """
    data = Signal(object, float)
    def __init__(self, local_coords=None, qa=1.0, qalpha=1.0, r_pixel=0.4, parent=None):
        super().__init__(parent)
        """
        参数:
            local_coords: 机体坐标系下5个点的坐标 (5,3) 或 (3,5)
            qa: 加速度过程噪声谱密度
            qalpha: 角加速度过程噪声谱密度
            r_pixel: 像素测量噪声标准差
        """
        self.dim_x = 13
        
        # 状态向量与协方差
        self.x = None
        self.P = None

        # 过程噪声参数
        self.qa = qa
        self.qalpha = qalpha
        self.r_pixel = r_pixel

        # 相机参数 (需在初始化前设置)
        self.camera_params = [] # 列表，元素为 {'R_wc': ..., 't_wc': ..., 'K': ...}
        self.img_width = 853
        self.img_height = 480

        # 机体坐标系下五个点的坐标
        self.local_coords = None
        if local_coords is not None:
            self.set_local_coords(local_coords)

        self.last_time = None
        self.initialized = False

        # 预计算的不变量(3D重建移民)
        self.K_inv = None
        self.camera_positions = None
        self.A_matrices = None
        self.R_wc_matrices = None

    def set_local_coords(self, local_coords):
        arr = np.array(local_coords)
        if arr.shape == (5, 3):
            self.local_coords = arr.T
        elif arr.shape == (3, 5):
            self.local_coords = arr
        else:
            raise ValueError("local_coords must be (5,3) or (3,5)")
        
        if not np.allclose(self.local_coords[:,0], 0):
            print("Warning: first point (center) should be (0,0,0)")

    def set_camera_params(self, camera_params_list, width, height):
        """设置相机参数列表"""
        self.camera_params = camera_params_list
        self.img_width = width
        self.img_height = height

    def initialize(self, center, rotmat, v_init=None, omega_init=None, P_init=0.1):
        """初始化状态"""
        # 姿态处理
        if len(rotmat) == 4:
            q = np.array(rotmat).flatten()
            q = q / np.linalg.norm(q)
        else:
            R = np.array(rotmat)
            q = self._rotmat_to_quat(R)

        # 状态向量赋值
        p = np.array(center).flatten()
        v = np.zeros(3) if v_init is None else np.array(v_init).flatten()
        omega = np.zeros(3) if omega_init is None else np.array(omega_init).flatten()
        self.x = np.hstack((p, v, q, omega))

        # 协方差初始化
        if np.isscalar(P_init):
            self.P = np.eye(self.dim_x) * P_init
        else:
            self.P = np.diag(P_init)

        self.last_time = time.time()
        self.initialized = True

    def process_step(self, all_points_2d, current_time):
        if not self.initialized:
            all_points_3d = self.reconstruct_3d_points_fast(all_points_2d)
            drone_center_np = np.array(all_points_3d[0])
            rotor_world_np = np.array(all_points_3d[1:]).T  # Shape: (3, N)
            rotor_local_np = np.array(rotor_position).T    # Shape: (3, N)

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

            # 5. 初始化
            self.initialize(drone_center_np, R, [0, 0, 0], [0, 0, 0], P_init=0.1)
            self.data.emit(self.x, current_time)
        else:
            # 预测
            dt = current_time - self.last_time
            if dt <= 0:
                dt = 1e-6
            self.predict(dt, current_time)
                
            # 构建紧耦合观测数据
            # 更新
            if all(coord == 0 for row in all_points_2d for point in row for coord in point):
                print("所有点均为0，无有效观测")
            else:
                self.update(all_points_2d)
                    
            self.data.emit(self.x, current_time)

    def predict(self, dt, current_time=None):
        """预测步 (四元数积分)"""
        if not self.initialized:
            raise RuntimeError("EKF not initialized.")

        # 状态转移函数 f
        def f(state):
            p = state[0:3]
            v = state[3:6]
            q = state[6:10]
            w = state[10:13]

            p_new = p + v * dt
            v_new = v
            dq = self._omega_to_quat(w, dt)
            q_new = self._quat_multiply(q, dq)
            q_new /= np.linalg.norm(q_new)
            w_new = w
            return np.hstack((p_new, v_new, q_new, w_new))

        # 计算雅可比 F
        F = self._numerical_jacobian(f, self.x)

        # 预测状态
        self.x = f(self.x)

        # 过程噪声 Q
        Qd = np.zeros((self.dim_x, self.dim_x))
        dt3 = dt**3 / 3.0
        dt2 = dt**2 / 2.0
        Qd[0:3, 0:3] = np.eye(3) * self.qa * dt3
        Qd[0:3, 3:6] = np.eye(3) * self.qa * dt2
        Qd[3:6, 0:3] = np.eye(3) * self.qa * dt2
        Qd[3:6, 3:6] = np.eye(3) * self.qa * dt
        Qd[10:13, 10:13] = np.eye(3) * self.qalpha * dt

        self.P = F @ self.P @ F.T + Qd
        self.last_time = current_time

    def update(self, all_points_2d):
        """
        更新步 (紧耦合)
        all_points_2d: 列表 [8][5] 的结构，每个元素是 [u, v]，若未观测到则为 [0, 0]
        """
        if not self.initialized:
            return

        # 构建测量向量 z 和对应的索引
        z_list = []
        obs_info = [] # 存储 {'cam_idx': ..., 'marker_idx': ...}
        
        for i in range(8):
            for j in range(5):
                u, v = all_points_2d[i][j]
                if u == 0 and v == 0:
                    continue
                z_list.extend([u, v])
                obs_info.append({'cam_idx': i, 'marker_idx': j})
        
        if len(z_list) == 0:
            return # 无有效观测

        z = np.array(z_list)
        
        # 定义测量预测函数 h
        def h(state):
            p = state[0:3]
            q = state[6:10]
            R_wb = self._quat_to_rotmat(q) # Body -> World
            
            preds = []
            for obs in obs_info:
                cam_idx = obs['cam_idx']
                mk_idx = obs['marker_idx']
                
                cam = self.camera_params[cam_idx]
                R_wc = cam['R_wc'] # World -> Camera
                t_wc = cam['t_wc'] # Camera Position in World
                K = cam['K']
                
                # 1. 计算标志点在世界系下的坐标
                # P_w = P_center + R_wb * P_local
                p_local = self.local_coords[:, mk_idx]
                p_world = p + R_wb @ p_local
                
                # 2. 转换到相机坐标系
                # P_c = R_wc * (P_w - t_wc)
                p_cam = R_wc @ (p_world - t_wc)
                
                # 3. 投影
                if p_cam[2] < 0.1:
                    # 无效点，填充0 (后续处理需注意，但这里为了维度一致暂不处理，实际中应剔除)
                    preds.extend([0, 0])
                    continue
                    
                u_proj = K[0,0] * (p_cam[0] / p_cam[2]) + K[0,2]
                v_proj = K[1,1] * (p_cam[1] / p_cam[2]) + K[1,2]
                
                # 4. 坐标翻转 (对应你的 LS 代码逻辑)
                # LS中: u_original = width - u_meas
                # 所以 EKF预测值 h(x) 也需要翻转来匹配测量值 z
                u_pred = self.img_width - u_proj
                #u_pred = u_proj
                v_pred = self.img_height - v_proj
                #v_pred = v_proj
                
                preds.extend([u_pred, v_pred])
            
            return np.array(preds)

        # 计算雅可比 H
        H = self._numerical_jacobian(h, self.x)
        
        # 预测测量
        z_pred = h(self.x)
        
        # 残差
        y = z - z_pred
        #print(y)
        
        # 测量噪声 R
        R = np.eye(len(z)) * (self.r_pixel ** 2)
        
        # 卡尔曼增益
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # 更新状态
        self.x = self.x + K @ y
        self.x[6:10] /= np.linalg.norm(self.x[6:10]) # 四元数归一化
        
        # 更新协方差
        I = np.eye(self.dim_x)
        self.P = (I - K @ H) @ self.P

    # ------------------ 辅助函数 ------------------
    
    @staticmethod
    def _numerical_jacobian(func, x, epsilon=1e-6):
        m = len(func(x))
        n = len(x)
        J = np.zeros((m, n))
        for i in range(n):
            x_plus = x.copy()
            x_plus[i] += epsilon
            f_plus = func(x_plus)
            J[:, i] = (f_plus - func(x)) / epsilon
        return J

    @staticmethod
    def _quat_multiply(q, r):
        w1, x1, y1, z1 = q
        w2, x2, y2, z2 = r
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])

    @staticmethod
    def _quat_to_rotmat(q):
        w, x, y, z = q
        return np.array([
            [1-2*(y*y+z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1-2*(x*x+z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1-2*(x*x+y*y)]
        ])

    @staticmethod
    def _rotmat_to_quat(R):
        tr = np.trace(R)
        if tr > 0:
            S = np.sqrt(tr+1.0) * 2
            qw = 0.25 * S
            qx = (R[2,1] - R[1,2]) / S
            qy = (R[0,2] - R[2,0]) / S
            qz = (R[1,0] - R[0,1]) / S
        elif (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
            S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            qw = (R[2,1] - R[1,2]) / S
            qx = 0.25 * S
            qy = (R[0,1] + R[1,0]) / S
            qz = (R[0,2] + R[2,0]) / S
        elif R[1,1] > R[2,2]:
            S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            qw = (R[0,2] - R[2,0]) / S
            qx = (R[0,1] + R[1,0]) / S
            qy = 0.25 * S
            qz = (R[1,2] + R[2,1]) / S
        else:
            S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            qw = (R[1,0] - R[0,1]) / S
            qx = (R[0,2] + R[2,0]) / S
            qy = (R[1,2] + R[2,1]) / S
            qz = 0.25 * S
        return np.array([qw, qx, qy, qz])

    @staticmethod
    def _omega_to_quat(omega, dt):
        theta = np.linalg.norm(omega) * dt
        if theta < 1e-12:
            return np.array([1.0, 0, 0, 0])
        axis = omega / np.linalg.norm(omega)
        return np.array([np.cos(theta/2),
                         axis[0]*np.sin(theta/2),
                         axis[1]*np.sin(theta/2),
                         axis[2]*np.sin(theta/2)])

    @staticmethod
    def _quat_to_euler(q):
        w, x, y, z = q
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        sinp = 2*(w*y - z*x)
        pitch = np.arcsin(np.clip(sinp, -1, 1))
        yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return np.array([roll, pitch, yaw])

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
                    except np.linalg.LinAlgError:
                        print(f"物体 {j+1} 的线性方程求解失败。")
                        reconstructed_points.append(np.array([np.nan, np.nan, np.nan]))
                else:
                    print(f"物体 {j+1} 的有效观测点不足，无法重建。")
                    reconstructed_points.append(np.array([np.nan, np.nan, np.nan]))
                    
            return reconstructed_points
        # 重建3D点

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
                self.R_wc_matrices.append(R_wc)
            
            print("3D重建参数（EKF）预计算完成")
            
        except Exception as e:
            print(f"初始化3D重建参数时出错: {e}")
            import traceback
            traceback.print_exc()
    # 在开始工作前预计算所有不变量
