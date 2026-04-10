import airsim
import time
import json
import os
from datetime import datetime
import threading
import random

# 全局变量控制程序运行
running = True
record_count = 0

# 定义飞行空间范围（立方体边界）
X_MIN, X_MAX = -10, 10
Y_MIN, Y_MAX = -10, 10
Z_MIN, Z_MAX = 0, -6

def keyboard_listener():
    """监听键盘输入，按下1键停止程序"""
    global running
    while running:
        try:
            # 简单的键盘监听 - 在Windows上可以使用msvcrt
            import msvcrt
            if msvcrt.kbhit():
                key = msvcrt.getch().decode('utf-8')
                if key == '1':
                    print("检测到按键'1'，准备停止程序...")
                    running = False
                    break
        except ImportError:
            # 非Windows系统可以使用其他方式
            import sys, select
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key == '1':
                    print("检测到按键'1'，准备停止程序...")
                    running = False
                    break
        time.sleep(0.1)

def generate_random_waypoint():
    """在指定立方体内生成随机航点"""
    x = random.uniform(X_MIN, X_MAX)
    y = random.uniform(Y_MIN, Y_MAX)
    z = random.uniform(Z_MIN, Z_MAX)
    return x, y, z

client = airsim.MultirotorClient()
client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync().join()

def create_trigger_file(record_name):
    """创建触发文件通知UE4拍照"""
    # 创建统一的通信目录
    comm_dir = "C:\\Users\\whz\\Desktop\\WindowsNoEditor\\try2\\connect"
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

def create_record_file(record_name, drone_info):
    """创建记录文件保存无人机信息"""
    # 创建统一的保存目录
    comm_dir = "D:\\anaconda\\envs\\Shining\\shin\\recorder"
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

def get_drone_info():
    """获取无人机详细信息"""
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

# 启动键盘监听线程
keyboard_thread = threading.Thread(target=keyboard_listener, daemon=True)
keyboard_thread.start()

# 飞行和触发拍照
print("上升到3m高度...")
client.moveToZAsync(-3, 1).join()

print("开始在随机航点间飞行并记录...")

# 每隔3秒记录一次，并飞向新的随机位置
while running:
    # 生成随机航点
    target_x, target_y, target_z = generate_random_waypoint()
    
    # 飞向随机位置（速度设为5m/s）
    print(f"飞往随机位置: ({target_x:.1f}, {target_y:.1f}, {target_z:.1f})")
    client.moveToPositionAsync(target_x, target_y, target_z, 5)
    
    # 获取无人机信息并创建触发文件
    drone_info = get_drone_info() 
    record_name = f"{record_count}"
    print(f"记录 {record_name}, 位置: {drone_info['position']}, "
          f"俯仰: {drone_info['orientation']['pitch']:.2f}, "
          f"偏航: {drone_info['orientation']['yaw']:.2f}")
    create_trigger_file(record_name)
    create_record_file(record_name, drone_info)
    
    record_count += 1
    
    # 等待3秒
    time.sleep(3)

# 取消当前飞行任务
client.cancelAllTasks()
client.hoverAsync().join()

# 降落
print("降落...")
client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)
print("任务完成")