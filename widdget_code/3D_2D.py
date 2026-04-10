import json
import os
import numpy as np
from global_import import(split_picture_folder, calculate_camera_position_bias, angle, l, width, height, K, rotor_position)
import cv2

def calculate_rotor_world_positions(drone_pos, drone_orient, rotor_local_coords):
    """
    计算所有旋翼在UE4世界坐标系中的位置 (仅使用numpy)
    
    Args:
        drone_pos (dict): 无人机在UE4世界中的位置
        drone_orient (dict): 无人机在AirSim中的姿态
        rotor_local_coords (list): 旋翼在机体坐标系中的局部坐标列表
        
    Returns:
        list: 包含无人机中心和所有旋翼的世界坐标字典列表
    """
    # 1. 将AirSim的欧拉角转换为UE4的欧拉角
    roll_rad = drone_orient['roll']
    pitch_rad = drone_orient['pitch']
    yaw_rad = drone_orient['yaw'] # 关键：Yaw取反
    print(f"Roll: {roll_rad}, Pitch: {pitch_rad}, Yaw: {yaw_rad}")

    # 2. 使用numpy构建UE4左手系的旋转矩阵
    
    # 绕X轴旋转矩阵 (Roll)
    Rx = np.array([
        [1, 0,             0            ],
        [0, np.cos(roll_rad), np.sin(roll_rad)],
        [0, -np.sin(roll_rad), np.cos(roll_rad)]
    ])
    
    # 绕Y轴旋转矩阵 (Pitch)
    Ry = np.array([
        [np.cos(pitch_rad), 0, -np.sin(pitch_rad)],
        [0,                 1,  0                 ],
        [np.sin(pitch_rad), 0,  np.cos(pitch_rad)]
    ])
    
    # 绕Z轴旋转矩阵 (Yaw)
    Rz = np.array([
        [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
        [np.sin(yaw_rad),  np.cos(yaw_rad), 0],
        [0,                0,               1]
    ])

    # 组合旋转矩阵 R = Rz * Ry * Rx (ZYX欧拉角顺序)
    rotation_matrix = Rz @ Ry @ Rx

    # 3. 无人机在UE4中的世界坐标 (单位: 厘米)
    drone_world_pos_np = np.array([
        drone_pos["x"] * 100, 
        drone_pos["y"] * 100, 
        -drone_pos["z"] * 100  # 注意Z轴方向
    ])

    # 4. 计算所有点的世界坐标
    all_world_positions = [drone_pos] # 先加入无人机中心

    for rotor_local in rotor_local_coords:
        rotor_local_np = np.array(rotor_local)
        
        # 应用核心变换公式: P_world = (R @ P_local) + P_object_world
        rotor_world_np = (rotation_matrix @ rotor_local_np) + drone_world_pos_np
        
        all_world_positions.append({
            "x": rotor_world_np[0] / 100,
            "y": rotor_world_np[1] / 100,
            "z": -rotor_world_np[2] / 100
        })
        
    return all_world_positions
def safe_read_drone_position(json_file_path):
    """
    安全地读取JSON文件中的无人机位置信息
    
    Args:
        json_file_path (str): JSON文件路径
        
    Returns:
        dict: 包含x, y, z坐标的字典，如果读取失败则返回None
    """
    try:
        # 检查文件是否存在
        if not os.path.exists(json_file_path):
            print(f"错误: 文件不存在 {json_file_path}")
            return None
            
        # 检查是否为文件（而非目录）
        if not os.path.isfile(json_file_path):
            print(f"错误: 路径不是文件 {json_file_path}")
            return None
            
        # 尝试打开并解析JSON文件
        with open(json_file_path, 'r', encoding='utf-8') as file:
            data = json.load(file)
            
        position = data["drone_info"]["position"]
        orientation = data["drone_info"]["orientation"]
                
        # 返回位置信息
        return (
            {
            "x": float(position["x"]),
            "y": float(position["y"]),
            "z": float(position["z"])
            },
            {
            "roll": float(orientation["roll"]),
            "pitch": float(orientation["pitch"]),
            "yaw": float(orientation["yaw"])
            }
        )
        
    except json.JSONDecodeError as e:
        print(f"错误: JSON格式无效 {json_file_path}: {e}")
        return None
    except ValueError as e:
        print(f"错误: 坐标值无法转换为浮点数 {json_file_path}: {e}")
        return None
    except Exception as e:
        print(f"错误: 读取文件时发生未知错误 {json_file_path}: {e}")
        return None
def calculate_projection_points(position, camera_positions):
    """
    计算物体在8个相机视角下的投影点
    """
    # 将物体位置从米转换为厘米（保持UE4坐标系）
    object_pos = np.array([position["x"] * 100, position["y"] * 100, - position["z"] * 100])
    
    projection_points = []
    
    for i, cam_pos_normalized in enumerate(camera_positions):
        # 实际相机位置
        camera_pos = np.array(cam_pos_normalized) * l
        
        # 1. 定义相机在世界坐标系中的三个基向量
        # 相机前向向量 (UE4的X轴)
        forward_vec = (-camera_pos) / np.linalg.norm(camera_pos)
        
        # 世界“上”向量
        world_up_vec = np.array([0, 0, 1])
        
        # 相机右向量 (UE4的Y轴)，注意叉乘顺序
        right_vec = np.cross(forward_vec, world_up_vec)
        if np.linalg.norm(right_vec) < 1e-6: # 处理相机垂直向上或向下的特殊情况
            right_vec = np.array([1, 0, 0])
        right_vec /= np.linalg.norm(right_vec)
        
        # 相机上向量 (UE4的Z轴)
        up_vec = np.cross(right_vec, forward_vec)
        
        # 2. 构建从世界坐标系到相机坐标系的旋转矩阵 R_wc
        # R的行向量就是相机坐标系的三个基向量在世界坐标系下的表示
        R = np.vstack((right_vec, up_vec, forward_vec))
        
        # 将物体位置转换到相机坐标系
        point_world_relative = object_pos - camera_pos
        point_camera = R @ point_world_relative
        
        # 如果点在相机后面，则跳过
        if point_camera[2] <= 0:
            projection_points.append((np.nan, np.nan, 1))
            print(f"  点在相机后面，跳过")
            continue
        
        # 投影到图像平面
        point_image = K @ point_camera
        u = point_image[0] / point_image[2]
        v = point_image[1] / point_image[2]
        
        # 由于相机坐标系Y轴向上，而图像坐标系V轴向下，需要翻转Y坐标
        v = height - v
        u = width - u
        
        # 检查点是否在图像范围内
        if 0 <= u <= width and 0 <= v <= height:
            projection_points.append((u, v, 2))  # visibility=2表示可见
        else:
            projection_points.append((u, v, 1))  # visibility=1表示存在但不可见（超出边界）
    
    return projection_points
def draw_projection_points_on_images(base_name, output_folder, projections):
    """
    在切割好的图片上标注投影点
    """
    for i, proj_point in enumerate(projections):
        if proj_point is None or (np.isnan(proj_point[0]) and np.isnan(proj_point[1])):
            continue
            
        u, v, x = proj_point
        # 检查点是否在图像范围内
        if not (0 <= u <= width and 0 <= v <= height):
            continue
            
        # 构造切割后图片的文件名
        image_path = os.path.join(output_folder, f"{base_name}_{i + 1}.png")
        
        # 检查图片文件是否存在
        if not os.path.exists(image_path):
            print(f"警告: 图片文件不存在 {image_path}")
            continue
            
        # 读取图片
        image = cv2.imread(image_path)
        if image is None:
            print(f"警告: 无法读取图片 {image_path}")
            continue
            
        # 在图片上绘制点（尝试两种方式，看哪种正确）
        center_original = (int(u), int(v))  # 原始坐标
        center_flipped = (int(u), int(height - v))  # Y轴翻转
        
        # 先用原始坐标绘制
        cv2.circle(image, center_original, 5, (0, 0, 255), -1)
        cv2.circle(image, center_original, 7, (255, 255, 255), 2)
        
        # 添加坐标文本
        text = f"({u:.1f}, {v:.1f})"
        cv2.putText(image, text, (center_original[0] + 10, center_original[1] - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        
        # 保存标注后的图片（覆盖原图）
        cv2.imwrite(image_path, image)
        print(f"已在 {os.path.basename(image_path)} 上标注点 ({u:.1f}, {v:.1f})")

input_folder = 'D:\\anaconda\\envs\\Shining\\shin\\widdget_code\\WindowsNoEditor\\try2\\Saved\\BugIt\\WindowsNoEditor'
output_folder = 'D:\\anaconda\\envs\\Shining\\shin\\widdget_code\\WindowsNoEditor\\try2\\Saved\\BugIt\\WindowsNoEditor_Cropped'
l = 1600
# 读取特定的记录文件
record_path = 'D:\\anaconda\\envs\\Shining\\shin\\widdget_code\\recorder'

for filename in os.listdir(input_folder):
    if filename.endswith('.png'):
        print(f"处理图片: {filename}")
        name, ext = os.path.splitext(filename)

        points = []
        
        # 获取对应的记录文件
        record_file = os.path.join(record_path, f'record_{int(name[-3:])}.json')
        position, oriention = safe_read_drone_position(record_file)
        positions = calculate_rotor_world_positions(position, oriention, rotor_position)
        #print(positions)
        
        # 先进行图片切割
        split_picture_folder(input_folder, output_folder, filename)

        for position in positions:
            print(f"物体世界坐标: ({position['x']}, {position['y']}, {position['z']})")
            # 计算相机位置（根据当前angle值）
            current_camera_positions = calculate_camera_position_bias(angle)
    
            # 计算投影点
            projections = calculate_projection_points(position, current_camera_positions)
    
            print(f"物体在8个相机视角下的投影点:")
            for i, proj_point in enumerate(projections):
                if proj_point:
                    u, v, visibility = proj_point
                    visibility_text = ["未标记", "不可见", "可见"][visibility]
                    print(f"  相机{i+1}: ({u:.2f}, {v:.2f}) - {visibility_text}")

            # 在切割好的图片上标注投影点
            #draw_projection_points_on_images(name, output_folder, projections)

            points.append(projections)

        points_array = np.array(points)
        points_transposed = points_array.transpose(1, 0, 2)  # 转置
        result_data = {}

        # 按图片存储
        for i in range(8):  # 8个相机视角
            camera_projections = points_transposed[i]
            formatted_points = []
            for point in camera_projections:
                if point is not None:
                    # 保持浮点数格式，包括可见性信息
                    formatted_points.append([float(point[0]), float(point[1]), int(point[2])])
                else:
            # 对于完全缺失的点，使用null并标记为未标记
                    formatted_points.append(None)
    
            result_data[f"picture{i+1}"] = formatted_points

        # 保存JSON文件，每个filename产出一份JSON
        json_filename = os.path.join(output_folder, f'{name}_projections.json')
        with open(json_filename, 'w') as f:
            json.dump(result_data, f, indent=2)

        print(f"已保存投影点数据到 {json_filename}")