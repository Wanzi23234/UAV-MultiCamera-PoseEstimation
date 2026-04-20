
---

# 基于多相机纯视觉与多算法结算的室内无人机位姿估计系统

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.8](https://img.shields.io/badge/python-3.8-blue.svg)](https://www.python.org/downloads/release/python-380/)
[![UE4](https://img.shields.io/badge/UE4-4.27%2B-important)](https://www.unrealengine.com/)
[![YOLOv8](https://img.shields.io/badge/YOLOv8-Ultralytics-blueviolet)](https://github.com/ultralytics/ultralytics)

## 📖 项目简介

本项目是一套面向室内无GPS环境下的**高精度无人机位姿识别全链路系统**。依托 **UE4 + AirSim** 高保真仿真平台，结合 **YOLOv8-pose** 关键点检测与**紧耦合扩展卡尔曼滤波（Tightly-Coupled EKF）**，实现了在复杂工况（遮挡、高机动、视觉干扰）下的鲁棒位姿估计。

系统覆盖了**仿真环境搭建 → 自动化数据生成 → 前端深度学习检测 → 后端多算法位姿解算 → 三维可视化**的全流程，旨在为室内无人机视觉导航研究提供一个可复用、可扩展的基准平台。

## ✨ 主要特性

- **高保真仿真环境**：基于 UE4 与 AirSim，支持多场景（白房、积木房、室外）及多相机（8路）同步采集。
- **自动化数据流水线**：图形化控制端，一键生成带精确真值的 YOLO 格式关键点标注数据。
- **递进式后端算法**：内置三种位姿估计算法：
  - 纯几何最小二乘法（LS + Kabsch）
  - 松耦合扩展卡尔曼滤波（LS + EKF）
  - **紧耦合扩展卡尔曼滤波（Tightly-Coupled EKF）**
- **实时可视化**：3D 空间骨架实时绘制、误差监测面板、轨迹对比图。
- **系统级效能优化**：紧耦合架构通过消除三维中间态，实际运行效率优于传统松耦合方案。

## 📂 项目结构

```
.
├── control_code/                 # 核心控制与算法代码
│   ├── YOLO_try/                 # YOLOv8 训练相关脚本与权重
│   ├── collect.py                #飞行控制辅助代码（没啥用，不用看）
│   ├── split.py                  #数据集制造辅助代码
├── test/                         # 单元测试与调试脚本
├── widdget_code/                 # 自定义 PyQt 控件与可视化组件
│   ├── Shining.py                # 主程序入口（仿真控制端）
│   ├── Shining.py                # 数据集制造辅助代码（运行完这个运行split.py）
│   ├── main_log.py               # 主函数（仿真控制端）
│   ├── start_window.py           # 启动界面（非主要）
│   ├── global_import.py          # 全局变量与参数定义（含相机内参、EKF调参等）
│   ├── recorder/                 # 自动生成的飞行记录数据（JSON）
│   ├── WindowsNoEditor/try2/connect/ # UE4 通信目录（触发文件）
└── requirements.txt              # Python 依赖列表
```
(你看着具体下载就是了，反正能跑😁)

## 🛠️ 环境配置

### 1. 基础环境
- **操作系统**：Windows 10/11
- **Python**：3.8.20
- **GPU**：NVIDIA GeForce RTX 系列（推荐，用于 YOLO 推理加速）
- **UE4 项目**：需自行配置 AirSim 插件并放置于 `WindowsNoEditor` 目录下

### 2. 安装依赖
```bash
pip install -r requirements.txt
```
（🤣装炸了别怪我，这个AI生成的，真要跑建议对着main_log.py配一下，基本都是常用包）

主要依赖包：
- `PySide6`：图形界面框架
- `qt-material`：界面美化主题
- `airsim`：AirSim Python API
- `ultralytics`：YOLOv8 推理
- `numpy`, `scipy`, `matplotlib`：科学计算与绘图
- `pyqtgraph`：3D 实时渲染
- `opencv-python`：图像处理
- `numdifftools`：数值雅可比计算（紧耦合 EKF 依赖）

### 3. YOLO 模型权重
将训练好的 YOLOv8-pose 权重文件 `best.pt` 放置于 `control_code/` 目录下，或在 `global_import.py` 中修改 `pt_name` 变量指向正确路径。

### 4. UE4 仿真项目
请确保 UE4 项目已正确配置 AirSim，并将项目文件夹命名为 `WindowsNoEditor` 置于项目根目录（或修改 `global_import.py` 中的相关路径）。

## ⚙️ 关键参数配置

在 `global_import.py` 中可调整以下核心参数：

| 变量名 | 含义 | 默认值 |
| :--- | :--- | :--- |
| `K` | 相机内参矩阵（3x3） | 仿真标定值 |
| `rotor_position` | 无人机四个旋翼的机体坐标（mm） | DJI Mini 4 Pro 骨架 |
| `l` | 相机分布球面半径（cm） | 1600 |
| `angle` | 相机俯仰角（度） | 45 |
| `picture_split` | 8路拼接图切割坐标 | 预设值 |
| `R_diag_global` | 松耦合 EKF 观测噪声方差 | 10 |
| `r_pixel` | 紧耦合 EKF 像素观测标准差 | 0.5 |
| `q_a_global` | 线加速度过程噪声谱密度 | 500 |
| `q_alpha_global` | 角加速度过程噪声谱密度 | 0.2 |
| `manual_delay` | 松耦合延迟补偿（秒） | 0.1 |

## 🚀 运行指南

### 1. 启动 UE4 仿真环境
- 双击 `WindowsNoEditor/try2.exe` 启动 UE4 项目，或者直接从代码控制端打开
- 确认 AirSim 已正确加载，无人机模型出现在场景中。

### 2. 运行主控制端
```bash
cd widdget_code
python Shining.py
```

### 3. 操作流程
1. **连接 AirSim**：点击界面“开启仿真数据收集端口”按钮，建立与 UE4 的通信。
2. **选择算法模式**：😅不给你选，三个算法一起跑，本关考验你电脑性能。（是做优化了的，如果带不动记得限制一下图传）
3. **轨迹设置**：选择预设轨迹（圆形、方形、8字）或启用“随机飞行”。
4. **干扰配置**：可勾选“落叶干扰”、“相机屏蔽”等选项进行鲁棒性测试。
5. **开始实验**：点击“开始”按钮，无人机将按设定飞行，右侧面板实时显示位姿估计结果与重投影误差。
6. **数据记录**：所有飞行真值与估计值将自动保存至 `recorder/` 目录下的 JSON 文件。

### 4. 可视化窗口
- **3D 骨架图**：实时绘制无人机 5 个关键点（中心 + 4 旋翼）的空间位置。
- **误差监测面板**：动态显示 `(x, y, z, roll, pitch, yaw)` 的数值及与真值的差值。
- **轨迹对比图**：飞行结束后自动绘制真值轨迹与估计轨迹的 3D 对比图。

## 🧪 核心算法说明

### 紧耦合扩展卡尔曼滤波 (Tightly-Coupled EKF)
本项目的核心创新点。传统松耦合 EKF 将三维重建作为中间观测，易引入“二次损耗”导致精度下降甚至发散。紧耦合 EKF 直接将**二维像素坐标**作为观测输入，通过实时计算投影雅可比矩阵，在像素残差层面对状态量进行修正。（没那么玄乎，其实调对了参数在低频图传下各方面都是松耦合占优）

**状态向量** (13维)：
```
[px, py, pz, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz]
```
**观测模型**：
对于每个被检测到的关键点，预测其像素坐标并与 YOLO 输出比较，构建残差。

该架构能充分利用多相机的冗余观测信息，在部分遮挡或观测噪声较大时仍能保持稳定估计。

## 📊 实验数据示例

| 工况 | LS 平均误差 (px) | 松耦合 EKF (px) | 紧耦合 EKF (px) |
| :--- | :---: | :---: | :---: |
| 白房圆形轨迹 | 1.26 | 2.65 | **1.09** |
| 随机高机动 | 2.70 | 13.70 (崩溃) | **3.13** |
| 视觉干扰 (落叶) | 2.81 | 3.12 | **2.50** |
| 2台相机退化 | 3.03 | 3.08 | **2.11** |

*详细分析请参考项目进度报告或发表论文。*

## 🙏 致谢与引用

如果您发现本项目对您的研究有帮助，请引用以下工作：
```
@misc{wang2026uavpose,
  author = {丸子🍡},
  title = {基于多相机纯视觉与紧耦合EKF的室内无人机位姿估计系统},
  year = {2026},
  publisher = {GitHub},
  journal = {GitHub repository},
  howpublished = {\url{https://github.com/Wanzi23234/UAV-MultiCamera-PoseEstimation}}
}
```

本项目受到以下开源项目的启发：
- [Microsoft AirSim](https://github.com/microsoft/AirSim)
- [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics)
- [PySide6](https://wiki.qt.io/Qt_for_Python)

---

**作者**：丸子 
**联系方式**：2952749640@qq.com  
**项目状态**：积极维护中，欢迎 Star ⭐ 和 Issue 反馈。
