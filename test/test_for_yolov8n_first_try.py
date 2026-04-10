from ultralytics import YOLO
import pathlib

# 1. 配置路径（根据你自己的路径修改）
DATA_YAML = "datasets/uav_pose/uav_pose.yaml"   # 上一步那个 YAML
MODEL_PTH  = "runs/pose/train/weights/best.pt"  # 训练出来的 best.pt

# 2. 加载模型
model = YOLO(MODEL_PTH)

# 3. 运行验证（使用 COCO 风格的指标）
# 这里会自动按 YAML 里的数据集、kpt_shape、flip_idx 等配置进行评估<span data-allow-html class='source-item source-aggregated' data-group-key='source-group-12' data-url='https://docs.ultralytics.com/modes/val/' data-id='turn3fetch0'><span data-allow-html class='source-item-num' data-group-key='source-group-12' data-id='turn3fetch0' data-url='https://docs.ultralytics.com/modes/val/'><span class='source-item-num-name' data-allow-html>ultralytics.com/modes/val/</span><span data-allow-html class='source-item-num-count'></span></span></span>
metrics = model.val(
    data=DATA_YAML,   # 数据集 YAML
    imgsz=640,        # 验证时输入尺寸（和训练时保持一致）
    batch=16,         # 根据显存调整
    plots=True,       # 生成 PR 曲线、混淆矩阵、预测 vs GT 的可视化图
    save_json=False,  # 如需保存 COCO 风格 JSON（用于提交或后续分析），改为 True
    device="0",       # GPU id，有 GPU 写 "0"，CPU 写 "cpu"
)

# 4. 打印关键指标（COCO 风格）
print("\n========== COCO 风格指标汇总 ==========")

# 检测框相关（B = Box）
print("检测框（Box）：")
print(f"  mAP50-95(B) = {metrics.box.map:.4f}")
print(f"  mAP50(B)    = {metrics.box.map50:.4f}")
print(f"  mAP75(B)    = {metrics.box.map75:.4f}")
print(f"  各类 mAP    = {metrics.box.maps}")  # 每个类别的 mAP50-95 列表

# 关键点相关（P = Pose / Keypoints）
print("\n关键点（Pose）：")
print(f"  mAP50-95(P) = {metrics.pose.map:.4f}")
print(f"  mAP50(P)    = {metrics.pose.map50:.4f}")
print(f"  mAP75(P)    = {metrics.pose.map75:.4f}")
print(f"  各类 mAP    = {metrics.pose.maps}")

# 如果你想要一个简单的“综合分数”，可以自己加个平均分，例如：
avg_map = (metrics.box.map + metrics.pose.map) / 2.0
print(f"\n自定义综合 mAP（Box+Pose 平均）= {avg_map:.4f}")