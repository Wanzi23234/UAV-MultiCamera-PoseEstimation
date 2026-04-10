import cv2
import os

x, y = 11, 529  
width, height = 582, 480

# 输入文件夹路径
input_folder = "D:\\Unreal Projects\\try2\\Saved\\BugIt\\Windows"
# 输出文件夹路径
output_folder = "D:\\Unreal Projects\\try2\\Saved\\BugIt\\Windows_Cropped"

# 创建输出文件夹（如果不存在）
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

for filename in os.listdir(input_folder):
    # 检查是否为图片文件
    if filename.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tiff')):
        # 正确的文件路径拼接
        input_path = os.path.join(input_folder, filename)
        
        # 读取图片
        image = cv2.imread(input_path)
        
        # 检查图片是否成功读取
        if image is not None:
            # 裁剪图片
            cropped_image = image[y:y+height, x:x+width]
            
            # 输出文件路径
            output_path = os.path.join(output_folder, filename)
            
            # 保存裁剪后的图片
            cv2.imwrite(output_path, cropped_image)
            print(f"Processed: {filename}")
        else:
            print(f"Failed to read: {filename}")
            