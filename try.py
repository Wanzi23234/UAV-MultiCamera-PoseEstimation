import subprocess

# 定义可执行文件的完整路径
exe_path = r"C:\Users\whz\Desktop\WindowsNoEditor\try2.exe"

try:
    # 使用 subprocess.Popen 启动进程
    process = subprocess.Popen(exe_path)
    print(f"成功启动 {exe_path}，进程 ID: {process.pid}")
except FileNotFoundError:
    print(f"错误：找不到文件 {exe_path}")
except Exception as e:
    print(f"启动失败，原因：{e}")