from main_log import Stats
import sys
import os
from PySide6 import QtWidgets
from PySide6.QtGui import QIcon
from PySide6.QtWidgets import QApplication
from qt_material import apply_stylesheet
from start_window import MainWindow

# 创建一个QApplication实例，即构建窗口
app = QtWidgets.QApplication(sys.argv)
# 设置应用程序图标
icon_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'picture', 'logo_transparent.ico')
app.setWindowIcon(QIcon(icon_path))

# 添加炫酷的实例
apply_stylesheet(app, theme='dark_teal.xml')

# 显示启动窗口
start_window = MainWindow()
start_window.center_on_screen()  # 将窗口居中
start_window.show()
app.exec()  # 运行启动窗口直到关闭

# 启动窗口关闭后，运行主程序
stats = Stats()
stats.window.show()
sys.exit(app.exec())  # 运行主程序