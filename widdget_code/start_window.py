from PySide6.QtCore import Qt, QPropertyAnimation, QEasingCurve, Property, QSize, QRect, QPoint
from PySide6.QtGui import QPainter, QPixmap, QScreen, QColor
from PySide6.QtWidgets import QLabel, QApplication, QWidget, QVBoxLayout
import os

class LogoImg(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.angle = 0
        self.imgAmimation = QPropertyAnimation(self, b'angle')
        self.imgAmimation.setDuration(960)  # 设置动画的持续时间，以毫秒为单位 1秒 = 1000毫秒
        self.imgAmimation.setStartValue(0)
        self.imgAmimation.setEndValue(360)
        self.imgAmimation.setEasingCurve(QEasingCurve.Type.InOutBack)  # 设置执行时间曲线(可以简单理解为执行的不同效果)
        self.imgAmimation.setLoopCount(3)  # 重复次数(-1为无数次)
        self.imgAmimation.finished.connect(self.on_animation_finished)  # 连接动画完成信号
        self.imgAmimation.start()  # 启动动画

    @Property(int)  # 注意这里是PySide6封装的装饰器
    def angle(self):
        return self._angle

    @angle.setter
    def angle(self, value):
        self._angle = value  # 注意这里声明的是一个内部变量，通过这个内部变量去访问修改angle
        self.update()  # 更新绘制(调用PainterEvent)

    def paintEvent(self, event):
        painter = QPainter(self)
        path = os.path.join(os.path.dirname(__file__), 'picture\\logo_transparent.png')
        pixmap = QPixmap(path)

        # 设置目标大小
        target_size = QSize(200, 200)  # 例如，设置图片大小为 200x200 像素

        # 调整 pixmap 大小
        scaled_pixmap = pixmap.scaled(target_size, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)

        x, y = (self.geometry().size() - scaled_pixmap.size()).toTuple()
        painter.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)  # 设置渲染提示以达到不失真
        painter.translate(self.width() / 2, self.height() / 2)  # 平移到旋转中心
        painter.rotate(self.angle)  # 旋转
        painter.translate(-self.width() / 2, -self.height() / 2)  # 平移回原位置
        painter.drawPixmap(x / 2, y / 2, scaled_pixmap)
    
    def on_animation_finished(self):
        """动画完成时的处理函数"""
        # 获取父窗口并关闭
        if self.parent():
            self.parent().close()

# 示例主窗口
class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Startup Animation")
        self.setGeometry(100, 100, 400, 400)

        # 设置窗口透明
        self.setAttribute(Qt.WA_TranslucentBackground)
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)
        self.setStyleSheet("background-color: transparent;")  # 设置背景颜色为透明

        layout = QVBoxLayout()
        self.logo_img = LogoImg(self)
        layout.addWidget(self.logo_img)
        self.setLayout(layout)

    def center_on_screen(self):
        screen_geometry = QApplication.primaryScreen().geometry()
        window_geometry = self.frameGeometry()
        center_point = screen_geometry.center()
        window_geometry.moveCenter(center_point)
        self.move(window_geometry.topLeft())