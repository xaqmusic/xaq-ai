import sys
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame
from PyQt6.QtCore import Qt, QPoint

app = QApplication(sys.argv)
container = QWidget()
layout = QVBoxLayout(container)

track_widget = QWidget()
track_layout = QHBoxLayout(track_widget)

info_panel = QFrame()
info_panel.setFixedWidth(150)
track_layout.addWidget(info_panel)

canvas1 = QWidget()
track_layout.addWidget(canvas1)

layout.addWidget(track_widget)

container.show()

# Test mapping
p_topleft = canvas1.mapTo(container, canvas1.rect().topLeft())
# p_topleft should be QPoint(150+margins?, layout_margins?)
print("Mapping result:", p_topleft)
