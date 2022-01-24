# PyQt
from PyQt5 import QtCore, QtWidgets, uic

# ros
import rospy
from std_msgs.msg import Int32MultiArray, Float64MultiArray

# oters
import time
import numpy as np

class Ui(QtWidgets.QWidget):
    def __init__(self):
        super(Ui, self).__init__()
        wid = uic.loadUi('vertices.ui', self)
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(wid)
        self.setLayout(layout)

        self.__sub = rospy.Subscriber('/optimizer/vertices', Int32MultiArray, self.callback)

    def callback(self, data):
        self.findChild(QtWidgets.QLCDNumber, 'num_fixed_vertices').display(data.data[0])
        self.findChild(QtWidgets.QLCDNumber, 'num_active_vertices').display(data.data[1])

class Ui_error(QtWidgets.QWidget):
    def __init__(self, counter):
        super(Ui_error, self).__init__()
        layout = QtWidgets.QVBoxLayout()
        wid_list = list()
        for i in range(counter):
            wid = uic.loadUi('error.ui')
            wid.findChild(QtWidgets.QLabel, 'label').setText('Error vertex' + str(i))
            wid_list.append(wid)
            layout.addWidget(wid)
        self.setLayout(layout)

class VisionTime(QtWidgets.QWidget):
    def __init__(self):
        super(VisionTime, self).__init__()
        layout = QtWidgets.QVBoxLayout()
        wid = uic.loadUi('vision_times.ui')
        layout.addWidget(wid)
        self.setLayout(layout)

        rospy.Subscriber('/times', Float64MultiArray, self.callback)

    def callback(self, data):
        self.findChild(QtWidgets.QDoubleSpinBox, 'voxel_time').setValue(data.data[0])
        self.findChild(QtWidgets.QDoubleSpinBox, 'segmentation_time').setValue(data.data[1])
        self.findChild(QtWidgets.QDoubleSpinBox, 'outlier_time').setValue(data.data[2])








