# PyQt
from PyQt5 import QtCore, QtWidgets, uic

# ros
import rospy
from std_msgs.msg import Int32MultiArray, Float32

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








