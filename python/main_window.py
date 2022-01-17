from PyQt5 import QtWidgets
import sys
import time
import numpy as np

import rospy
from std_msgs.msg import Float32MultiArray

from matplotlib.backends.backend_qt5agg import (FigureCanvasQTAgg, NavigationToolbar2QT as NavigationToolbar)
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.figure import Figure
import matplotlib.cm as cm

import teb_test_gui

class MeinWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(MeinWindow, self).__init__()
        self._main = QtWidgets.QWidget()
        self.setCentralWidget(self._main)
        layout = QtWidgets.QHBoxLayout(self._main)

        # dynamic graph widget (the only one that must be encapsuled directly inside the MainWindow)
        layout_graph = QtWidgets.QVBoxLayout(self._main)

        wid_graph = QtWidgets.QWidget()
        # define Canvas Widget
        fig = Figure(figsize=(5, 3))
        dynamic_canvas = FigureCanvasQTAgg(fig)
        self._dynamic_ax = dynamic_canvas.figure.subplots()
        self._line, = self._dynamic_ax.plot([], [], lw=2)
        self._dynamic_ax.grid()
        ani = animation.FuncAnimation(fig, self.__run, interval=10, repeat=False)
        layout_graph.addWidget(dynamic_canvas)
        # NavigationToolbar Widget
        layout_graph.addWidget(NavigationToolbar(dynamic_canvas, self))

        wid_graph.setLayout(layout_graph)

        # imported widgets
        wid1 = teb_test_gui.Ui()
        wid2 = teb_test_gui.Ui_error(5)
        layout.addWidget(wid1)
        layout.addWidget(wid2)
        layout.addWidget(wid_graph)

        self.__time_sub = rospy.Subscriber('/optimizer/time', Float32MultiArray, self.__callback)
        self.__time_list = list()
        self.__opt_time_list = list()
        self.__counter = 0

    def __callback(self, data):
        self.__time_list.append(data.data[1])
        if len(self.__time_list) > 500:
            del self.__time_list[0]
        self.__opt_time_list.append(data.data[0])
        if len(self.__opt_time_list) > 500:
            del self.__opt_time_list[0]

        if self.__counter % 3 == 0:
            self.__run(data)

        self.__counter += 1

    def __run(self, data):
        xmin, xmax = self._dynamic_ax.get_xlim()
        if data.data[1] >= xmax:
            self._dynamic_ax.set_xlim(self.__time_list[0], self.__time_list[-1])
            ymax, ymin = np.array(self.__opt_time_list).max(), np.array(self.__opt_time_list).min()
            self._dynamic_ax.set_ylim(ymin - (ymax - ymin) * 0.05, ymax + (ymax - ymin) * 0.05)
        self._line.set_data(self.__time_list, self.__opt_time_list)
        self._dynamic_ax.figure.canvas.draw()

        return self._line


if __name__ == '__main__':
    rospy.init_node("gui_node")
    qapp = QtWidgets.QApplication.instance()
    if not qapp:
        qapp = QtWidgets.QApplication(sys.argv)
    mein_window = MeinWindow()
    mein_window.activateWindow()
    mein_window.raise_()
    mein_window.show()
    qapp.exec()
