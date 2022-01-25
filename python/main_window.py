from PyQt5 import QtWidgets
import sys
import time
import numpy as np

import rospy
from std_msgs.msg import Float32MultiArray, Float32

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
        layout_graph = QtWidgets.QVBoxLayout()

        # plot animated graph for computation time
        wid_graph = QtWidgets.QWidget()
        fig = Figure(figsize=(5, 3))
        dynamic_canvas = FigureCanvasQTAgg(fig)
        self._dynamic_ax = dynamic_canvas.figure.subplots()
        self._line, = self._dynamic_ax.plot([], [], lw=2)
        self._dynamic_ax.grid()
        ani = animation.FuncAnimation(fig, self.__run, interval=10, repeat=False)
        layout_graph.addWidget(dynamic_canvas)
        layout_graph.addWidget(NavigationToolbar(dynamic_canvas, self))
        # wid_graph.setLayout(layout_graph)

        # plot animated graph for residual error
        wid_err = QtWidgets.QWidget()
        layout_err = QtWidgets.QVBoxLayout()
        fig_err = Figure(figsize=(5, 3))
        dynamic_err = FigureCanvasQTAgg(fig_err)
        self._dynamic_ax_err = dynamic_err.figure.subplots()
        self._line_err, = self._dynamic_ax_err.plot([], [], lw=2, label='EdgeCollision')
        self._line_err_vel, = self._dynamic_ax_err.plot([], [], lw=2, label='EdgeRobotVel')
        self._dynamic_ax_err.legend()
        self._dynamic_ax_err.grid()
        ani = animation.FuncAnimation(fig, self.__run_err, interval=10, repeat=False)
        layout_graph.addWidget(dynamic_err)
        layout_graph.addWidget(NavigationToolbar(dynamic_err, self))
        wid_graph.setLayout(layout_graph)

        # imported widgets
        wid1 = teb_test_gui.Ui()
        wid2 = teb_test_gui.Ui_error(5)
        vision_times = teb_test_gui.VisionTime()
        layout.addWidget(wid1)
        layout.addWidget(wid2)
        layout.addWidget(vision_times)
        layout.addWidget(wid_graph)
        layout.addWidget(wid_err)

        self.__time_sub = rospy.Subscriber('/optimizer/time', Float32MultiArray, self.__callback)
        self.__err_sub = rospy.Subscriber('/optimizer/error', Float32MultiArray, self.__callback_error)
        self.__time_list = list()
        self.__opt_time_list = list()
        self.__error_coll = list()
        self.__error_vel = list()
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

    def __callback_error(self, data):
        self.__error_coll.append(data.data[0])
        if len(self.__error_coll) > 500:
            del self.__error_coll[0]

        self.__error_vel.append(data.data[1])
        if len(self.__error_vel) > 500:
            del self.__error_vel[0]

        if self.__counter % 3 == 0:
            self.__run_err()

    def __run(self, data):
        xmin, xmax = self._dynamic_ax.get_xlim()
        if data.data[1] >= xmax:
            self._dynamic_ax.set_xlim(self.__time_list[0], self.__time_list[-1])
            ymax, ymin = np.array(self.__opt_time_list).max(), np.array(self.__opt_time_list).min()
            self._dynamic_ax.set_ylim(ymin - (ymax - ymin) * 0.05, ymax + (ymax - ymin) * 0.05)
        self._line.set_data(self.__time_list, self.__opt_time_list)
        self._dynamic_ax.figure.canvas.draw()

        return self._line

    def __run_err(self):
        xmin, xmax = self._dynamic_ax_err.get_xlim()
        if self.__time_list[-1] >= xmax:
            self._dynamic_ax_err.set_xlim(self.__time_list[0], self.__time_list[-1])
        if np.array(self.__error_coll).max() < np.array(self.__error_vel).max():
            ymax = np.array(self.__error_vel).max()
        else:
            ymax = np.array(self.__error_coll).max()
        if np.array(self.__error_coll).min() < np.array(self.__error_vel).min():
            ymin = np.array(self.__error_coll).min()
        else:
            ymin = np.array(self.__error_vel).min()
        self._dynamic_ax_err.set_ylim(ymin - (ymax - ymin) * 0.05, ymax + (ymax - ymin) * 0.05)
        self._line_err.set_data(self.__time_list[:len(self.__error_coll)], self.__error_coll)
        self._line_err_vel.set_data(self.__time_list[:len(self.__error_vel)], self.__error_vel)
        self._dynamic_ax_err.figure.canvas.draw()

        return [self._line_err, self._line_err_vel]

if __name__ == '__main__':
    rospy.init_node("gui_node")
    qapp = QtWidgets.QApplication.instance()
    if not qapp:
        qapp = QtWidgets.QApplication(sys.argv)
    mein_window = MeinWindow()
    mein_window.activateWindow()
    mein_window.raise_()
    mein_window.show()
    qapp.exec_()
