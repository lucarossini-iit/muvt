from PyQt5 import QtWidgets
import sys
import rospy

import teb_test_gui

class MeinWindow(QtWidgets.QMainWindow):
    def __init__(self):
        wid = QtWidgets.QWidget()
        super(MeinWindow, self).__init__()
        layout = QtWidgets.QHBoxLayout()
        wid1 = teb_test_gui.Ui()
        wid2 = teb_test_gui.Ui_error(5)
        layout.addWidget(wid1)
        layout.addWidget(wid2)
        wid.setLayout(layout)
        self.setCentralWidget(wid)

if __name__ == '__main__':
    rospy.init_node("gui_node")
    app = QtWidgets.QApplication(sys.argv)
    mein_window = MeinWindow()
    mein_window.show()
    app.exec_()
