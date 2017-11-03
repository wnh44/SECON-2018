from threading import *
import sys
from PyQt4 import QtGui
import time

class PyQtThread(Thread):
    def __init__(self, val = 10):
        Thread.__init__(self)
        self.val = val

    def run(self):
        app = QtGui.QApplication(sys.argv)

        w = QtGui.QWidget()
        w.resize(250, 150)
        w.move(100, 100)
        w.setWindowTitle('Lame GUI')
        w.show()

        sys.exit(app.exec_())
    
     
