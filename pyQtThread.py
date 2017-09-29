from threading import *
import sys
from PyQt4 import QtGui
import time

class pyQtThread(Thread):
    def __init__(self, val = 10):
        Thread.__init__(self)
        self.val = val

    def run(self):
       app = QtGui.QApplication(sys.argv)

        w = QtGui.QWidget()
        w.resize(250, 150)
        w.move(300, 300)
        w.setWindowTitle('Simple')
        w.show()
        
        sys.exit(app.exec_())
    
     
