from threading import *
import sys, time, serial
from PyQt4 import QtGui, QtCore, uic
from MainWindow import *
from debugThread import *

class PyQtThread(Thread):
    def __init__(self):#, debugThread):
        Thread.__init__(self)
        #super(DebugThread, self).__init__()
        #self.debugThread = debugThread

    def run(self):
        print('here')
        app = QtGui.QApplication(sys.argv)
        
        mainWindow = MainWindow()#self.debugThread)
        
        mainWindow.show()   
        QtCore.QTimer.singleShot(0, mainWindow.appinit)
        sys.exit(app.exec_())
    
     
