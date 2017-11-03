import sys, time, serial
from PyQt4 import QtCore, QtGui

class debugListener(QtCore.QThread):
    def __init__(self, parent):
        # Setup Code
        QtCore.QThread.__init__(self, parent)
        self.signal = QtCore.SIGNAL("test Signal")
        
        self.conn = serial.Serial('/dev/ttyUSB0', 115200)
        self.conn.timeout = 1
        self.end = False
        time.sleep(1)
        
    def run(self):
        while self.end == False:
            self.output = self.conn.readline().decode().strip()
            if len(self.output) > 0:
                self.emit(self.signal, self.output)
        self.conn.close()




"""from threading import *
import time

from ArduinoSerial import *

from MainWindow import *
from PyQt4 import QtCore

class DebugThread(Thread, QtCore.QObject):
    
    #testSignal = QtCore.pyqtSignal(str)
    
    def __init__(self):
        Thread.__init__(self)
        #super(DebugThread, self).__init__()
        #pass
        
        # Setup code
        self.conn = serial.Serial('/dev/ttyUSB0', 115200)
        self.conn.timeout = 1
        time.sleep(1)
        
        # Kills loop in run() ...or it's supposed to
        self.end = False

    def run(self):
        while self.end == False:
            self.output = self.conn.readline().decode().strip()
            if len(self.output) > 0:
                self.testSignal.emit(output)
                
    def stop(self):
        self.end = True"""
