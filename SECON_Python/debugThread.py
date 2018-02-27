import time, serial
from PyQt4 import QtCore, QtGui


class debugListener(QtCore.QThread):
    def __init__(self, parent):
        QtCore.QThread.__init__(self, parent)
        self.signal = QtCore.SIGNAL("test Signal")

        self.conn = serial.Serial('/dev/ttyUSB0', 57600)
        self.conn.timeout = 1
        self.end = False
        time.sleep(1)

    def run(self):
        while self.end == False:
            self.output = self.conn.readline().decode().strip()
            if len(self.output) > 0:
                self.emit(self.signal, self.output)
        self.conn.close()
