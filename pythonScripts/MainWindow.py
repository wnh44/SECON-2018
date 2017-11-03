import sys
from PyQt4 import QtCore, QtGui, uic
from threading import *
from debugThread import *
from time import gmtime, strftime

Ui_MainWindow, QtBaseClass = uic.loadUiType('mainwindow.ui')
#testSignal = QtCore.pyqtSignal()

class MainWindow(QtGui.QMainWindow, Ui_MainWindow):
    def __init__(self):#, debugThread):
        QtGui.QMainWindow.__init__(self)
        #super(DebugThread, self).__init__()
        self.setupUi(self)
        
    def appinit(self):
        thread = debugListener(self)
        self.connect(thread, thread.signal, self.debugFunction)
        thread.start()
        
    def debugFunction(self, debugMessage):
        self.debugText.append(debugMessage)
    
        #self.debugThread = debugThread
        #self.debugThread.testSignal.connect(self.appendDebugText)
        
    #def appendDebugText(self, message):
        #self.debugText.append(message)
        