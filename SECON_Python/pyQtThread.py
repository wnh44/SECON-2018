from threading import *
from PyQt4 import QtGui, QtCore, uic
from MainWindow import *


class PyQtThread(Thread):
	def __init__(self):
		Thread.__init__(self)

	def run(self):
		print('here')
		app = QtGui.QApplication(sys.argv)

		mainWindow = MainWindow()

		mainWindow.show()
		QtCore.QTimer.singleShot(0, mainWindow.appinit)
		sys.exit(app.exec_())
