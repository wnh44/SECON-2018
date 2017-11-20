from PyQt4 import QtCore, QtGui, uic
from debugThread import *
from datetime import datetime

Ui_MainWindow, QtBaseClass = uic.loadUiType('mainwindow.ui')


class MainWindow(QtGui.QMainWindow, Ui_MainWindow):
	def __init__(self):
		QtGui.QMainWindow.__init__(self)
		self.setupUi(self)

	def appinit(self):
		thread = debugListener(self)
		self.connect(thread, thread.signal, self.appendDebugText)
		thread.start()

	def appendDebugText(self, debugMessage):
		time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
		# Attempting to use HTML formatting, not sure if it works yet
		row = '<tr><td>' + debugMessage + '</td><td>' + time + '</td></tr>'
		self.debugText.append(row)
