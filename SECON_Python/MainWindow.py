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
		self.connect(thread, thread.signal, self.receiveMessage)
		thread.start()

    # Receives message from arduino serial connection.
	# U0-U3: Microswitches
	# R0-R4: Rangefinders
	# D: Debug text
	def receiveMessage(self, message):
		item, value = message.split(':')

		if item[0] == 'U':
			if item[1] == 0:
				#self.microswitch0value.set(value)
			if item[1] == 1:
				#self.microswitch1value.set(value)
			if item[1] == 2:
				#self.microswitch2value.set(value)
			if item[1] == 3:
				#self.microswitch3value.set(value)

		elif item[0] == 'R':
			if item[1] == 0:
				#self.rangefinder0value.set(value)
			if item[1] == 1:
				#self.rangefinder1value.set(value)
			if item[1] == 2:
				#self.rangefinder2value.set(value)
			if item[1] == 3:
				#self.rangefinder3value.set(value)
			if item[1] == 4:
				#self.rangefinder4value.set(value)

		else:
			time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
			# Attempting to use HTML formatting, not sure if it works yet
			row = '<tr><td>' + debugMessage + '</td><td>' + time + '</td></tr>'
			self.debugText.append(row)
