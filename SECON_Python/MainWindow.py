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
			if item[1] == '0':
				if value == '1':
					self.microswitch0.setStyleSheet("background-color: red; border: 1px solid black;")
				else:
					self.microswitch0.setStyleSheet("background-color: #CCCCCC; border: 1px solid black;")
			if item[1] == '1':
				if value == '1':
					self.microswitch1.setStyleSheet("background-color: red; border: 1px solid black;")
				else:
					self.microswitch1.setStyleSheet("background-color: #CCCCCC; border: 1px solid black;")
			if item[1] == '2':
				if value == '1':
					self.microswitch2.setStyleSheet("background-color: red; border: 1px solid black;")
				else:
					self.microswitch2.setStyleSheet("background-color: #CCCCCC; border: 1px solid black;")
			if item[1] == '3':
				if value == '1':
					self.microswitch3.setStyleSheet("background-color: red; border: 1px solid black;")
				else:
					self.microswitch3.setStyleSheet("background-color: #CCCCCC; border: 1px solid black;")

		elif item[0] == 'R':
			if item[1] == '0':
				self.rangefinder0_value.clear()
				self.rangefinder0_value.appendPlainText(value)
			if item[1] == '1':
				self.rangefinder1_value.clear()
				self.rangefinder1_value.appendPlainText(value)
			if item[1] == '2':
				self.rangefinder2_value.clear()
				self.rangefinder2_value.appendPlainText(value)
			if item[1] == '3':
				self.rangefinder3_value.clear()
				self.rangefinder3_value.appendPlainText(value)
			if item[1] == '4':
				self.rangefinder4_value.clear()
				self.rangefinder4_value.appendPlainText(value)

		elif item[0] == 'D':
			self.debugText.append(value)

		else:
			time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
			# Attempting to use HTML formatting, not sure if it works yet
			row = '<tr><td>' + message + '</td><td>' + time + '</td></tr>'
			self.debugText.append(row)
