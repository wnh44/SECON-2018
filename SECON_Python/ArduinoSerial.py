"""
Filename: ArduinoSerial.py
Description: ArduinoSerial is a class that handles serial communication between
             the raspberry pi and the arduino.


Serial data is sent in the following manner:

    {operation}{mode}{index}{':'}{quantity}{':'}{value}{'/n'}

	{operation}
		M: Set motor speed
		P: Set pin mode
		R: Read value
		W: Write value

	{mode}
		A: Analog
		D: Digital
		F: FORWARD
		I: INPUT
		O: OUTPUT
		P: INPUT_PULLUP
		R: BACKWARD

	{index}
		If M: Motor number to set.
		If P: Pin number to set mode of.
		If R: Pin number to read from.
		If W: Pin number to write to.

	{quantity}
		If M: Number of (sequential) motors to set.
		If P: N/A
		If R: Number of (sequential) inputs to read.
		If W: N/A

	{value}
		If M: Motor speed to set.
		If P: N/A
		If R: N/A
		If W: Value to write.

	Examples:
		Set Pin Mode:     PI4
		Digital Read:     DR7:3
		Digital Write:    DW4:0
		Analog Read:      AR4:0
		Analog Write:     AW0:759
		Set Motor Speed:  MF0:4:255
		Set Motor Speed:  MR2:2:60

"""

import serial


class ArduinoSerial():
    def __init__(self, serialPort = '/dev/ttyACM0', baud_rate = 9600, read_timeout = 5):
        self.conn = serial.Serial(serialPort, baud_rate)
        self.conn.timeout = read_timeout

    """
	setPinMode(pinNumber, mode)
	- sets the pin mode (mode) for a digital pin (pinNumber)
	- I=INPUT, O=OUTPUT, P=INPUT_PULLUP
	- message sent across serial connection resembles the following examples:
		MI2
		MO8
		MP12
    """

    def setPinMode(self, pinNumber, mode):
        message = ''.join(('M', mode, str(pinNumber)))
        self.conn.write(message.encode())

    """
	digitalRead(pinNumber, quantity)
	- reads a number (quantity) of sequential digital values (readValues) starting 
	  at pin (pinNumber)
	- D(readValue) = [0, 1]
	- message sent across serial connection resembles the following examples:
		RD3
		RD12
    """

    def digitalRead(self, pinNumber, quantity = 1):  
        message = ''.join(('R', 'D', str(pinNumber), ':', str(quantity)))
        print(message)
        self.conn.write(message.encode())
        readValues = self.conn.readline().decode().strip().split(':')
        readValues = list(map(int, readValues))
        print(readValues)
        return readValues
    
    
    """
	digitalWrite(pinNumber, writeValue)
	- writes a value (writeValue) to a digital pin (pinNumber)
	- message sent across serial connection resembles the following examples:
		WD4:1
		WD11:0
    """

    def digitalWrite(self, pinNumber, writeValue):
        message = ''.join(('W', 'D', str(pinNumber), ':', str(writeValue)))
        self.conn.write(message.encode())

    """
	analogRead(pinNumber, quantity)
	- reads a number (quantity) of sequential analog values (readValues) starting 
	  at pin (pinNumber)
	- D(readValue) = [0,1023]
	- message sent across serial connection resembles the following examples:
		RA5:1
		RA0:5
    """

    def analogRead(self, pinNumber, quantity = 1):
        message = (''.join(('R', 'A', str(pinNumber), ':', str(quantity))))
        self.conn.write(message.encode())
        readValues = self.conn.readline().decode().strip().split(':')
        readValues = list(map(int, readValues))

        return readValues

    """
	analogWrite(pinNumber, writeValue)
	- writes a value (writeValue) to an analog pin (pinNumber)
	- D(writeValue) = [0,1023]
	- message sent across serial connection resembles the following examples:
		WA2:256
		WA5:1020
    """

    def analogWrite(self, pinNumber, writeValue):
        message = ''.join(('W', 'A', str(pinNumber), ':', str(writeValue)))
        self.conn.write(message.encode())

    """
	setMotorSpeed(motorNumber, mode, quantity, setSpeed)
	- sets a number (quantity) of sequential motors speeds (setSpeed) starting 
	  at motorNumber in the direction (mode) specified
	- speed = [0,255]
	- message sent across serial connection resembles the following examples:
		MF0:4:255
		MB2:2:60
    """

    def setMotorSpeed(self, motorNumber, mode, quantity = 0, setSpeed = 255):
        message = (''.join(('M', mode, str(motorNumber), ':', str(quantity), ':', str(setSpeed))))
        self.conn.write(message.encode())

    """
	callArduinoFunction(function)
	- calls a function defined in the Arduino code
	- message sent across serial connection resembles the following examples:
	    FL0  <-- Receives and decodes PWM IR LED
		FC0  <-- Sets all motors to their assigned motor velocities
    """

    def callArduinoFunction(self, function):
        message = (''.join(('F', function, str(0))))
        self.conn.write(message.encode())

    """
	close()
	- closes serial connection with the arduino
    """

    def close(self):
        self.conn.close()
        print('Arduino serial connection closed.\n.')
