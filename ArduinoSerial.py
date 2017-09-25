"""
Filename: ArduinoSerial.py
Description: ArduinoSerial is a class that handles serial communication between
    the raspberry pi and the arduino.
"""

import serial

class ArduinoSerial():
    def __init__(self, serialPort = '/dev/ttyACM0', baud_rate=9600, read_timeout=5):
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
    digitalRead(pinNumber)
    - reads a value (readValue) from a digital pin (pinNumber)
    - message sent across serial connection resembles the following examples:
        RD3
        RD12
    """
    def digitalRead(self, pinNumber):
        message = ''.join(('R', 'D', str(pinNumber)))
        self.conn.write(message.encode())
        readValue = self.conn.readline().decode().strip()
        return readValue
    
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
    analogRead(pinNumber)
    - reads a value (readValue) from an analog pin (pinNumber)
    - D(readValue) = [0,1023]
    - message sent across serial connection resembles the following examples:
        RA5
        RA0
    """
    def analogRead(self, pinNumber):
        message = ''.join(('R', 'A', str(pinNumber)))
        self.conn.write(message.encode())
        readValue = int(self.conn.readline().decode().strip(), 16)
        return readValue
    
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
    close()
    - closes serial connection with the arduino
    """
    def close(self):
        self.conn.close()
        print('Arduino serial connection closed.\n.')
        
