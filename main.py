from ArduinoSerial import *
import time

while True:
    Arduino = ArduinoSerial()
    time.sleep(1)
    distance = Arduino.analogRead(0)
    print(distance)
    #time.sleep(0.05)
