from ArduinoSerial import *
import time

while True:
    Arduino = ArduinoSerial()
    distance = Arduino.analogRead(0)
    print(distance)
    time.sleep(0.05)
