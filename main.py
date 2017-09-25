from ArduinoSerial import *
import time

Arduino = ArduinoSerial()
time.sleep(2)

while True:
    distance = Arduino.analogRead(0)
    print(distance)
    time.sleep(0.05)
