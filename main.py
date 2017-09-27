from ArduinoSerial import *
import time

Arduino = ArduinoSerial()
time.sleep(2)

while True:
    distance = Arduino.analogRead(0)
    print('US1: %04d' % distance + '  US1: %04d' % distance)#, end ='')
    time.sleep(0.05)
