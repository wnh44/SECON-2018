from threading import Thread
import time
from testThread import *
from testThread1 import *
from ArduinoSerial import *

Arduino = ArduinoSerial()
time.sleep(2)
#Arduino.setPinMode('A', 0, 'I')

while True:
    distance = Arduino.analogRead(0)
    value = Arduino.digitalRead(2)
    print('US1: %04d' % distance + '  LS1: %04d' % value)#, end ='')
    time.sleep(0.05)