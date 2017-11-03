from threading import *
import time
from ArduinoSerial import *

class DebugThread(Thread):
    def __init__(self):
        Thread.__init__(self)
        
        # Setup code
        self.conn = serial.Serial('/dev/ttyUSB0', 115200)
        self.conn.timeout = 1
        time.sleep(1)
        
        # Kills loop in run()
        self.end = False

    def run(self):
        while self.end == False:
            self.output = self.conn.readline().decode().strip()
            if len(self.output) > 0:
                print("debug: %s" % self.output)
                
    def stop(self):
        self.end = True
