from threading import *
import time


class MainThread(Thread):
    def __init__(self, val = 10):
        Thread.__init__(self)
        self.val = val

    def run(self):
        for i in range(1, self.val):
            print(self.getName(), ': %d' % (i))
            time.sleep(0.25)
