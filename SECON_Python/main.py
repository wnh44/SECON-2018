import threading
import atexit

from mainThread import *
from pyQtThread import *

import atexit
    
if __name__ == '__main__': 
    mainThread = MainThread()
    mainThread.setName('mainThread')
    mainThread.daemon = True
    
    pyQtThread = PyQtThread()
    pyQtThread.setName('pyQtThread')
    pyQtThread.daemon = True

    mainThread.start()
    pyQtThread.start()

    mainThread.join()
    pyQtThread.join()

    print('\nMain terminating...')

def exit_handler():
    print('My application is ending!')
