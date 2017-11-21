from mainThread import *
from pyQtThread import *

    
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
