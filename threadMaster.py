import threading
from mainThread import *
from pyQtThread import *

#printLock = threading.Lock()

if __name__ == '__main__': 
    mainThread = mainThread()
    mainThread.setName('mainThread')
    mainThread.daemon = True
    
    pyQtThread = pyQtThread()
    pyQtThread.setName('pyQtThread')
    pyQtThread.daemon = True

    mainThread.start()
    pyQtThread.start()

    mainThread.join()
    pyQtThread.join()

    print('\nMain terminating...')
