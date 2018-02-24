from mainThread import *
from pyQtThread import *


if __name__ == '__main__':
    pyQtThread = PyQtThread()
    pyQtThread.setName('pyQtThread')
    pyQtThread.daemon = True

    time.sleep(2)
    pyQtThread.start()

    pyQtThread.join()

    print('\nMain terminating...')
