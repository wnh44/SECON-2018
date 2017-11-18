from threading import *
import time
from StateEnum import StateEnum

import string, sys, time

states = ['WAIT_FOR_START',
          'START',
          'DECODE_LED',
          'TO_STAGE_A',
          'STAGE_A',
          'FROM_STAGE_A',
          'TO_STAGE_B',
          'STAGE_B',
          'TO_CENTER',
          'TO_BOOTY',
          'RETRIEVE_BOOTY',
          'TO_FLAG',
          'TO_SHIP',
          'TO_STAGE_C',
          'STAGE_C']
state = StateEnum(states)

while 1:
    print(state)
    state.next()
    if state.currentState() == 'TO_BOOTY':
        print('Navigating towards the booty.')
    time.sleep(0.5)

