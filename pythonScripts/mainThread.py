from threading import *
import time
from ArduinoSerial import *
from enum import Enum

import string, sys, time
#from State import state

class MainThread(Thread):
    def __init__(self):
        Thread.__init__(self)

    def run(self):
        # Setup code
        Arduino = ArduinoSerial('/dev/ttyACM0')
        time.sleep(2)
        state = CustomEnumeratorBecausePythonSucks(0)
        
        while 1:
            if state.currentState() == 'WAIT_FOR_START':
                print(state)
                Arduino.setMotorSpeed(0, 'F', 4, 255)
                time.sleep(5)
                state.next()
            elif state.currentState() == 'START':
                print(state)
                Arduino.setMotorSpeed(0, 'F', 4, 0)
                time.sleep(2)
                state.next()
            elif state.currentState() == 'DECODE_LED':
                print(state)
                Arduino.setMotorSpeed(0, 'R', 4, 255)
                time.sleep(5)
                state.next()
            elif state.currentState() == 'TO_STAGE_A':
                print(state)
                Arduino.setMotorSpeed(0, 'F', 4, 0)
                time.sleep(2)
                state.next()
                

                
            #distance = Arduino.analogRead(0)
            #value = Arduino.digitalRead(53)
            
            #if value[0] == 1:
                #Arduino.digitalWrite(22, 1)
            #else:
                #Arduino.digitalWrite(22, 0)
            
            #print('US1: %04d' % distance[0] + '  LS1: %04d' % value[0])
            #time.sleep(0.05)
            #i += 1

# CustomEnumeratorBecausePythonSucks acts as enumerators in other languages do
class CustomEnumeratorBecausePythonSucks:
    def __init__(self, current = 0):
        self.current = current
        self.states = ['WAIT_FOR_START',
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

    # Returns current state when object is printed
    def __str__(self):
        return self.states[self.current]

    # Returns current state of the object
    def currentState(self):
        return self.states[self.current]

    # Sets the current state of the object
    def setCurrentState(self, desiredState):
        self.current = desiredState

    # Increments the current state of the object (with wraparound)
    def next(self):
        self.current = (self.current + 1) % 15

    # Decrements the current state of the object (with wraparound)
    def prev(self):
        self.current = (self.current - 1) % 15
""""
state = CustomEnumeratorBecausePythonSucks(0)

while 1:
    print(state)
    state.next()
    if state.currentState() == 'TO_BOOTY':
        print('Navigating towards the booty.')
    time.sleep(0.5)
"""

"""
class Robot(StateMachine):
    def __init__(self):
        StateMachine.__init__(self, Robot.waitingForStart)


Robot.waitingForStart = WaitingForStart()
Robot.start = Start()
Robot.decodeLED = DecodeLED()
Robot.navigateToStageA = NavigateToStageA()
Robot.stageA = StageA()
Robot.navigateFromStageA = NavigateFromStageA()
Robot.navigateToStageB = NavigateToStageB()
Robot.stageB = StageB()
Robot.navigateToCenter = NavigateToCenter()
Robot.navigateToBooty = NavigateToBooty()
Robot.retrieveBooty = RetrieveBooty()
Robot.navigateToFlag = NavigateToFlag()
Robot.raiseFlag = RaiseFlag()
Robot.navigateToShip = NavigateToShip()
Robot.navigateToStageC = NavigateToStageC()
Robot.stageC = StageC()

Robot().runAll(0.1)
"""
