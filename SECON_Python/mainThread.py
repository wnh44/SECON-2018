from threading import *
from ArduinoSerial import *
from StateEnum import StateEnum

from navigation import *
from courseObjectives import *

import time
import RPi.GPIO as GPIO
START_BUTTON = 23


class MainThread(Thread):
    def __init__(self):
        Thread.__init__(self)

        # Initialize serial connection
        self.Arduino = ArduinoSerial('/dev/ttyACM0', 19200)
        
        # self.locations contains the locations for stages A, B, and C
        self.locations = [0, 0, 0]

        # Set up GPIO in order to use interrupt for start button
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(START_BUTTON, GPIO.IN, pull_up_down = GPIO.PUD_UP)
        
        # Setup for state machine
        states = ['WAIT_FOR_START',
                  'DECODE_LED',
                  'TO_STAGE_A',
                  'STAGE_A',
                  'FROM_STAGE_A',
                  'TO_STAGE_B',
                  'STAGE_B',
                  'TO_BOOTY',
                  'RETRIEVE_BOOTY',
                  'TO_FLAG',
                  'FLAG',
                  'TO_SHIP',
                  'TO_STAGE_C',
                  'STAGE_C']
        self.state = StateEnum(states)  
        time.sleep(1)

    def run(self):
        # FIXME: Theory, theory, theory...

        while 1:
            print(self.state)

            # Waits for start, using GPIO on RasPi as an interrupt
            if self.state.currentState() == 'WAIT_FOR_START':
                print('Awaiting start button...\n')
                #GPIO.wait_for_edge(START_BUTTON, GPIO.FALLING)
                time.sleep(5)
                print('Start button pressed.')
                self.state.setCurrentState('TO_STAGE_A')
                #self.state.next()

            # Decodes IR LED on starting square. Decoding function is in arduino
            # sketch because timing is crucial.
            elif self.state.currentState() == 'DECODE_LED':
                decodeLED(self)
                self.state.next()

            # Navigate to Stage A
            elif self.state.currentState() == 'TO_STAGE_A':
                toStageA(self)
                self.state.setCurrentState('WAIT_FOR_START')
                #self.state.next()

            # Activate Stage A
            elif self.state.currentState() == 'STAGE_A':
                activateStageA(self)
                self.state.next()

            # Navigates from Stage A back to the center of the ship
            elif self.state.currentState() == 'FROM_STAGE_A':
                toCenterOfShip(self)
                self.state.next()

            # Navigate to Stage B
            elif self.state.currentState() == 'TO_STAGE_B':
                goDownRamp()
                toStageB(self)
                self.state.next()

            # Activate Stage B
            elif self.state.currentState() == 'STAGE_B':
                activateStageB(self)
                self.state.next()

            # Navigate to the treasure chest
            elif self.state.currentState() == 'TO_BOOTY':
                toTreasureChest(self)
                self.state.next()

            # Retrieve and secure the treasure chest
            elif self.state.currentState() == 'RETRIEVE_BOOTY':
                retrieveChest()
                toCenterOfField()
                self.state.next()

            # Navigate to the flag
            elif self.state.currentState() == 'TO_FLAG':
                toFlag()
                self.state.next()

            # Raise the flag
            elif self.state.currentState() == 'FLAG':
                raiseFlag()
                self.state.next()

            # Navigate back to the ship
            elif self.state.currentState() == 'TO_SHIP':
                toCenterOfField()
                goUpRamp()
                self.state.next()

            # Navigate to Stage C
            elif self.state.currentState() == 'TO_STAGE_C':
                toStageC(self)
                self.state.next()

            # Activate Stage C
            elif self.state.currentState() == 'STAGE_C':
                activateStageC(self)
                self.state.next()
