from threading import *
from ArduinoSerial import *
from StateEnum import StateEnum

from navigation import *
from courseObjectives import *

import RPi.GPIO as GPIO
START_BUTTON = 23

import time

class MainThread(Thread):
    def __init__(self):
        Thread.__init__(self)

        self.locations = [0, 0, 0]
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(START_BUTTON, GPIO.IN, pull_up_down = GPIO.PUD_UP)

    def run(self):
        # Setup code
        self.Arduino = ArduinoSerial('/dev/ttyACM0', 57600)
        time.sleep(2)
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
        self.state = StateEnum(states)


        # NOTE: State machine is theorized for now

        while 1:
            print(self.state)

            if self.state.currentState() == 'WAIT_FOR_START':
                GPIO.wait_for_edge(START_BUTTON, GPIO.FALLING)
                print('Start button pressed.')
                self.state.next()

            elif self.state.currentState() == 'START':
                print('Starting...')
                self.state.next()

            elif self.state.currentState() == 'DECODE_LED':
                decodeLED(self)
                self.state.next()

            elif self.state.currentState() == 'TO_STAGE_A':
                toStageA(self)
                self.state.next()

            elif self.state.currentState() == 'STAGE_A':
                activateStageA(self)
                self.state.next()

            elif self.state.currentState() == 'FROM_STAGE_A':
                self.state.next()

            elif self.state.currentState() == 'TO_STAGE_B':
                self.state.next()

            elif self.state.currentState() == 'STAGE_B':
                self.state.next()

            elif self.state.currentState() == 'TO_CENTER':
                self.state.next()

            elif self.state.currentState() == 'TO_BOOTY':
                self.state.next()

            elif self.state.currentState() == 'RETRIEVE_BOOTY':
                self.state.next()

            elif self.state.currentState() == 'TO_FLAG':
                self.state.next()

            elif self.state.currentState() == 'TO_SHIP':
                self.state.next()

            elif self.state.currentState() == 'TO_STAGE_C':
                self.state.next()

            elif self.state.currentState() == 'STAGE_C':
                self.state.next()
                
            #distance = Arduino.analogRead(0)
            #value = Arduino.digitalRead(53)

            #if value[0] == 1:
                #Arduino.digitalWrite(22, 1)
            #else:
                #Arduino.digitalWrite(22, 0)

            #print('US1: %04d' % distance[0] + '  LS1: %04d' % value[0])
            #time.sleep(0.05)
            #i += 1
