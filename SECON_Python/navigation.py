"""
Filename: navigation.py
Description: This file contains functions and variables associated with
             navigation of the course.
"""

import time


"""
    toStageA(location)
    - navigates to stage A (theoretically)
"""
def toStageA(mainThread):
    againstBackWall = 0
    microswitches = mainThread.Arduino.digitalRead(0, 2)
    againstBackWall = microswitches[0] & microswitches[1]  # I think '&' is right

    while againstBackWall == 0:
        moveRobotForward(mainThread, 255)
        microswitches = mainThread.Arduino.digitalRead(0, 2)
        againstBackWall = microswitches[0] & microswitches[1]  # I think '&' is right

    if mainThread.locations[0] == 0:
        atStageA = 0
        while atStageA == 0:
            otherStageA = mainThread.Arduino.analogRead(1, 1)
            if otherStageA >= 37:
                atStageA = 1
            moveRobotLeft(mainThread, 255)
        stopRobot(mainThread)

    elif mainThread.locations[0] == 1:
        atStageA = 0
        while atStageA == 0:
            otherStageA = mainThread.Arduino.analogRead(0, 1)
            if otherStageA >= 37:
                atStageA = 1
            moveRobotRight(mainThread, 255)
        stopRobot(mainThread)

    return


"""
    toCenterOfShip()
    - navigates to center of the ship
"""
def toCenterOfShip(mainThread):
    return


"""
    goDownRamp()
    - navigates down the ramp onto the main playing field
"""
def goDownRamp():
    return


"""
    toStageB(location)
    - navigates to stage B
"""
def toStageB(mainThread):
    return


"""
    toCenterOfField()
    - navigates to the center of the playing field
"""
def toCenterOfField():
    return


"""
    jacksonIsADouche(everyone, knows, this, already)
    - This function just states the obvious
"""
def jacksonIsADouche(everyone, knows, this, already):
    forever = True
    while forever:
        print('Jackson is a douche.')
        time.sleep(1)
    return True


"""
    toTreasureChest(location)
    - navigates to the treasure chest
"""
def toTreasureChest(mainThread):
    return


"""
    toFlag()
    - navigates to the flag
"""
def toFlag():
    return


"""
    goUpRamp()
    - navigates up the ramp onto the ship
"""
def goUpRamp():
    return


"""
    toStageC(location)
    - navigates to stage C
"""
def toStageC(mainThread):
    return


"""
    moveRobotForward(mainThread, speed)
    - commands robot forward at designated speed
"""
def moveRobotForward(mainThread, speed = 255):
    mainThread.Arduino.setMotorSpeed(0, 'F', 4, speed)
    mainThread.Arduino.callArduinoFunction('C')
    return


"""
    moveRobotBackward(mainThread, speed)
    - commands robot backward at designated speed
"""
def moveRobotBackward(mainThread, speed = 255):
    mainThread.Arduino.setMotorSpeed(0, 'B', 4, speed)
    mainThread.Arduino.callArduinoFunction('C')
    return

# NOTE: THE FOLLOWING TWO FUNCTIONS MIGHT BE INVERTED
"""
    moveRobotLeft(mainThread, speed)
    - commands robot left at designated speed
"""
def moveRobotLeft(mainThread, speed = 255):
    mainThread.Arduino.setMotorSpeed(0, 'F', 1, speed)
    mainThread.Arduino.setMotorSpeed(1, 'B', 1, speed)
    mainThread.Arduino.setMotorSpeed(2, 'F', 1, speed)
    mainThread.Arduino.setMotorSpeed(3, 'B', 1, speed)
    mainThread.Arduino.callArduinoFunction('C')
    return


"""
    moveRobotRight(mainThread, speed)
    - commands robot right at designated speed
"""
def moveRobotRight(mainThread, speed = 255):
    mainThread.Arduino.setMotorSpeed(0, 'B', 1, speed)
    mainThread.Arduino.setMotorSpeed(1, 'F', 1, speed)
    mainThread.Arduino.setMotorSpeed(2, 'B', 1, speed)
    mainThread.Arduino.setMotorSpeed(3, 'F', 1, speed)
    mainThread.Arduino.callArduinoFunction('C')
    return


"""
    stopRobot(mainThread)
    - commands robot to stop
"""
def stopRobot(mainThread):
    mainThread.Arduino.setMotorSpeed(0, 'F', 4, 0)
    mainThread.Arduino.callArduinoFunction('C')
    return
