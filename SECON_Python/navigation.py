"""
Filename: navigation.py
Description: This file contains functions and variables associated with
             navigation of the course.
"""

import time

# FIXME: Pins need to be changed to correct numbers

"""
    toStageA(location)
    - navigates to stage A (theoretically)
"""
def toStageA(mainThread):
    microswitches = mainThread.Arduino.digitalRead(4, 2)
    againstBackWall = microswitches[0] & microswitches[1]  # I think '&' is right

    # Move robot to back wall
    moveRobotBackward(mainThread, 255)
    while againstBackWall == 0:
        time.sleep(0.1)
        print(microswitches)
        microswitches = mainThread.Arduino.digitalRead(4, 2)
        againstBackWall = microswitches[0] & microswitches[1]  # I think '&' is right
    #stopRobot(mainThread)

    """# If Stage A location is 0, move left until reached
    if mainThread.locations[0] == 0:
        moveRobotLeft(mainThread, 255)
        oppositeStageA = mainThread.Arduino.analogRead(1, 1)
        while oppositeStageA[0] >= 37:
			oppositeStageA = mainThread.Arduino.analogRead(1, 1)

	# If Stage A location is 1, move right until reached
    else:
        moveRobotRight(mainThread, 255)
        oppositeStageA = mainThread.Arduino.analogRead(0, 1)
        while oppositeStageA[0] >= 37:
			oppositeStageA = mainThread.Arduino.analogRead(0, 1)"""
    stopRobot(mainThread)

    return


"""
    toCenterOfShip()
    - navigates to center of the ship
"""
def toCenterOfShip(mainThread):
	# sides[0] -> Left side
	# sides[1] -> Right side
	sides = mainThread.Arduino.analogRead(0, 2)

	# If Stage A location is 0, move right until center is reached
	if mainThread.locations[0] == 0:
		moveRobotRight(mainThread, 255)
		while(sides[0] < sides[1]):
			sides = mainThread.Arudino.analogRead(0, 2)
		
	# If Stage A location is 1, move left until center is reached
	else:
		moveRobotLeft(mainThread, 255)
		while(sides[0] < sides[1]):
			sides = mainThread.Arudino.analogRead(0, 2)
			
	stopRobot(mainThread)
	
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
    - This function just states the blatantly obvious
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

# FIXME: THE FOLLOWING TWO FUNCTIONS MIGHT BE INVERTED
"""
    moveRobotLeft(mainThread, speed)
    - commands robot left at designated speed
"""
def moveRobotLeft(mainThread, speed = 255):
    mainThread.Arduino.setMotorSpeed(0, 'B', 1, speed)
    mainThread.Arduino.setMotorSpeed(1, 'F', 1, speed)
    mainThread.Arduino.setMotorSpeed(2, 'B', 1, speed)
    mainThread.Arduino.setMotorSpeed(3, 'F', 1, speed)
    mainThread.Arduino.callArduinoFunction('C')
    return


"""
    moveRobotRight(mainThread, speed)
    - commands robot right at designated speed
"""
def moveRobotRight(mainThread, speed = 255):
    mainThread.Arduino.setMotorSpeed(0, 'F', 1, speed)
    mainThread.Arduino.setMotorSpeed(1, 'B', 1, speed)
    mainThread.Arduino.setMotorSpeed(2, 'F', 1, speed)
    mainThread.Arduino.setMotorSpeed(3, 'B', 1, speed)
    mainThread.Arduino.callArduinoFunction('C')
    return


"""
    turnRobotLeft(mainThread, speed)
    - commands robot to turn left at designated speed
"""
def turnRobotLeft(mainThread, speed = 127):
	mainThread.Arduino.setMotorSpeed(1, 'F', 2, speed)
	mainThread.Arduino.callArduinoFunction('C')
	
	# Might be a better method
	#mainThread.Arduino.setMotorSpeed(0, 'R', 1, speed)
	#mainThread.Arduino.setMotorSpeed(1, 'F', 1, speed)
	#mainThread.Arduino.callArduinoFunction('C')
	
	# Probably too fast
	#mainThread.Arduino.setMotorSpeed(4, 'R', 2, speed)
	#mainThread.Arduino.setMotorSpeed(1, 'F', 2, speed)
	#mainThread.Arduino.callArduinoFunction('C')
	return


"""
    turnRobotRight(mainThread, speed)
    - commands robot to turn right at designated speed
"""
def turnRobotRight(mainThread, speed = 127):
    mainThread.Arduino.setMotorSpeed(4, 'F', 2, speed)
    mainThread.Arduino.callArduinoFunction('C')
	
	# Might be a better method
    #mainThread.Arduino.setMotorSpeed(0, 'F', 1, speed)
    #mainThread.Arduino.setMotorSpeed(1, 'R', 1, speed)
    #mainThread.Arduino.callArduinoFunction('C')
	
	# Probably too fast
    #mainThread.Arduino.setMotorSpeed(4, 'F', 2, speed)
    #mainThread.Arduino.setMotorSpeed(1, 'R', 2, speed)
    #mainThread.Arduino.callArduinoFunction('C')
    return


"""
    stopRobot(mainThread)
    - commands robot to stop
"""
def stopRobot(mainThread):
    mainThread.Arduino.setMotorSpeed(0, 'F', 4, 0)
    mainThread.Arduino.callArduinoFunction('C')
    return
