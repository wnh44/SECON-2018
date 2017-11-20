"""
filename: courseObjectives.py
Description: This file contains functions and variables associated with
             course objectives.
"""

import random   # for now


"""
    decodeLED()
    - decodes the PWM IR LED located at the start of the course
    - sets global location parameters
"""
def decodeLED(mainThread):
    # FIXME: Implement IR LED

    # Random selection for now
    for item in mainThread.locations:
        item = random.randint(0,1)

    print('\nRandomized locations:')
    print(mainThread.locations)

    return


"""
    activateStageA(location)
    - activates stage A
"""
def activateStageA(mainThread):
    # Might not be necessary
    return


"""
    activateStageB(location)
    - activates stage B
"""
def activateStageB(mainThread):
    return


"""
    retrieveChest()
    - retrieves and secures the treasure chest
"""
def retrieveChest():
    # Do some stuff
    return


"""
    raiseFlag()
    - raises the flag
"""
def raiseFlag():
    # Do some other stuff
    return


"""
    ActivateStageC(location)
    - activates stage C
    - location parameter is the received location of stage C
"""
def activateStageC(mainThread):
    # Might not be necessary
    return
