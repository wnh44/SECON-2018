"""
Filename: StateEnum.py
Description: StateEnum is an enumerator class useful for state machines, I
             wrote my own class for it because python's enumerator is missing
             some pretty important functionalities
"""


class StateEnum:
    def __init__(self, states = [], current = 0):
        self.current = current
        self.states = states


    """
    __str__():
    - returns the string of the current state when the object is used in a 
      print statement
    """

    def __str__(self):
        return self.states[self.current]


    """
    addState(states):
    - adds states to self.states
    - works if passed string or an array of strings
    """

    def addState(self, states):
        if type(states) is str:
            self.states.append(states)
        elif type(states) is list:
            self.states.extend(states)
        return


    """
    currentState():
    - returns the string of the current state
    """
    def currentState(self):
        return self.states[self.current]


    """
    setCurrentState(desiredState)
    - sets the current state of the object
    - accepts an index or a string containing the desired state name
    """

    def setCurrentState(self, desiredState):
        if type(desiredState) is int:
            try:
                self.current = desiredState
            except:
                print("Out of range.")
                pass
        elif type(desiredState) is str:
            try:
                self.current = self.states.index(desiredState)
            except:
                print("Not a valid state.")
                pass
        return


    """
    next():
    - iterates to the next state and jumps to the beginning when the 
      end is reached
    """

    def next(self):
        self.current = (self.current + 1) % len(self.states)
        return


    """
    prev():
    - iterates to the previous state and jumps to the end when the beginning
    is reached
    """

    def prev(self):
        self.current = (self.current - 1) % len(self.states)
        return
