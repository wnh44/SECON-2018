# StateEnum acts as an enumerator for a FSM
class StateEnum:
    def __init__(self, states = [], current = 0):
        self.current = current
        self.states = states

    # Returns current state when object is printed
    def __str__(self):
        return self.states[self.current]

    # Adds additional state(s) to the object
    def addState(self, states):
        if type(states) is str:
            self.states.append(states)
        elif type(states) is list:
            self.states.extend(states)
        return

    # Returns current state of the object
    def currentState(self):
        return self.states[self.current]

    # Sets the current state of the object
    def setCurrentState(self, desiredState):
        self.current = desiredState
        return

    # Increments the current state of the object (with wraparound)
    def next(self):
        self.current = (self.current + 1) % len(self.states)
        return

    # Decrements the current state of the object (with wraparound)
    def prev(self):
        self.current = (self.current - 1) % len(self.states)
        return
