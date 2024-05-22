from enum import Enum

class Dir(Enum):
    UP = "^"
    DOWN = "v"
    LEFT = "<"
    RIGHT = ">"

class State(Enum):    
    WAITING = 0
    MOVING = 1
    STOPPED = 2
    ROTATING = 3
    YET = 4
    DONE = 5
    IDLE = 6

