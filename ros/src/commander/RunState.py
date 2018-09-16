import enum

class RunState(enum.Enum):
    IDLE = 0
    SEARCH = 1
    MOVE = 2
    KILL = 3
