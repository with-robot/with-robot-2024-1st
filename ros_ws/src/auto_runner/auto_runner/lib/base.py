import math
import time
from typing import TypeVar, Sequence
from enum import Enum
from auto_runner.lib.common import *

state: State = State.ROTATE_STOP


class BaseLayer:
    '''발송, 후진, 맵'''
    def __init__(self, node: MessageHandler, state_data: StateData):
        self.node = node
        self.state_data = state_data
