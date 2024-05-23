from enum import Enum
from typing import TypeVar, Sequence
from typing import NamedTuple

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
    STRAIT = 6

# Python에서는 프로토타입 기반의 객체 지향 프로그래밍을 직접적으로 지원하지 않지만, 
# __getattr__ 메서드와 메타클래스를 활용하면 프로토타입 기반의 인터페이스를 구현할 수 있습니다. 다음과 같이 작성할 수 있습니다:
# class InterfaceMeta(type):
#     def __new__(cls, name, bases, attrs):
#         new_attrs = {}
#         for base in bases:
#             for attr_name, attr_value in base.__dict__.items():
#                 if not attr_name.startswith("_"):
#                     new_attrs[attr_name] = attr_value
#         attrs.update(new_attrs)
#         return super().__new__(cls, name, bases, attrs)

# class Interface(metaclass=InterfaceMeta):
    # '''인터페이스정의 클래스'''

class MessageHandler:
    def _logging(self, message):'''로깅을 한다'''
    def _send_message(self, message):'''로봇에 메시지를 전달한다'''


class StateData(NamedTuple):
    new: any
    old: any

    def shift(self, newdata):
        self.old = self.new
        self.new = newdata
