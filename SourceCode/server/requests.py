from abc import ABC
from codequest22.server.ant import AntTypes

class Request(ABC):
    
    def __repr__(self) -> str:
        return f"{{\"{self.__class__.__name__}\": {self.get_args()}}}"
    
    def to_json(self):
        return {
            "classname": self.__class__.__name__,
            "args": self.get_args(),
        }
    
    @classmethod
    def from_json(cls, data):
        return cls(*data["args"])

class SpawnRequest(Request):

    def __init__(self, ant_type: AntTypes, id=None, color=None, goal=None) -> None:
        super().__init__()
        self.ant_type = ant_type
        self.id = id
        self.color = color
        self.goal = None
        if goal is not None:
            self.goal = goal[1], goal[0]
        self.cost = AntTypes.get_stats(self.ant_type).COST
    
    def get_args(self):
        return [self.ant_type, self.id, self.color, self.goal]

class GoalRequest(Request):
    
    def __init__(self, id, position) -> None:
        super().__init__()
        self.ant_id = id
        self.position = position

    def get_args(self):
        return [self.ant_id, self.position]

