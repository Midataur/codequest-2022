from abc import ABC
from codequest22.server.ant import Ant, WorkerAnt
import codequest22.stats as stats

class Event(ABC):
    
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

class SpawnEvent(Event):
    def __init__(self, ant: Ant, cost: int, color=None, path=None, goal=None) -> None:
        self.ant_str = ant.to_json()
        self.player_index = ant.player_index
        self.ant_id = ant.id
        self.ant_type = ant.TYPE
        self.position = ant.position[::-1]
        self.hp = ant.hp
        self.cost = cost
        self.color = color
        self.path = path
        self.goal = goal
        if hasattr(ant, "ticks_left"):
            self.ticks_left = ant.ticks_left
        if hasattr(ant, "remaining_trips"):
            self.remaining_trips = ant.remaining_trips

    def get_args(self):
        return [self.ant_str, self.cost, self.color, self.path, self.goal]

class MoveEvent(Event):
    def __init__(self, ant: Ant, path=None) -> None:
        self.ant_str = ant.to_json()
        self.player_index = ant.player_index
        self.ant_id = ant.id
        self.position = ant.position[::-1]
        self.path = path

    def get_args(self):
        return [self.ant_str, self.path]

class DieEvent(Event):

    def __init__(self, ant: Ant) -> None:
        self.ant_str = ant.to_json()
        self.player_index = ant.player_index
        self.ant_id = ant.id
        self.old_age = ant.hp > 0
    
    def get_args(self):
        return [self.ant_str]

class AttackEvent(Event):

    def __init__(self, ant1: Ant, ant2: Ant) -> None:
        self.ant1_str = ant1.to_json()
        self.ant2_str = ant2.to_json()
        self.attacker_index = ant1.player_index
        self.defender_index = ant2.player_index
        self.attacker_id = ant1.id
        self.defender_id = ant2.id
        self.defender_hp = ant2.hp

    def get_args(self):
        return [self.ant1_str, self.ant2_str]

class DepositEvent(Event):

    def __init__(self, ant: WorkerAnt, cur_energy: int) -> None:
        self.ant_str = ant.to_json()
        self.cur_energy = cur_energy
        self.player_index = ant.player_index
        self.ant_id = ant.id
        self.energy_amount = int(ant.encumbered_energy)
        self.total_energy = min(cur_energy + self.energy_amount, stats.general.MAX_ENERGY_STORED)

    def get_args(self):
        return [self.ant_str, self.cur_energy]

class ProductionEvent(Event):

    def __init__(self, ant: WorkerAnt) -> None:
        self.ant_str = ant.to_json()
        self.player_index = ant.player_index
        self.ant_id = ant.id
        self.energy_amount = ant.encumbered_energy
    
    def get_args(self):
        return [self.ant_str]

class ZoneActiveEvent(Event):

    def __init__(self, zone_index: int, num_ticks: int, points: list) -> None:
        self.zone_index = zone_index
        self.points = points
        self.num_ticks = num_ticks

    def get_args(self):
        return [self.zone_index, self.num_ticks, self.points]

class ZoneDeactivateEvent(Event):
    
    def __init__(self, zone_index: int, points: list) -> None:
        self.zone_index = zone_index
        self.points = points

    def get_args(self):
        return [self.zone_index, self.points]

class FoodTileActiveEvent(Event):

    def __init__(self, pos: tuple, num_ticks: int, multiplier: int) -> None:
        self.pos = pos
        self.num_ticks = num_ticks
        self.multiplier = multiplier

    def get_args(self):
        return [self.pos, self.num_ticks, self.multiplier]

class FoodTileDeactivateEvent(Event):
    
    def __init__(self, pos: tuple) -> None:
        self.pos = pos

    def get_args(self):
        return [self.pos]

class SettlerScoreEvent(Event):

    def __init__(self, ant: Ant, score: int) -> None:
        self.ant_str = ant.to_json()
        self.player_index = ant.player_index
        self.ant_id = ant.id
        self.score_amount = score

    def get_args(self):
        return [self.ant_str, self.score_amount]

class QueenAttackEvent(Event):

    def __init__(self, ant: Ant, queen_index: int, queen_hp: int):
        self.ant_str = ant.to_json()
        self.ant_player_index = ant.player_index
        self.ant_id = ant.id
        self.queen_player_index = queen_index
        self.queen_hp = queen_hp

    def get_args(self):
        return [self.ant_str, self.queen_player_index, self.queen_hp]

class TeamDefeatedEvent(Event):

    def __init__(self, defeated_index: int, by_index: int, hill_score: int) -> None:
        self.defeated_index = defeated_index
        self.by_index = by_index
        self.new_hill_score = hill_score

    def get_args(self):
        return [self.defeated_index, self.by_index, self.new_hill_score]
