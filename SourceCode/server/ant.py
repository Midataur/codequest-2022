import numpy as np
import random
from numpy.linalg import norm
from abc import ABC
from enum import Enum, auto

from codequest22.server.global_map import GlobalMap
from codequest22.stats.ants import Worker, Fighter, Settler

class Ant(ABC):

    def __init__(self, player_index, id, position, energy, color) -> None:
        stats = AntTypes.get_stats(self.TYPE)
        self.color = color
        self.player_index = player_index
        self.id = id
        self.position = position
        self.hp = stats.HP
        self.speed = stats.SPEED
        self.cost = energy
        if hasattr(stats, "LIFESPAN"):
            self.ticks_left = stats.LIFESPAN
        self.goal = None
        self.moved = False
        self.defined_keys = [
            "color",
            "player_index",
            "id",
            "position",
            "hp",
            "cost",
        ]

    def tick(self) -> None:
        self.moved = False
        self.old_position = self.position
        if hasattr(self, "ticks_left"):
            self.ticks_left -= 1
        if not self.alive(): return
        if self.goal is not None:
            togo = self.calculate_speed()
            while self._follow_path:
                new_pos = np.array(list(map(float, self._follow_path[-1])))
                if len(self._follow_path) > 1:
                    new_pos += 0.8*np.array([0.5-random.random(), 0.5-random.random()])
                move_vec = new_pos - np.array(self.position)
                normal = norm(move_vec)
                if normal > togo:
                    self.position = np.array(self.position) + move_vec * (togo / normal)
                    break
                self.position = new_pos
                togo -= normal
                self._follow_path.pop()
                if togo <= 0:
                    break
            # I could be in a bad position atm.
            ipos = (round(self.position[0]), round(self.position[1]))
            if GlobalMap.map_obj[ipos[0]][ipos[1]] == "W":
                # ISSUE: We need to backtrack in some direction from our move_vec.
                options = []
                if move_vec[0] > 0:
                    options.append((1, 0))
                elif move_vec[0] < 0:
                    options.append((-1, 0))
                if move_vec[1] > 0:
                    options.append((0, 1))
                elif move_vec[1] < 0:
                    options.append((0, -1))
                best = None
                for dx, dy in options:
                    if GlobalMap.map_obj[ipos[0]+dx][ipos[1]+dy] != "W":
                        # We can go here.
                        if dx != 0:
                            fixed_pos = (int(self.position[0]+dx) - 0.5 * dx, self.position[1])
                            distance = pow(fixed_pos[0] - new_pos[0], 2) + pow(fixed_pos[1] - new_pos[1], 2)
                            if best is None or best[1] > distance:
                                best = (fixed_pos, distance)
                        if dy != 0:
                            fixed_pos = (self.position[0], int(self.position[1]+dy) - 0.5 * dy)
                            distance = pow(fixed_pos[0] - new_pos[0], 2) + pow(fixed_pos[1] - new_pos[1], 2)
                            if best is None or best[1] > distance:
                                best = (fixed_pos, distance)
                if best is None:
                    print(ipos)
                    print("ERROR: A pathfinding error occured. An ant was placed on a non-traversable tile and couldn't be shifted. Just going to move it to the next follow point.")
                    self.position = self._follow_path[-1] if self._follow_path else self.goal
                else:
                    self.position = fixed_pos
            if not self._follow_path:
                self.goal = None
            self.moved = True

    def alive(self) -> bool:
        return self.hp > 0 and self.ticks_left > 0

    def set_goal(self, g):
        self.goal = (g[0], g[1])
        pos = (self.position[0], self.position[1])
        self._follow_path = GlobalMap.get_path(pos, self.goal)[::-1]

    def calculate_speed(self):
        return self.speed

    def __repr__(self):
        return f"{ {key: getattr(self, key) for key in self.defined_keys} }"

    def to_json(self):
        self.position = tuple(self.position)
        return {
            "classname": self.__class__.__name__,
            "info": {key: getattr(self, key) for key in self.defined_keys}
        }
    
    @classmethod
    def from_json(cls, data):
        ant = cls(data["info"]["player_index"], data["info"]["id"], data["info"]["position"], data["info"]["cost"], data["info"]["color"])
        for key in data["info"]:
            setattr(ant, key, data["info"][key])
        return ant

class AntTypes(Enum):
    WORKER = auto()
    FIGHTER = auto()
    SETTLER = auto()

    def get_class(type: "AntTypes") -> Ant:
        if type == AntTypes.WORKER:
            return WorkerAnt
        elif type == AntTypes.FIGHTER:
            return FighterAnt
        elif type == AntTypes.SETTLER:
            return SettlerAnt
        raise ValueError(f"Unknown ant type {type}")

    def get_stats(type: "AntTypes"):
        if type == AntTypes.WORKER or str(type).strip("'") == "WORKER":
            return Worker
        elif type == AntTypes.FIGHTER or str(type).strip("'") == "FIGHTER":
            return Fighter
        elif type == AntTypes.SETTLER or str(type).strip("'") == "SETTLER":
            return Settler
        raise ValueError(f"Unknown ant type {type}")

    def __repr__(self) -> str:
        return f"\'{self.name}\'"

    def to_json(self):
        return self.__repr__()

class WorkerAnt(Ant):
    TYPE = AntTypes.WORKER

    def __init__(self, player_index, id, position, energy, color) -> None:
        super().__init__(player_index, id, position, energy, color)
        self.encumbered = False
        self.encumbered_energy = 0
        stats = AntTypes.get_stats(AntTypes.WORKER)
        self.remaining_trips = stats.TRIPS
        self.work_rate = stats.WORK_RATE
        self.encumbered_rate = stats.ENCUMBERED_RATE
        self.defined_keys.append("encumbered_energy")

    def alive(self) -> bool:
        return self.hp > 0 and self.remaining_trips > 0

    def free_energy(self):
        self.encumbered = False
        self.encumbered_energy = 0
        self.remaining_trips -= 1
    
    def calculate_speed(self):
        s = super().calculate_speed()
        if self.encumbered:
            return s * self.encumbered_rate
        return s

class FighterAnt(Ant):
    TYPE = AntTypes.FIGHTER

    def __init__(self, player_index, id, position, energy, color) -> None:
        super().__init__(player_index, id, position, energy, color)
        stats = AntTypes.get_stats(AntTypes.FIGHTER)
        self.attack_damage = stats.ATTACK
        self.range = stats.RANGE
        self.num_attacks = stats.NUM_ATTACKS
        self.defined_keys.append("ticks_left")

    def attack(self, other_ant) -> None:
        other_ant.hp -= self.attack_damage

class SettlerAnt(Ant):
    TYPE = AntTypes.SETTLER

    def __init__(self, player_index, id, position, energy, color) -> None:
        super().__init__(player_index, id, position, energy, color)
        self.defined_keys.append("ticks_left")
