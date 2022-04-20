import math
import json
from codequest22.server.ant import AntTypes
from codequest22.server.events import AttackEvent, DepositEvent, DieEvent, MoveEvent, SettlerScoreEvent, SpawnEvent
from codequest22.visual.game_input import _rec_finish_dict

class GameSummariser:

    @classmethod
    def get_results(cls, replay_path: str) -> dict:
        with open(replay_path, "r") as f:
            replay_obj = [_rec_finish_dict(json.loads(line)) for line in f.readlines()]
        return {
            "ants": {
                "worker": cls.get_worker_stats(replay_obj),
                "fighter": cls.get_fighter_stats(replay_obj),
                "settler": cls.get_settler_stats(replay_obj),
            },
            "misc": cls.get_misc_stats(replay_obj)
        }
    
    @classmethod
    def get_worker_stats(cls, replay_obj):
        data = None
        worker_ids = None
        for line in replay_obj:
            if line["type"] == "player_data":
                data = [{
                    "spawned": 0,
                    "deposits": 0,
                    "total_deposited": 0,
                    "hp_died": 0,
                    "lifetime_died": 0,
                } for _ in range(len(line["names"]))]
                worker_ids = [
                    set() for _ in range(len(line["names"]))
                ]
            elif line["type"] == "tick":
                for ev in line["events"]:
                    if isinstance(ev, SpawnEvent) and ev.ant_type == AntTypes.WORKER:
                        data[ev.player_index]["spawned"] += 1
                        worker_ids[ev.player_index].add(ev.ant_id)
                    elif isinstance(ev, DepositEvent) and ev.ant_id in worker_ids[ev.player_index]:
                        data[ev.player_index]["deposits"] += 1
                        data[ev.player_index]["total_deposited"] += ev.energy_amount
                    elif isinstance(ev, DieEvent) and ev.ant_id in worker_ids[ev.player_index]:
                        if ev.old_age:
                            data[ev.player_index]["lifetime_died"] += 1
                        else:
                            data[ev.player_index]["hp_died"] += 1
        return data

    @classmethod
    def get_fighter_stats(cls, replay_obj):
        data = None
        fighter_ids = None
        for line in replay_obj:
            if line["type"] == "player_data":
                data = [{
                    "spawned": 0,
                    "kills": 0,
                    "hp_died": 0,
                    "lifetime_died": 0,
                } for _ in range(len(line["names"]))]
                fighter_ids = [
                    set() for _ in range(len(line["names"]))
                ]
            elif line["type"] == "tick":
                for ev in line["events"]:
                    if isinstance(ev, SpawnEvent) and ev.ant_type == AntTypes.FIGHTER:
                        data[ev.player_index]["spawned"] += 1
                        fighter_ids[ev.player_index].add(ev.ant_id)
                    elif isinstance(ev, AttackEvent) and ev.attacker_id in fighter_ids[ev.attacker_index]:
                        if ev.defender_hp <= 0:
                            data[ev.attacker_index]["kills"] += 1
                    elif isinstance(ev, DieEvent) and ev.ant_id in fighter_ids[ev.player_index]:
                        if ev.old_age:
                            data[ev.player_index]["lifetime_died"] += 1
                        else:
                            data[ev.player_index]["hp_died"] += 1
        return data
    
    @classmethod
    def get_settler_stats(cls, replay_obj):
        data = None
        settler_ids = None
        for line in replay_obj:
            if line["type"] == "player_data":
                data = [{
                    "spawned": 0,
                    "hp_died": 0,
                    "lifetime_died": 0,
                    "points": 0,
                } for _ in range(len(line["names"]))]
                settler_ids = [
                    set() for _ in range(len(line["names"]))
                ]
            elif line["type"] == "tick":
                for ev in line["events"]:
                    if isinstance(ev, SpawnEvent) and ev.ant_type == AntTypes.SETTLER:
                        data[ev.player_index]["spawned"] += 1
                        settler_ids[ev.player_index].add(ev.ant_id)
                    elif isinstance(ev, SettlerScoreEvent) and ev.ant_id in settler_ids[ev.player_index]:
                        data[ev.player_index]["points"] += ev.score_amount
                    elif isinstance(ev, DieEvent) and ev.ant_id in settler_ids[ev.player_index]:
                        if ev.old_age:
                            data[ev.player_index]["lifetime_died"] += 1
                        else:
                            data[ev.player_index]["hp_died"] += 1
        return data

    @classmethod
    def get_misc_stats(cls, replay_obj):
        # IDEAS
        # Most aggressive - Highest ratio of fighter ants to other spawns
        # Marathon Runner - Longest average distance travelled per ant
        # Waiting for the right moment - Highest stored energy
        # Stayin Alive - Had the highest ratio of ants dying by age compared to HP loss.
        misc = {}
        # Most aggressive.
        fighter_spawned = []
        other_spawned = []
        for line in replay_obj:
            if line["type"] == "player_data":
                fighter_spawned = [0] * len(line["names"])
                other_spawned = [0] * len(line["names"])
            elif line["type"] == "tick":
                for ev in line["events"]:
                    if isinstance(ev, SpawnEvent):
                        if ev.ant_type == AntTypes.FIGHTER:
                            fighter_spawned[ev.player_index] += 1
                        else:
                            other_spawned[ev.player_index] += 1
        misc["aggressive"] = {
            "fighter": fighter_spawned,
            "other": other_spawned,
            "ratio": [(f/(f+o) if f+o > 0 else 0) for f, o in zip(fighter_spawned, other_spawned)]
        }
        # Marathon runner
        distance_travelled = []
        cur_positions = []
        total_spawns = []
        for line in replay_obj:
            if line["type"] == "player_data":
                distance_travelled = [0] * len(line["names"])
                total_spawns = [0] * len(line["names"])
                cur_positions = [{} for _ in range(len(line["names"]))]
            elif line["type"] == "tick":
                for ev in line["events"]:
                    if isinstance(ev, SpawnEvent):
                        total_spawns[ev.player_index] += 1
                        cur_positions[ev.player_index][ev.ant_id] = ev.position
                    elif isinstance(ev, MoveEvent):
                        old_pos = cur_positions[ev.player_index][ev.ant_id]
                        distance_travelled[ev.player_index] += math.sqrt((ev.position[0] - old_pos[0])**2 + (ev.position[1] - old_pos[1])**2)
        misc["marathon"] = {
            "distance": distance_travelled,
            "spawns": total_spawns,
            "avg": [(d/s if s > 0 else 0) for d, s in zip(distance_travelled, total_spawns)]
        }
        # Waiting for the right moment
        cur_max = []
        for line in replay_obj:
            if line["type"] == "player_data":
                cur_max = [0] * len(line["names"])
            elif line["type"] == "tick":
                for ev in line["events"]:
                    if isinstance(ev, DepositEvent):
                        # Deposit is the only way to increase energy
                        cur_max[ev.player_index] = max(cur_max[ev.player_index], ev.cur_energy)
        misc["waiting"] = {
            "max_energy": cur_max
        }
        # Stayin Alive
        hp_loss = []
        age = []
        for line in replay_obj:
            if line["type"] == "player_data":
                hp_loss = [0] * len(line["names"])
                age = [0] * len(line["names"])
            elif line["type"] == "tick":
                for ev in line["events"]:
                    if isinstance(ev, DieEvent):
                        if ev.old_age:
                            age[ev.player_index] += 1
                        else:
                            hp_loss[ev.player_index] += 1
        misc["alive"] = {
            "hp_loss": hp_loss,
            "age": age,
            "pct": [(a/(a+h) if a+h > 0 else 0) for a, h in zip(age, hp_loss)]
        }
        return misc

if __name__ == "__main__":
    print(GameSummariser.get_results("replay.txt"))
