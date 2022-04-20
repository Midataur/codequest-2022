import os.path
from os import getcwd
import random
from codequest22.server.events import *
from codequest22.server.network import NetworkManager
from codequest22.server.replay import ReplayManager
from codequest22.server.requests import *
from codequest22.server.global_map import GlobalMap, UnreachableException
from codequest22.server.ant import AntTypes
from codequest22 import stats
from traceback import format_exc
from pyqtree import Index
import numpy as np



def start_server(map_path, replay_path, recv_queue, visual_queue, error_queue, client_queues, is_visual):
    try:
        GlobalMap.load_map(map_path)
        spindex = Index(bbox=(-0.5, -0.5, GlobalMap.h - 0.5, GlobalMap.w - 0.5))
        ReplayManager.set_output(replay_path)
        NetworkManager.set_queues(visual_queue, recv_queue, client_queues, is_visual)

        # Send initial map data.
        NetworkManager.send_internal_obj({
            "type": "map", 
            "obj": GlobalMap.map_obj,
            "energy_info": GlobalMap.energy_tiles,
            "zones": GlobalMap.capture_zones,
        })

        zone_activated = [False for _ in GlobalMap.capture_zones]

        start_up = stats.hill.GRACE_PERIOD
        zone_times = len(GlobalMap.capture_zones) * stats.hill.NUM_ACTIVATIONS
        zone_allocations = [x % len(GlobalMap.capture_zones) for x in range(zone_times)]
        random.shuffle(zone_allocations)
        # Each capture zone will light up the same number of times.
        # Each time, this will take somewhere between 60 and 80 ticks, with 20 - 30 tick cooldown
        min_zone_time = stats.hill.MIN_ZONE_TIME
        max_zone_time = stats.hill.MAX_ZONE_TIME
        min_wait_time = stats.hill.MIN_WAIT_TIME
        max_bar = stats.general.SIMULATION_TICKS - start_up - zone_times * (max_zone_time + min_wait_time)
        zone_spots = [random.randint(1, max_bar) for _ in range(zone_times)]
        zone_spots.sort()
        zone_events = []
        cur_addition = start_up
        for i, v in enumerate(zone_spots):
            actual_zone_time = random.randint(min_zone_time, max_zone_time)
            zone_events.append((v + cur_addition, actual_zone_time, zone_allocations[i]))
            zone_events.append((v + cur_addition + actual_zone_time, None, zone_allocations[i]))
            cur_addition += actual_zone_time + min_wait_time

        tile_activated = [1 for _ in GlobalMap.energy_tiles]
        tile_start_up = stats.energy.GRACE_PERIOD
        min_tile_wait_time = stats.energy.MIN_WAIT_TIME
        min_tile_activation_time = stats.energy.MIN_OVERCHARGE_TIME
        max_tile_activation_time = stats.energy.MAX_OVERCHARGE_TIME
        n_times = stats.energy.NUM_ACTIVATIONS
        tile_times = [
            [
                random.randint(1, stats.general.SIMULATION_TICKS - tile_start_up - (min_tile_wait_time + max_tile_activation_time) * n_times)
                for _ in range(n_times)
            ] for _ in range(len(tile_activated))
        ]
        tile_events = []
        for x in range(len(tile_times)):
            tile_times[x].sort()
            for y in range(n_times):
                start = tile_times[x][y] + tile_start_up + y * (min_tile_wait_time + max_tile_activation_time)
                length = random.randint(min_tile_activation_time, max_tile_activation_time)
                tile_events.append((start, length, x))
                tile_events.append((start+length, None, x))

        player_data = NetworkManager.recv_client_response()
        # Send every client their index.
        for i in range(len(client_queues)):
            NetworkManager.send_client(i, {
                "index": i,
                "total": len(client_queues),
            })

        NetworkManager.broadcast_obj({
            "type": "map", 
            "obj": GlobalMap.map_obj,
            "energy_tiles": {
                tuple(e.position[::-1]): e.amount
                for e in GlobalMap.energy_tiles
            }
        })

        ants = [
            {} for d in player_data
        ]
        player_energy = [stats.general.STARTING_ENERGY for d in player_data]
        player_health = [stats.general.QUEEN_HEALTH for d in player_data]
        defeated = [False for d in player_data]
        hill_score = [0 for _ in player_data]
        # Remove cwd from absolute paths
        image_names = [os.path.relpath(d["image"], getcwd()) for d in player_data]
        NetworkManager.send_internal_obj({
            "type": "player_data", 
            "names": [d["name"] for d in player_data], 
            "images": image_names,
            "energy": player_energy,
            "hill": hill_score,
            "health": player_health,
        })

        ant_ids = [0 for d in player_data]
        alive_ants = [
            {
                AntTypes.WORKER: 0,
                AntTypes.FIGHTER: 0,
                AntTypes.SETTLER: 0,
            } 
            for d in player_data
        ]

        player_events = []
        server_events = []
        failed_requests = []
        cur_tick = 0
        last_succeeded_request = 0
        while True:
            NetworkManager.broadcast_obj({
                "type": "tick", 
                "events": player_events,
                "failed_requests": failed_requests,
            })
            player_events = []
            server_events = []
            failed_requests = []
            group_requests = NetworkManager.recv_client_response()
            for i, d in enumerate(player_data):
                n_spawns = 0
                if defeated[i] and group_requests[i]:
                    group_requests[i][0].player_index = i
                    group_requests[i][0].reason = f"You are defeated. Please stop sending events."
                    failed_requests.append(group_requests[i][0])
                    continue
                for req in group_requests[i]:
                    if isinstance(req, SpawnRequest):
                        if n_spawns >= stats.general.MAX_SPAWNS_PER_TICK:
                            req.player_index = i
                            req.reason = f"You can only spawn {stats.general.MAX_SPAWNS_PER_TICK} ants per tick."
                            failed_requests.append(req)
                            continue
                        if player_energy[i] < req.cost:
                            req.player_index = i
                            req.reason = f"Does not have enough energy to request. Requested spawn with cost {req.cost}, but you have {player_energy[i]} energy"
                            failed_requests.append(req)
                            continue
                        if req.cost < AntTypes.get_stats(req.ant_type).COST:
                            req.player_index = i
                            req.reason = f"The basic ant of this type requires at least {AntTypes.get_stats(req.ant_type).COST} energy, you cannot pay {req.cost} energy"
                            failed_requests.append(req)
                            continue
                        if alive_ants[i][AntTypes.WORKER] + alive_ants[i][AntTypes.FIGHTER] + alive_ants[i][AntTypes.SETTLER] >= stats.general.MAX_ANTS_PER_PLAYER:
                            req.player_index = i
                            req.reason = f"You already have {stats.general.MAX_ANTS_PER_PLAYER} on the field, cannot spawn more until ants die."
                            failed_requests.append(req)
                            continue
                        if req.id in ants[i]:
                            req.player_index = i
                            req.reason = f"There already exists one of your ants with id {req.id}"
                            failed_requests.append(req)
                            continue
                        if req.id is None:
                            req.id = f"ant-{ant_ids[i]}"
                            ant_ids[i] += 1
                        klass = AntTypes.get_class(req.ant_type)
                        ant_obj = klass(i, req.id, GlobalMap.player_spawns[i], req.cost, req.color or (0, 0, 0, 0))
                        if req.goal is not None:
                            try:
                                ant_obj.set_goal(req.goal)
                            except UnreachableException:
                                req.player_index = i
                                req.reason = f"Ant goal position is unreachable. Ant position: {ant_obj.position[::-1]}, Ant goal: {req.goal[::-1]}"
                                failed_requests.append(req)
                                continue
                        ants[i][req.id] = ant_obj
                        spindex.insert((ant_obj.player_index, ant_obj.id), (*ant_obj.position, *ant_obj.position))
                        player_energy[i] -= req.cost
                        player_events.append(SpawnEvent(ant_obj, req.cost))
                        server_spawn = SpawnEvent(ant_obj, req.cost, ant_obj.color, getattr(ant_obj, "_follow_path", []), ant_obj.goal)
                        server_events.append(server_spawn)
                        alive_ants[i][req.ant_type] += 1
                        n_spawns += 1
                        last_succeeded_request = cur_tick
                    elif isinstance(req, GoalRequest):
                        if req.ant_id not in ants[i]:
                            req.player_index = i
                            req.reason = f"This ant does not exist: {req.ant_id}"
                            failed_requests.append(req)
                            continue
                        x, y = req.position
                        try:
                            ants[i][req.ant_id].set_goal((y, x))
                            last_succeeded_request = cur_tick
                        except UnreachableException:
                            req.player_index = i
                            req.reason = f"Ant goal position is unreachable. Ant position: {ant_obj.position[::-1]}, Ant goal: {(x, y)}"
                            failed_requests.append(req)
                            continue
            for i in range(len(player_data)):
                for ant in ants[i].values():
                    if ant.alive():
                        ant.tick()
            for i in range(len(player_data)):
                for k, v in ants[i].items():
                    if v.alive() and v.moved:
                        spindex.remove((v.player_index, v.id), (*v.old_position, *v.old_position))
                        spindex.insert((v.player_index, v.id), (*v.position, *v.position))
                        player_events.append(MoveEvent(v))
                        server_move = MoveEvent(v, getattr(v, "_follow_path", []))
                        server_events.append(server_move)
            # Handle all attacks
            for i in range(len(player_data)):
                for k, v in ants[i].items():
                    if v.TYPE == AntTypes.FIGHTER:
                        # Search in a range around this ant.
                        count = 0
                        attack_range = v.range
                        possible = spindex.intersect((
                            v.position[0]-attack_range,
                            v.position[1]-attack_range,
                            v.position[0]+attack_range,
                            v.position[1]+attack_range,
                        ))
                        to_remove = []
                        for i2 in range(len(possible)):
                            a = ants[possible[i2][0]][possible[i2][1]]
                            if (
                                (a.position[0] - v.position[0]) * (a.position[0] - v.position[0]) + 
                                (a.position[1] - v.position[1]) * (a.position[1] - v.position[1])
                            ) > attack_range:
                                to_remove.append(i2)
                            elif possible[i2][0] == i:
                                to_remove.append(i2)
                        for i2 in to_remove[::-1]:
                            del possible[i2]
                        # We now have all possible candidates for attack.
                        n_attacks = min(len(possible), v.num_attacks)
                        choices = np.random.choice(len(possible), size=n_attacks, replace=False)
                        for c in choices:
                            i2, id2 = possible[c]
                            # Attack!
                            v.attack(ants[i2][id2])
                            player_events.append(AttackEvent(v, ants[i2][id2]))
                            server_events.append(AttackEvent(v, ants[i2][id2]))
                            count += 1
                        if count < v.num_attacks:
                            for i2 in range(len(player_data)):
                                if i != i2:
                                    dx = GlobalMap.player_spawns[i2][0] - v.position[0]
                                    dy = GlobalMap.player_spawns[i2][0] - v.position[0]
                                    if dx * dx + dy * dy <= attack_range:
                                        # Attack Queen
                                        player_health[i2] -= v.attack_damage * (v.num_attacks - count)
                                        player_health[i2] = max(player_health[i2], 0)
                                        ev = QueenAttackEvent(v, i2, player_health[i2])
                                        player_events.append(ev)
                                        server_events.append(ev)
                                        if player_health[i2] <= 0:
                                            defeated[i2] = True
                                            hill_score[i] += hill_score[i2]
                                            ev2 = TeamDefeatedEvent(i2, i, hill_score[i])
                                            player_events.append(ev2)
                                            server_events.append(ev2)
            # Handle deaths
            for i in range(len(player_data)):
                to_remove = []
                for k, v in ants[i].items():
                    if not v.alive():
                        to_remove.append(k)
                        player_events.append(DieEvent(v))
                        server_events.append(DieEvent(v))
                for k in to_remove:
                    spindex.remove((ants[i][k].player_index, ants[i][k].id), (*ants[i][k].position, *ants[i][k].position))
                    alive_ants[i][ants[i][k].TYPE] -= 1
                    del ants[i][k]
            # Worker actions
            # step 1: Queen depositing
            for i in range(len(player_data)):
                for k, v in ants[i].items():
                    if v.TYPE == AntTypes.WORKER and v.encumbered:
                        # Am I close to my queen?
                        ipos = (round(v.position[0]), round(v.position[1]))
                        if GlobalMap.player_spawns[i][0] == ipos[0] and GlobalMap.player_spawns[i][1] == ipos[1]:
                            # Deposit my energy
                            dv = DepositEvent(v, player_energy[i])
                            player_energy[i] = dv.total_energy
                            player_events.append(dv)
                            server_events.append(dv)
                            v.free_energy()
            # step 2: Tile production
            for t in GlobalMap.energy_tiles:
                if t.cur_delay > 0:
                    t.cur_delay -= 1
            ballots = [[] for _ in GlobalMap.energy_tiles]
            for i in range(len(player_data)):
                for k, v in ants[i].items():
                    if v.TYPE == AntTypes.WORKER and not v.encumbered:
                        # Am I close to a food tile?
                        for i2, tile in enumerate(GlobalMap.energy_tiles):
                            ipos = (round(v.position[0]), round(v.position[1]))
                            if tile.position == ipos:
                                # Pick up some energy
                                ballots[i2].append((i, k))
            for i2, (tile, b) in enumerate(zip(GlobalMap.energy_tiles, ballots)):
                mult = tile_activated[i2]
                if tile.cur_delay > 0: continue
                for x in range(tile.per_tick):
                    if not b: continue
                    tile.cur_delay = tile.max_delay
                    i, k = random.choice(b)
                    v = ants[i][k]
                    v.encumbered = True
                    v.encumbered_energy = v.work_rate * tile.amount * mult
                    player_events.append(ProductionEvent(v))
                    server_events.append(ProductionEvent(v))
                    for b2 in ballots:
                        if (i, k) in b2:
                            b2.remove((i, k))
            # Settler scoring
            for i in range(len(player_data)):
                for k, v in ants[i].items():
                    if v.TYPE == AntTypes.SETTLER:
                        rounded_pos = round(v.position[0]), round(v.position[1])
                        for i2, zone in enumerate(zone_activated):
                            if not zone: continue
                            if rounded_pos in GlobalMap.capture_zones[i2]:
                                hill_score[i] += 1
                                server_events.append(SettlerScoreEvent(v, 1))
                                player_events.append(SettlerScoreEvent(v, 1))
                                break
            # Zone activation / deactivation
            for tick, length, z in zone_events:
                if cur_tick == tick:
                    if length is None:
                        zone_activated[z] = False
                        server_events.append(ZoneDeactivateEvent(z, [p[::-1] for p in GlobalMap.capture_zones[z]]))
                        player_events.append(ZoneDeactivateEvent(z, [p[::-1] for p in GlobalMap.capture_zones[z]]))
                    else:
                        zone_activated[z] = True
                        server_events.append(ZoneActiveEvent(z, length, [p[::-1] for p in GlobalMap.capture_zones[z]]))
                        player_events.append(ZoneActiveEvent(z, length, [p[::-1] for p in GlobalMap.capture_zones[z]]))
            # Tile activation / deactivation
            for tick, length, z in tile_events:
                if cur_tick == tick:
                    if length is None:
                        tile_activated[z] = 1
                        server_events.append(FoodTileDeactivateEvent(GlobalMap.energy_tiles[z].position[::-1]))
                        player_events.append(FoodTileDeactivateEvent(GlobalMap.energy_tiles[z].position[::-1]))
                    else:
                        tile_activated[z] = 2
                        server_events.append(FoodTileActiveEvent(GlobalMap.energy_tiles[z].position[::-1], length, 2))
                        player_events.append(FoodTileActiveEvent(GlobalMap.energy_tiles[z].position[::-1], length, 2))
            NetworkManager.send_internal_obj({
                "type": "tick",
                "events": server_events,
                "failed_requests": failed_requests,
            })
            NetworkManager.wait_visual_response()
            cur_tick += 1
            # if cur_tick % 10 == 0:
            #     print(cur_tick)
            # Check for any players with no ants and not enough nectar to buy any ants
            for i in range(len(player_data)):
                if not defeated[i] and len(ants[i].keys()) == 0:
                    if player_energy[i] < min(stats.ants.Fighter.COST, stats.ants.Worker.COST, stats.ants.Settler.COST):
                        defeated[i] = True
            if cur_tick == stats.general.SIMULATION_TICKS or cur_tick - last_succeeded_request > 100:
                NetworkManager.broadcast_obj({"type": "finish"})
                for i in range(len(hill_score)):
                    if defeated[i]:
                        hill_score[i] = 0
                sorted_hill_score = list(sorted(hill_score))
                cur_score = 100
                score_map = {}
                for v in sorted_hill_score[::-1]:
                    if v not in score_map:
                        score_map[v] = cur_score
                        cur_score -= 25
                NetworkManager.send_internal_obj({
                    "type": "winner",
                    "indicies": [i for i in range(len(hill_score)) if hill_score[i] == max(hill_score)],
                    "score": [score_map[hill_score[i]] for i in range(len(hill_score))]
                })
                ReplayManager.close()
                if not is_visual:
                    error_queue.put("server")
                return
            if len(defeated) - sum(defeated) <= 1 and (len(defeated) != 1 or len(defeated) == sum(defeated)):
                if sum(defeated) == len(defeated) - 1 and max(hill_score) != max(hill_score[i] for i in range(len(hill_score)) if not defeated[i]):
                    # Keep going
                    pass
                else:
                    # Game over.
                    for i in range(len(hill_score)):
                        if defeated[i]:
                            hill_score[i] = 0
                    sorted_hill_score = list(sorted(hill_score))
                    cur_score = 100
                    score_map = {}
                    for v in sorted_hill_score[::-1]:
                        if v not in score_map:
                            score_map[v] = cur_score
                            cur_score -= 25
                    NetworkManager.send_internal_obj({
                        "type": "winner",
                        "indicies": [i for i in range(len(hill_score)) if hill_score[i] == max(hill_score)],
                        "score": [score_map[hill_score[i]] for i in range(len(hill_score))]
                    })
                    ReplayManager.close()
                    if not is_visual:
                        error_queue.put("server")
                    return
    except Exception as e:
        error_queue.put([e, format_exc()])

def start_server_replay(map_path, replay_path, recv_queue, visual_queue, client_queues):
    ReplayManager.set_input(replay_path)
    NetworkManager.set_queues(visual_queue, recv_queue, client_queues)
    
    # Send initial map data.
    NetworkManager.send_internal_obj(eval(ReplayManager.read_line()), replay=False)
    # Send player data
    NetworkManager.send_internal_obj(eval(ReplayManager.read_line()), replay=False)

    # Sample mainloop
    while True:
        line = eval(ReplayManager.read_line())
        NetworkManager.send_internal_obj(line, replay=False)
        NetworkManager.wait_visual_response()
        if line["type"] == "winner":
            return
