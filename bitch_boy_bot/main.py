from glob import glob
from typing import overload
from codequest22.server.ant import AntTypes
import codequest22.stats as stats
from codequest22.server.events import DepositEvent, DieEvent, ProductionEvent, SpawnEvent, MoveEvent, FoodTileActiveEvent, FoodTileDeactivateEvent, TeamDefeatedEvent, ZoneActiveEvent
from codequest22.server.requests import GoalRequest, SpawnRequest
import heapq

from codequest22.stats import energy
from dist import dist_init, dist
import math
import numpy as np


def get_team_name():
    return f"M.A.X"

my_index = None
def read_index(player_index, n_players):
    global my_index
    my_index = player_index

my_energy = stats.general.STARTING_ENERGY
map_data = {}
spawns = [None]*4
food = []
distance = {}
closest_site = None
empty = []
wall = []
hill = []
my_ants={}
opp1_ants={}
opp2_ants={}
opp3_ants={}
#tracks all food sites with useful info
food_map = {}
#threshold for spawning fighters; higher is less aggresive
aggression = 90
#how many slots we should leave free for special spawns
buffer = 2

def read_map(md, energy_info):
    global map_data, spawns, food, distance, closest_site, food_sites, food_map
    map_data = md

    for y in range(len(map_data)):
        for x in range(len(map_data[0])):
            if map_data[y][x] == "F":
                food.append((x, y))
            if map_data[y][x] == ".":
                empty.append((x, y))
            if map_data[y][x] == "W":
                wall.append((x, y))
            if map_data[y][x] == "Z":
                hill.append((x, y))
            if map_data[y][x] in "RBYG":
                spawns["RBYG".index(map_data[y][x])] = (x, y)
    # Read map is called after read_index
    startpoint = spawns[my_index]
    # Dijkstra's Algorithm: Find the shortest path from your spawn to each food zone.
    # Step 1: Generate edges - for this we will just use orthogonally connected cells.
    adj = {}
    h, w = len(map_data), len(map_data[0])
    # A list of all points in the grid
    points = []
    # Mapping every point to a number
    idx = {}
    counter = 0
    for y in range(h):
        for x in range(w):
            adj[(x, y)] = []
            if map_data[y][x] == "W": continue
            points.append((x, y))
            idx[(x, y)] = counter
            counter += 1
    for x, y in points:
        for a, b in [(y+1, x), (y-1, x), (y, x+1), (y, x-1)]:
            if 0 <= a < h and 0 <= b < w and map_data[a][b] != "W":
                adj[(x, y)].append((b, a, 1))
    # Step 2: Run Dijkstra's
    # What nodes have we already looked at?
    expanded = [False] * len(points)
    # What nodes are we currently looking at?
    queue = []
    # What is the distance to the startpoint from every other point?
    heapq.heappush(queue, (0, startpoint))
    while queue:
        d, (a, b) = heapq.heappop(queue)
        if expanded[idx[(a, b)]]: continue
        # If we haven't already looked at this point, put it in expanded and update the distance.
        expanded[idx[(a, b)]] = True
        distance[(a, b)] = d
        # Look at all neighbours
        for j, k, d2 in adj[(a, b)]:
            if not expanded[idx[(j, k)]]:
                heapq.heappush(queue, (
                    d + d2,
                    (j, k)
                ))
    # Now I can calculate the closest food site.
    food_sites = list(sorted(food, key=lambda prod: distance[prod]))
    closest_site = food_sites[0]

    #construct food map
    for tile in energy_info.keys():
        baserate = energy_info[tile]
        overclocked = False
        distance_from_base = distance[tile]
        food_map[tile] = {
            'baserate': baserate,
            'overclocked': overclocked,
            'distance_from_base': distance_from_base
        }

    dist_init(map_data)

    for tile in energy_info.keys():
        print(f'For {my_index}, {tile} is {food_source_power(tile)}')

def handle_failed_requests(requests):
    global my_energy
    for req in requests:
        if req.player_index == my_index:
            print(f"Request {req.__class__.__name__} failed. Reason: {req.reason}.")
            raise ValueError()

def ants_around_tile(tile, radius):
    #needs to be fixed when ants is implented properly
    all_ants = dict(my_ants)
    all_ants.update(opp1_ants)
    all_ants.update(opp2_ants)
    all_ants.update(opp3_ants)

    surrounding = []
    tile_pos = np.array(tile)+np.array((0.5,0.5))

    for ident in all_ants.keys():
        antpos = np.array(all_ants[ident]['pos'])
        euclidean_dist = np.linalg.norm(antpos-tile_pos)
        if euclidean_dist <= radius:
            surrounding.append(ident)

    return surrounding


def food_source_power(tile):
    global food_map
    #get all the raw variables
    baserate = food_map[tile]['baserate']
    oc_rate = 2 if food_map[tile]['overclocked'] else 1
    distance = food_map[tile]['distance_from_base']
    speed = stats.ants.Worker.SPEED
    encumberance = stats.ants.Worker.ENCUMBERED_RATE
    delay = stats.energy.DELAY
    per_tick = stats.energy.PER_TICK
    #ants_around needs to be implented properly
    ants_around = len(ants_around_tile(tile,3))+1

    #extend the domain of log
    ln = lambda x: -float('inf') if x == 0 else math.log(x)

    #actually assemble the function
    travel_term = (distance/speed)*(1+1/encumberance)
    prob_term = (-delay/per_tick)*(ln(2)/(ln(ants_around-1)-ln(ants_around)))
    return (baserate*oc_rate)/(travel_term+prob_term)

def attack_target():
    global spawns, distances
    possible = list(spawns)
    
    while None in possible:
        possible.remove(None)

    #attack the closest base
    return sorted(possible, key=lambda x: distance[x])[1]

def handle_events(events):
    global my_energy, food_map, my_ants
    requests = []

    spawned_this_tick = 0

    for ev in events:
        if isinstance(ev, DepositEvent):
            if ev.player_index == my_index:
                # One of my worker ants just made it back to the Queen! Let's send them back to the food site.
                #find best possible goal
                goal = sorted(food_map.keys(), key=food_source_power)[-1]
                requests.append(GoalRequest(ev.ant_id, goal))
                # Additionally, let's update how much energy I've got.
                my_energy = ev.cur_energy
        elif isinstance(ev, ProductionEvent):
            if ev.player_index == my_index:
                # One of my worker ants just made it to the food site! Let's send them back to the Queen.
                requests.append(GoalRequest(ev.ant_id, spawns[my_index]))
        elif isinstance(ev, DieEvent):
            if ev.player_index == my_index:
                # One of my workers just died :(
                my_ants.pop(ev.ant_id)
        elif isinstance(ev,MoveEvent):
            if ev.player_index == my_index:
                my_ants[ev.ant_id]['pos'] = ev.position

        elif isinstance(ev, SpawnEvent):
            if ev.player_index == my_index:
                my_ants[ev.ant_id] = {
                    'type': ev.ant_type,
                    'goal': ev.goal,
                    'pos': ev.position,
                    'hp': ev.hp
                }

        elif isinstance(ev, FoodTileActiveEvent) or isinstance(ev, FoodTileDeactivateEvent):
            #toggles overclocked status
            food_map[ev.pos]['overclocked'] ^= True

        elif isinstance(ev, TeamDefeatedEvent):
            #toggles overclocked status
            spawns[ev.defeated_index] = None

        elif isinstance(ev, ZoneActiveEvent):
            if my_energy >= stats.ants.Settler.COST and len(my_ants.keys()) <= stats.general.MAX_ANTS_PER_PLAYER:
                spawned_this_tick += 1
                my_energy -= stats.ants.Settler.COST
                requests.append(SpawnRequest(ant_class, id=None, color=None, goal=ev.points[0]))

    # Can I spawn ants?
    while (
        len(my_ants.keys())+spawned_this_tick+buffer < stats.general.MAX_ANTS_PER_PLAYER and 
        spawned_this_tick < stats.general.MAX_SPAWNS_PER_TICK and
        my_energy >= stats.ants.Worker.COST
    ):
        #find best possible goal
        goal = sorted(food_map.keys(), key=food_source_power)[-1]

        ant_class = AntTypes.WORKER
        cost = stats.ants.Worker.COST

        #start killing
        if len(my_ants.keys())+spawned_this_tick >= aggression and my_energy >= stats.ants.Fighter.COST:
            ant_class = AntTypes.FIGHTER
            goal = attack_target()
            cost = stats.ants.Fighter.COST
            print('here i go killing again!')

        spawned_this_tick += 1
        # Spawn an ant, give it some id, no color, and send it to the closest site.
        # I will pay the base cost for this ant, so cost=None.
        requests.append(SpawnRequest(ant_class, id=None, color=None, goal=goal))
        my_energy -= cost

    return requests
