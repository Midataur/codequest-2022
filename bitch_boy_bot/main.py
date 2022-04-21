from glob import glob
from typing import overload
from codequest22.server.ant import AntTypes
import codequest22.stats as stats
from codequest22.server.events import DepositEvent, DieEvent, ProductionEvent, SpawnEvent, MoveEvent, FoodTileActiveEvent, FoodTileDeactivateEvent
from codequest22.server.requests import GoalRequest, SpawnRequest
import heapq

from codequest22.stats import energy
from dist import dist_init, dist
import math


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
total_ants = 0
empty = []
wall = []
hill = []
my_ants={}
opp1_ants={}
opp2_ants={}
opp3_ants={}
#tracks all food sites with useful info
food_map = {}

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
    ants_around = 1

    #extend the domain of log
    ln = lambda x: -float('inf') if x == 0 else math.log(x)

    #actually assemble the function
    travel_term = (distance/speed)*(1+1/encumberance)
    prob_term = (-delay/per_tick)*(ln(2)/(ln(ants_around-1)-ln(ants_around)))
    return (baserate*oc_rate)/(travel_term+prob_term)

def handle_events(events):
    global my_energy, total_ants, food_map, my_ants
    requests = []

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
                total_ants -= 1
                if ev.player_index == my_index:
                    my_ants.pop(ev.ant_id)
        elif isinstance(ev,MoveEvent):
            if ev.player_index == my_index:
                my_ants[ev.ant_id][2]=ev.position

        elif isinstance(ev, SpawnEvent):
            if ev.player_index == my_index:
                my_ants[ev.ant_id]=[ev.ant_type,ev.goal,ev.position,ev.hp]

        elif isinstance(ev, FoodTileActiveEvent) or isinstance(ev, FoodTileDeactivateEvent):
            #toggles overclocked status
            food_map[ev.pos]['overclocked'] ^= True

    # Can I spawn ants?
    spawned_this_tick = 0
    while (
        total_ants < stats.general.MAX_ANTS_PER_PLAYER and 
        spawned_this_tick < stats.general.MAX_SPAWNS_PER_TICK and
        my_energy >= stats.ants.Worker.COST
    ):
        #find best possible goal
        goal = sorted(food_map.keys(), key=food_source_power)[-1]

        #how the fuck did we get here?
        if goal == (5,1):
            print(food_source_power(5,1))

        spawned_this_tick += 1
        total_ants += 1
        # Spawn an ant, give it some id, no color, and send it to the closest site.
        # I will pay the base cost for this ant, so cost=None.
        requests.append(SpawnRequest(AntTypes.WORKER, id=None, color=None, goal=goal))
        my_energy -= stats.ants.Worker.COST

    return requests
