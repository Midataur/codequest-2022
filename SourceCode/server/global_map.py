import json
import math
from heapq import heappush, heappop

from codequest22.server.energy import EnergyTile

class UnreachableException(Exception):
    pass

class GlobalMap:

    map_obj = None

    @classmethod
    def load_map(cls, map_path):
        with open(map_path, "r") as f:
            obj = json.loads(f.read())
        try:
            cls.player_spawns = [None]*4
            cls.energy_tiles = []
            cls.capture_zones = []
            cls.h = len(obj["map"])
            cls.w = len(obj["map"][0])
            energy_info = obj.get("energy", {})
            new_energy_info = {
                eval(key)[::-1]: value
                for key, value in energy_info.items()
            }
            remap = "XW.GBZRYF"
            cls.map_obj = [[remap[obj["map"][x][y]] for y in range(cls.w)] for x in range(cls.h)]
            for x in range(cls.h):
                for y, c in enumerate(cls.map_obj[x]):
                    for i, c2 in enumerate("RBYG"):
                        if c == c2:
                            cls.player_spawns[i] = (x, y)
                    if c == "F":
                        amount = new_energy_info.get((x, y), -1)
                        cls.energy_tiles.append(EnergyTile((x, y), amount))
            for tile in cls.energy_tiles:
                if tile.amount == -1:
                    print("WARNING: The following production zone has no energy information:", tile.position[::-1])
            # Locate zones with DFS
            dealt = [[False for y in range(cls.w)] for x in range(cls.h)]
            for x in range(cls.h):
                for y in range(cls.w):
                    if cls.map_obj[x][y] == "Z" and not dealt[x][y]:
                        q = [(x, y)]
                        cls.capture_zones.append([])
                        while q:
                            (a, b) = q.pop()
                            cls.capture_zones[-1].append((a, b))
                            dealt[a][b] = True
                            for (c, d) in [(a+1, b),(a-1, b),(a, b+1),(a, b-1)]:
                                if not (0 <= c < cls.h and 0 <= d < cls.w): continue
                                if cls.map_obj[c][d] in "ZF" and not dealt[c][d] and (c, d) not in q:
                                    q.append((c, d))
        except Exception as e:
            print(f"An error occured, while reading the map {map_path}, given below:")
            raise e
        cls._precomp_paths()

    @classmethod
    def get_path(cls, p1, p2):
        if cls.map_obj[round(p1[0])][round(p1[1])] == "W" or cls.map_obj[round(p2[0])][round(p2[1])] == "W":
            raise UnreachableException()
        final_path = []
        to_add = []
        if type(p1[0]) != int or type(p1[1]) != int:
            p1 = (round(p1[0]), round(p1[1]))
            final_path.insert(0, p1)
        if type(p2[0]) != int or type(p2[1]) != int:
            to_add.append(p2)
            p2 = (round(p2[0]), round(p2[1]))
        if p2 not in cls.best_path[p1]:
            raise UnreachableException()
        p = cls.best_path[p1][p2][1]
        while p != p2:
            final_path.append(p)
            p = cls.best_path[p][p2][1]
        final_path.append(p2)
        return final_path + to_add


    @classmethod
    def _precomp_paths(cls):
        # 4*W*H*log(W*H)*W*H = V^2 LOG(V) runtime.
        # So at 30 seconds, Total area = 10^4 (100*100) is a reasonable size to expect students to get everything done.
        cls._gen_special_points()
        # Dijkstra's for every start point.
        # Generate edges
        adj = {}
        cls.best_path = {}
        h, w = len(cls.map_obj), len(cls.map_obj[0])
        points = []
        for y in range(h):
            for x in range(w):
                cls.best_path[(y, x)] = {}
                adj[(y, x)] = []
                if cls.map_obj[y][x] == "W": continue
                points.append((y, x))
        for y, x in points:
            for a, b in [(y+1, x), (y-1, x), (y, x+1), (y, x-1)]:
                if 0 <= a < h and 0 <= b < w and cls.map_obj[a][b] != "W":
                    adj[(y, x)].append((a, b, 1))
            for a, b in [(y+1, x+1), (y+1, x-1), (y-1, x+1), (y-1, x-1)]:
                if (
                    0 <= a < h and 0 <= b < w and 
                    cls.map_obj[a][b] != "W" and
                    cls.map_obj[a][x] != "W" and
                    cls.map_obj[y][b] != "W"
                ):
                    adj[(y, x)].append((a, b, math.sqrt(2)))
        idx = {p: i for i, p in enumerate(points)}
        for y, x in points:
            # Dijkstra run
            expanded = [False] * len(points)
            seen = [False] * len(points)
            queue = []
            heappush(queue, (0, (y, x), (y, x)))
            while queue:
                d, (a, b), parent = heappop(queue)
                if expanded[idx[(a, b)]]: continue
                expanded[idx[(a, b)]] = True
                cls.best_path[(y, x)][(a, b)] = (d, parent)
                # Look at all neighbours
                for j, k, d2 in adj[(a, b)]:
                    if not expanded[idx[(j, k)]]:
                        heappush(queue, (
                            d + d2,
                            (j, k),
                            parent if (parent in cls.special_points and parent != (y, x)) else (j, k)
                        ))

    @classmethod
    def _gen_special_points(cls):
        """
        A special point is one with two traversable tiles adjacent and a non-traversable tile in between.

        This ensures that the shortest path between any tiles on the map can begin with a straight line to a special point.
        """
        cls.special_points = set()
        h, w = len(cls.map_obj), len(cls.map_obj[0])
        for y in range(h):
            for x in range(w):
                points = [(y, x+1), (y+1, x+1), (y+1, x), (y+1, x-1), (y, x-1), (y-1, x-1), (y-1, x), (y-1, x+1), (y, x+1)]
                for z in range(0, 7, 2):
                    if 0 <= points[z][0] < h and 0 <= points[z][1] < w and 0 <= points[z+2][0] < h and 0 <= points[z+2][1] < w:
                        if (
                            cls.map_obj[points[z][0]][points[z][1]] != "W" and
                            cls.map_obj[points[z+1][0]][points[z+1][1]] == "W" and
                            cls.map_obj[points[z+2][0]][points[z+2][1]] != "W"
                        ):
                            cls.special_points.add((y, x))
    

