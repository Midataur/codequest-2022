import math
import os

this_dir = os.path.dirname(os.path.abspath(__file__))
cwd = os.getcwd()

def interpolate_colour_discrete(colours, max_v, min_v, actual):
    proportion = min(1, max(0, (actual - min_v) / (max_v - min_v)))
    index = min(len(colours)-1, math.floor(proportion * len(colours)))
    return colours[index]

def resolve_path(path, fallback=None):
    if path is not None:
        if os.path.isfile(os.path.join(cwd, path)):
            return os.path.join(cwd, path)
        elif os.path.isfile(os.path.join(this_dir, path)):
            return os.path.join(this_dir, path)
    if fallback is None:
        raise ValueError(f"Unknown sprite path {path}")
    return resolve_path(fallback, None)

def hex_to_rgb(hex):
    hex = hex.replace("#", "")
    return tuple(int(hex[i:i+2], 16) for i in range(0, len(hex), 2))
