import json
from queue import Empty
from typing import Optional
from multiprocessing import Queue

RECV_QUEUE: Optional[Queue] = None
SEND_QUEUE: Optional[Queue] = None

IS_REPLAY = None
REPLAY_PATH = None
REPLAY_LINES = None
CUR_REPLAY_INDEX = None

# Import JSON Serializable custom classes.
from codequest22.server.energy import EnergyTile
from codequest22.server.events import *
from codequest22.server.requests import *
from codequest22.server.ant import *

def _rec_finish_dict(object):
    # Recursively fill in json data with appropriate classes
    if isinstance(object, list):
        for i, o in enumerate(object):
            object[i] = _rec_finish_dict(o)
    elif isinstance(object, dict):
        for k, o in object.items():
            object[k] = _rec_finish_dict(o)
        if "classname" in object:
            return eval(object["classname"]).from_json(object)
    return object

def use_queue(recv, send):
    global RECV_QUEUE, SEND_QUEUE, IS_REPLAY
    IS_REPLAY = False
    RECV_QUEUE = recv
    SEND_QUEUE = send

def use_replay(replay_path):
    global IS_REPLAY, REPLAY_PATH, REPLAY_LINES, CUR_REPLAY_INDEX
    IS_REPLAY = True
    REPLAY_PATH = replay_path
    with open(replay_path, "r") as f:
        REPLAY_LINES = f.readlines()
    CUR_REPLAY_INDEX = 0

def is_replay():
    return IS_REPLAY

def send(data):
    SEND_QUEUE.put(data)

def get_input_wait():
    global CUR_REPLAY_INDEX
    if IS_REPLAY:
        l = json.loads(REPLAY_LINES[CUR_REPLAY_INDEX])
        l = _rec_finish_dict(l)
        CUR_REPLAY_INDEX += 1
        return l
    return RECV_QUEUE.get()

def get_input_maybe():
    global CUR_REPLAY_INDEX
    if IS_REPLAY and CUR_REPLAY_INDEX < len(REPLAY_LINES):
        l = json.loads(REPLAY_LINES[CUR_REPLAY_INDEX])
        l = _rec_finish_dict(l)
        CUR_REPLAY_INDEX += 1
        return l
    elif IS_REPLAY:
        return None
    try:
        return RECV_QUEUE.get_nowait()
    except Empty:
        return None
