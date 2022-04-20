import arcade
from codequest22.server.ant import AntTypes
from codequest22.server.events import *
from codequest22.server.summarise_game import GameSummariser

from codequest22.visual.game_input import get_input_maybe, send, use_queue, get_input_wait, use_replay, is_replay

class CleanExit(Exception):
    pass

class GameStateHandler:
    
    MODE_PLAY = "play"
    MODE_STOPPED = "stopped"
    CURRENT_MODE = MODE_PLAY
    RESUME_TIME = 1/10
    SPEED_MODE = "NORMAL"
    SCREEN_RES = (1280, 800)

    last_message = 0
    waiting_message = False
    c_time = 0

    @classmethod
    def get_wait_time(cls):
        if cls.SPEED_MODE == "NORMAL":
            return cls.RESUME_TIME
        elif cls.SPEED_MODE == "FAST":
            return 1/3 * cls.RESUME_TIME
        elif cls.SPEED_MODE == "SLOW":
            return 3 * cls.RESUME_TIME
        elif cls.SPEED_MODE == "SUPER_FAST":
            return 1/10 * cls.RESUME_TIME
        elif cls.SPEED_MODE == "SUPER_SLOW":
            return 10 * cls.RESUME_TIME
        raise ValueError(f"SPEED_MODE is invalid: {cls.SPEED_MODE}")

    @classmethod
    def init(cls):
        from codequest22.visual.screen import ScreenManager

        cls.start_payload = get_input_wait()
        assert cls.start_payload["type"] == "map", "Wrong payload encountered in game."
        cls.map_defn = cls.start_payload["obj"]
        cls.zones = cls.start_payload["zones"]
        cls.energy_info = cls.start_payload["energy_info"]
        cls.active_zones = [False for _ in cls.zones]
        cls.active_tiles = [False for _ in cls.energy_info]
        cls.sm = ScreenManager(*cls.SCREEN_RES)
        cls.sm.initMap(cls.map_defn)
        cls.cur_tick = 0
        cls.hill_snapshots = []

    @classmethod
    def tick(cls, dt):
        cls.c_time += dt
        if not (is_replay() and cls.c_time - cls.last_message <= cls.get_wait_time()):
            # If we are in a replay, we need to do the waiting ourselves.
            res = get_input_maybe()
            if res is not None:
                if res["type"] == "finish":
                    raise CleanExit()
                elif res["type"] == "player_data":
                    # Get names and images
                    cls.sm.names = res["names"]
                    # Load images as sprites
                    cls.sm.initPlayers(res["images"])
                    cls.energy = res["energy"]
                    cls.hill = res["hill"]
                    cls.health = res["health"]
                elif res["type"] == "tick":
                    cls.cur_tick += 1
                    cls.hill_snapshots.append(cls.hill[::])
                    # Get player information
                    """for i in range(len(cls.sm.names)):
                        cls.sm.movePlayer(i, res["positions"][i])"""
                    for ev in res["events"]:
                        if isinstance(ev, SpawnEvent):
                            ant_obj = AntTypes.get_class(ev.ant_type)(ev.player_index, ev.ant_id, ev.position, ev.cost, ev.color)
                            ant_obj.goal = ev.goal
                            ant_obj.path = ev.path
                            cls.sm.spawnAnt(ant_obj)
                            cls.energy[ev.player_index] -= ev.cost
                        elif isinstance(ev, MoveEvent):
                            cls.sm.moveAnt(ev.player_index, ev.ant_id, ev.position, ev.path)
                        elif isinstance(ev, DieEvent):
                            cls.sm.killAnimation(ev.player_index, ev.ant_id, ev.old_age)
                            cls.sm.removeAnt(ev.player_index, ev.ant_id)
                        elif isinstance(ev, ProductionEvent):
                            cls.sm.collectEnergyAnimation(ev.player_index, ev.ant_id)
                        elif isinstance(ev, DepositEvent):
                            cls.energy[ev.player_index] = ev.total_energy
                            cls.sm.playDepositSound()
                        elif isinstance(ev, ZoneActiveEvent):
                            cls.active_zones[ev.zone_index] = True
                            cls.sm.playZoneActiveSound()
                        elif isinstance(ev, ZoneDeactivateEvent):
                            cls.active_zones[ev.zone_index] = False
                            cls.sm.playZoneDeactiveSound()
                        elif isinstance(ev, FoodTileActiveEvent):
                            for i2, t in enumerate(cls.energy_info):
                                if t.position[::-1] == ev.pos:
                                    cls.active_tiles[i2] = True
                            cls.sm.playEnergyActiveSound()
                        elif isinstance(ev, FoodTileDeactivateEvent):
                            for i2, t in enumerate(cls.energy_info):
                                if t.position[::-1] == ev.pos:
                                    cls.active_tiles[i2] = False
                            cls.sm.playEnergyDeactiveSound()
                        elif isinstance(ev, SettlerScoreEvent):
                            cls.hill[ev.player_index] += ev.score_amount
                        elif isinstance(ev, QueenAttackEvent):
                            cls.health[ev.queen_player_index] = ev.queen_hp
                        elif isinstance(ev, TeamDefeatedEvent):
                            cls.hill[ev.by_index] = ev.new_hill_score
                elif res["type"] == "winner":
                    indicies = res["indicies"]
                    cls.CURRENT_MODE = cls.MODE_STOPPED
                    # Wait for the replay file to be closed, so we can open it.
                    failures = 0
                    while True:
                        try:
                            replay = open(cls.replay_path, "r")
                            replay.close()
                            break
                        except IOError:
                            if failures > 30:
                                raise ValueError("Replay file did not close.")
                            failures += 1
                            import time
                            time.sleep(0.02)
                    cls.summary = GameSummariser.get_results(cls.replay_path)
                    cls.sm.show_winner_ui(indicies)
                    cls.sm.playGameCompleteSound()
                cls.last_message = cls.c_time
                cls.waiting_message = True
        if cls.CURRENT_MODE == cls.MODE_PLAY:
            if cls.waiting_message and cls.c_time - cls.last_message > cls.get_wait_time():
                send("Resume")
                cls.waiting_message = False
        elif cls.CURRENT_MODE == cls.MODE_STOPPED:
            cls.last_message = cls.c_time

    @classmethod
    def toggleMode(cls):
        if cls.CURRENT_MODE == cls.MODE_STOPPED:
            cls.CURRENT_MODE = cls.MODE_PLAY
        else:
            cls.CURRENT_MODE = cls.MODE_STOPPED

    @classmethod
    def stepExecution(cls):
        if cls.CURRENT_MODE == cls.MODE_STOPPED:
            if cls.waiting_message:
                send("Resume")
                cls.waiting_message = False

def run_visual(recv_queue, send_queue, error_queue, is_replay=False, replay_path=""):
    from traceback import format_exc
    try:
        if is_replay:
            use_replay(replay_path)
        else:
            use_queue(recv_queue, send_queue)

        GameStateHandler.init()
        GameStateHandler.replay_path = replay_path
        arcade.run()
        error_queue.put("visual")
    except CleanExit:
        error_queue.put("visual")
    except Exception as e:
        error_queue.put([e, format_exc()])
