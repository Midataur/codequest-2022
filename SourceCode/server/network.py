import json
import progressbar
from codequest22.server.replay import ReplayManager
import codequest22.stats as stats

class CustomEncoder(json.JSONEncoder):

    def default(self, o):
        if hasattr(o, "to_json"):
            return o.to_json()
        return super().default(o)

class NetworkManager:

    RECV, SEND = range(2)

    @classmethod
    def set_queues(cls, visual_queue, recv_queue, queues, is_visual):
        cls.visual_queue = visual_queue
        cls.recv_queue = recv_queue
        cls.queues = queues
        cls.is_visual = is_visual
        cls.bar = progressbar.ProgressBar(widgets=[progressbar.Counter(format=f"%(value)d/{stats.general.SIMULATION_TICKS}"), progressbar.Bar()], max_value=stats.general.SIMULATION_TICKS).start()
        cls.tick = 0

    @classmethod
    def send_internal_obj(cls, obj, visual=True, replay=True):
        if visual:
            cls.visual_queue.put(obj)
        if replay:
            ReplayManager.write_line(json.dumps(obj, cls=CustomEncoder))
        if obj["type"] == "tick":
            cls.tick += 1
            cls.bar.update(cls.tick)
        elif obj["type"] == "winner":
            cls.bar.finish()

    @classmethod
    def wait_visual_response(cls):
        if not cls.is_visual: return
        res = cls.recv_queue.get()
        if res == "Resume":
            return
        else:
            raise ValueError("Visual runner exited.")

    @classmethod
    def broadcast_obj(cls, obj):
        for client in cls.queues:
            client[cls.SEND].put(obj)
    
    @classmethod
    def recv_client_response(cls):
        return [ client[cls.RECV].get() for client in cls.queues]

    @classmethod
    def send_client(cls, index, obj):
        cls.queues[index][cls.SEND].put(obj)
