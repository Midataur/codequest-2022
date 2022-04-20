from codequest22.client.runner import BotRunner
from codequest22.client.network import NetworkManager
from traceback import format_exc
import os.path

def run_client(bot_path, send_queue, recv_queue, error_queue):
    try:
        NetworkManager.set_queues(send_queue, recv_queue)

        runner = BotRunner()
        runner.load_bot(bot_path)

        team_name = runner.run_command("get_team_name", 0.5)
        folder_path = bot_path
        if folder_path.endswith(".py"):
            folder_path = os.path.dirname(folder_path)
        team_image = os.path.join(folder_path, "profile.png")
        if type(team_name) != str:
            print(f"Team name must be string, got {type(team_name)}.")
            team_name = "Error"
        NetworkManager.send_obj({
            "type": "player_data",
            "name": team_name,
            "image": team_image,
        })

        player_obj = NetworkManager.recv_obj()
        player_index = player_obj["index"]
        runner.run_command("read_index", 0.2, player_index, player_obj["total"])

        map_obj = NetworkManager.recv_obj()
        assert map_obj["type"] == "map"
        energy = map_obj["energy_tiles"]
        map_data = map_obj["obj"]
        runner.run_command("read_map", 30, map_data, energy)

        # Sample mainloop
        while True:
            tick_data = NetworkManager.recv_obj()
            if tick_data["type"] == "finish":
                return
            assert tick_data["type"] == "tick"
            runner.run_command("handle_failed_requests", 0.5, [req for req in tick_data["failed_requests"] if req.player_index == player_index])
            requests = runner.run_command("handle_events", 0.5, tick_data["events"])
            """if not position_valid(position, tick_data["positions"][player_index], map_data):
                print(f"Invalid position {position} received.")
                position = tick_data["positions"][player_index]
                runner.running = False"""
            NetworkManager.send_obj(requests)
    except Exception as e:
        error_queue.put([e, format_exc()])

def position_valid(new_pos, old_pos, map):
    return new_pos == old_pos or new_pos in map["edges"][old_pos]
