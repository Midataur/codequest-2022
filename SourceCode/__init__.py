__version__ = "1.0.0"

from codequest22.client import run_client
import argparse, os
from multiprocessing import Process, Queue

parser = argparse.ArgumentParser(description="Run the CodeQuest 2022 simulation.")
parser.add_argument(
    "players",
    type=str,
    nargs="+",
    help="Path to bot scripts for simulating, space separated. If this is 'replay', then it will attempt to play from a replay file instead.",
)
parser.add_argument(
    "--map",
    "-m",
    type=str,
    default="small.json",
    help="The map to simulate on.",
)
parser.add_argument(
    "--no-visual",
    "-nv",
    action="store_false",
    dest="visual",
    help="Whether to open a visual process",
)
parser.add_argument(
    "--replay",
    "-r",
    type=str,
    default="replay.txt",
    help="Path to store or read the replay file.",
)

def main():
    import sys, time
    args = parser.parse_args(sys.argv[1:])
    called_from = os.getcwd()

    if len(args.players) == 1 and args.players[0] == "replay":
        from codequest22.server import start_server_replay as start_server
        is_replay = True
    else:
        from codequest22.server import start_server as start_server
        is_replay = False
        # Resolve bot paths
        full_paths = [None]*len(args.players)
        for i, bot in enumerate(args.players):
            full_paths[i] = os.path.join(called_from, bot)
            if not os.path.isfile(full_paths[i]) and not os.path.isdir(full_paths[i]):
                full_paths[i] = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'client', bot)
            if not os.path.isfile(full_paths[i]) and not os.path.isdir(full_paths[i]):
                raise ValueError(f"Could not find bot file {bot}")

    # Resolve map and replay paths
    map_path = os.path.join(called_from, args.map)
    if not os.path.isfile(map_path):
        map_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'server/maps', args.map)
    if not os.path.isfile(map_path):
        raise ValueError(f"Could not find map file {args.map}")
    replay_path = os.path.join(called_from, args.replay)

    # The server and visual use a queue to communicate.
    # This queue is essentially a replay file from aiarena21, but it allows us to pause/step through execution.
    visual_queue = Queue()
    server_queue = Queue()
    error_queue = Queue()
    client_queues = [] if is_replay else [[Queue(), Queue()] for _ in full_paths]

    server_process = None
    client_processes = []

    def close_everything():
        # Close processes
        for process in [
            server_process,
        ] + client_processes + ([visual_process] if args.visual else []):
            if process is not None:
                process.terminate()
                process.join()
                process.close()
        # Empty queues
        for queue in [
            visual_queue,
            server_queue,
            error_queue,
        ] + [
            q[i]
            for q in client_queues
            for i in range(2)
        ]:
            while not queue.empty():
                queue.get()


    if not is_replay:
        server_process = Process(target=start_server, args=[map_path, replay_path, server_queue, visual_queue, error_queue, client_queues, args.visual])
        client_processes = [
            Process(target=run_client, args=[full_paths[i], *client_queues[i], error_queue])
            for i in range(len(full_paths))
        ]

        visual_process = None
        if args.visual:
            from codequest22.visual import run_visual
            visual_process = Process(target=run_visual, args=[visual_queue, server_queue, error_queue, False, replay_path], daemon=True)

        try:
            server_process.start()
            time.sleep(0.5)
            for p in client_processes:
                p.start()

            if args.visual:
                visual_process.start()

            while True:
                v = error_queue.get()
                if isinstance(v, list) and isinstance(v[0], Exception):
                    from codequest22.server.replay import ErrorManager
                    error_path = replay_path.replace(".txt", ".error_log")
                    ErrorManager.write_error(error_path, v[1])
                    print(f"An error occured. Check {error_path}")
                    close_everything()
                    break                    
                if isinstance(v, str) and v == "visual" and args.visual:
                    close_everything()
                    break
                if isinstance(v, str) and v == "server" and not args.visual:
                    close_everything()
                    break
        except KeyboardInterrupt:
            close_everything()     
        except Exception as e:
            close_everything()
            raise e
    else:
        from codequest22.visual import run_visual
        visual_process = Process(target=run_visual, args=[visual_queue, server_queue, error_queue, True, replay_path], daemon=True)

        try:
            visual_process.start()
            while True:
                v = error_queue.get()
                if isinstance(v, list) and isinstance(v[0], Exception):
                    from codequest22.server.replay import ErrorManager
                    error_path = replay_path.replace(".txt", ".error_log")
                    ErrorManager.write_error(error_path, v[1])
                    print(f"An error occured. Check {error_path}")
                    close_everything()
                    break
                if isinstance(v, str) and v == "visual":
                    close_everything()
                    break
        except KeyboardInterrupt:
            close_everything()     
        except Exception as e:
            from codequest22.server.replay import ErrorManager
            error_path = replay_path.replace(".txt", ".error_log")
            ErrorManager.write_error(error_path, str(e))
            print(f"An error occured. Check {error_path}")
            close_everything()
            raise e