
from contextlib import contextmanager
import time
import threading
import _thread

class TimeoutException(Exception):
    def __init__(self, msg=''):
        self.msg = msg

@contextmanager
def time_limit(seconds, msg=''):
    timer = threading.Timer(seconds, lambda: _thread.interrupt_main())
    timer.start()
    try:
        yield
    except KeyboardInterrupt:
        raise TimeoutException(f"Timed out for operation {msg}")
    finally:
        timer.cancel()

class BotRunner:
    """
    Wrapper for running bot commands.
    Allows a time limit to be set, as well as default functionality if the bot crashes or times out.
    """

    FUNCTIONS = [
        # name, and whether it is vital
        ("get_team_name", False),
        ("read_map", False),
        ("read_index", False),
        ("handle_events", False),
        ("handle_failed_requests", False),
    ]

    def __init__(self) -> None:
        self.registry = {}
        self.fallback = {}
        self.fallback["get_team_name"] = lambda : "Timed out"
        self.fallback["read_map"] = lambda _m, _m2: None
        self.fallback["read_index"] = lambda _i, _n: None
        self.fallback["handle_events"] = lambda _e: []
        self.fallback["handle_failed_requests"] = lambda _r: None

    def load_bot(self, bot_path):
        # Restart
        self.running = True
        self.over_elapsed = 0
        import importlib.util

        import os.path
        if os.path.isdir(bot_path):
            spec = importlib.util.spec_from_file_location("", os.path.join(bot_path, "main.py"))
            module = importlib.util.module_from_spec(spec)
            import sys
            sys.path.append(bot_path)
            spec.loader.exec_module(module)
        else:
            spec = importlib.util.spec_from_file_location("", bot_path)
            module = importlib.util.module_from_spec(spec)
            import sys
            sys.path.append(os.path.dirname(bot_path))
            spec.loader.exec_module(module)

        for key, vital in self.FUNCTIONS:
            if hasattr(module, key):
                self.registry[key] = getattr(module, key)
            elif vital:
                # We don't have a fallback
                raise ValueError(f"Bot {bot_path} must implement method {key}")
            else:
                print(f"Bot {bot_path} really should implement method {key}. We've provided a default.")

    def run_command(self, function_name, limit, *args, **kwargs):
        if not self.running:
            return self.fallback[function_name](*args, **kwargs)
        t0 = time.time()
        # Allow 3 times as much time to run.
        try:
            with time_limit(2 * limit, function_name):
                res = self.registry.get(function_name, self.fallback[function_name])(*args, **kwargs)
        except Exception as e:
            # If 2 or more times exceeded, assume failed.
            self.running = False
            res = self.run_command(function_name, limit, *args, **kwargs)
            if isinstance(e, TimeoutError):
                print("Bot spent longer than 2 times the time limit.")
            else:
                import traceback as tb

                error = "".join(tb.format_exception(None, e, e.__traceback__))
                print(f"Bot encountered Exception:\n{error}")
        else:
            elapsed = time.time() - t0
            if elapsed > limit:
                # If you spend 2 more seconds than you should be, then you die.
                self.over_elapsed += elapsed - limit
                if self.over_elapsed > 2:
                    self.running = False
                print("Bot spent more than 2 seconds total over limit.")
        return res
