import os
DELAY = int(os.environ.get("ENERGY_DELAY", "2"))
PER_TICK = int(os.environ.get("ENERGY_PER_TICK", "1"))
GRACE_PERIOD = int(os.environ.get("ENERGY_GRACE_PERIOD", "100"))
MIN_OVERCHARGE_TIME = int(os.environ.get("ENERGY_MIN_OVERCHARGE_TIME", "50"))
MAX_OVERCHARGE_TIME = int(os.environ.get("ENERGY_MAX_OVERCHARGE_TIME", "80"))
MIN_WAIT_TIME = int(os.environ.get("ENERGY_MIN_WAIT_TIME", "50"))
NUM_ACTIVATIONS = int(os.environ.get("ENERGY_NUM_ACTIVATIONS", "3"))