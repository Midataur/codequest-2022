import os
MAX_ANTS_PER_PLAYER = int(os.environ.get("MAX_ANTS_PER_PLAYER", "100"))
SIMULATION_TICKS = int(os.environ.get("SIMULATION_TICKS", "1200"))
MAX_SPAWNS_PER_TICK = int(os.environ.get("MAX_SPAWNS_PER_TICK", "5"))
MAX_ENERGY_STORED = int(os.environ.get("MAX_ENERGY_STORED", "750"))
QUEEN_HEALTH = int(os.environ.get("QUEEN_HEALTH", "3000"))
STARTING_ENERGY = int(os.environ.get("STARTING_ENERGY", "100"))