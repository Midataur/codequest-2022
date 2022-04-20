import os

class Worker:
    COST = int(os.environ.get("WORKER_COST", "20"))
    HP = int(os.environ.get("WORKER_HP", "10"))
    SPEED = float(os.environ.get("WORKER_SPEED", "3"))
    TRIPS = int(os.environ.get("WORKER_TRIPS", "3"))
    WORK_RATE = float(os.environ.get("WORK_RATE", "1"))
    ENCUMBERED_RATE = float(os.environ.get("WORKER_ENCUMBERED_RATE", "0.3"))

class Fighter:
    COST = int(os.environ.get("FIGHTER_COST", "40"))
    HP = int(os.environ.get("FIGHTER_HP", "15"))
    SPEED = float(os.environ.get("FIGHTER_SPEED", "2"))
    ATTACK = float(os.environ.get("FIGHTER_ATTACK", "3.5"))
    RANGE = float(os.environ.get("FIGHTER_RANGE", "1.5"))
    NUM_ATTACKS = int(os.environ.get("FIGHTER_NUM_ATTACKS", "2"))
    LIFESPAN = int(os.environ.get("FIGHTER_LIFESPAN", "30"))

class Settler:
    COST = int(os.environ.get("SETTLER_COST", "30"))
    HP = int(os.environ.get("SETTLER_HP", "10"))
    SPEED = float(os.environ.get("SETTLER_SPEED", "1.5"))
    LIFESPAN = int(os.environ.get("SETTLER_LIFESPAN", "40"))
