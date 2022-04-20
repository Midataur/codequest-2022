import os
GRACE_PERIOD = int(os.environ.get("HILL_GRACE_PERIOD", "100"))
NUM_ACTIVATIONS = int(os.environ.get("HILL_NUM_ACTIVATIONS", "2"))
MIN_ZONE_TIME = int(os.environ.get("HILL_MIN_ZONE_TIME", "60"))
MAX_ZONE_TIME = int(os.environ.get("HILL_MAX_ZONE_TIME", "80"))
MIN_WAIT_TIME = int(os.environ.get("HILL_MIN_WAIT_TIME", "20"))
