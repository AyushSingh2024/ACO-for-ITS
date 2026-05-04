"""
ACO-ITS Configuration Settings
"""

# ACO Hyperparameters
ACO_ALPHA = 1.0        # pheromone importance
ACO_BETA = 2.0         # heuristic (1/travel_time) importance
ACO_EVAPORATION = 0.1  # pheromone evaporation rate per timestep
ACO_Q = 100.0          # pheromone deposit constant
ACO_N_ANTS = 20        # ants per optimization cycle
ACO_INIT_PHEROMONE = 1.0

# PSO Hyperparameters
PSO_SWARM_SIZE = 30
PSO_MAX_ITERATIONS = 100
PSO_W = 0.7            # inertia weight
PSO_C1 = 1.5           # cognitive coefficient
PSO_C2 = 1.5           # social coefficient

# Simulation Settings
SIM_BEGIN_TIME = 0
SIM_END_TIME = 3600
SIM_TIME_STEP = 1

# Server Settings
HOST = "0.0.0.0"
PORT = 8000
DEBUG = True

# Data Paths
DATA_DIR = "data/simulation"
OUTPUT_DIR = "data/output"

# Congestion Thresholds
CONGESTION_HIGH = 5
CONGESTION_MEDIUM = 2

# Speed calculations (m/s)
BASE_SPEED = 13.89  # ~50 km/h
CONGESTION_SPEED_PENALTY = 1.5
