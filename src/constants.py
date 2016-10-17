#!/usr/bin/env python
TRAJECTORY = 'linear'
CONTROLLER = 'euler'

# control constants
K_X = 0.90
K_Y = 0.90
K_THETA = 0.90

# PID control constants
K_P_V = 0.2
K_I_V = 1.905
K_D_V = 0.00

K_P_W = 0.45
K_I_W = 1.25
K_D_W = 0.000

SIMULATION_TIME_IN_SECONDS = 0.0
if TRAJECTORY == 'linear':
    SIMULATION_TIME_IN_SECONDS = 60.0
    MAX_V = 0.075
    MAX_W = 1.25
elif TRAJECTORY == 'circular':
    SIMULATION_TIME_IN_SECONDS = 120.0
    MAX_V = 0.11
    MAX_W = 1.25
elif TRAJECTORY == 'squared':
    SIMULATION_TIME_IN_SECONDS = 160.0
    MAX_V = 0.055
    MAX_W = 1.20

DELTA_T = 0.1 # this is the sampling time
STEPS = int(SIMULATION_TIME_IN_SECONDS / DELTA_T)
RESULTS_DIRECTORY = '../txt_results/'
