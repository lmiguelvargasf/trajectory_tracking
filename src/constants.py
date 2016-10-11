#!/usr/bin/env python
TRAJECTORY = 'linear'
CONTROLLER = 'trapezoidal'

if TRAJECTORY == 'linear':
    SIMULATION_TIME_IN_SECONDS = 40.0
elif TRAJECTORY == 'circular':
    SIMULATION_TIME_IN_SECONDS = 120.0
elif TRAJECTORY == 'squared':
    SIMULATION_TIME_IN_SECONDS = 160.0

DELTA_T = 0.1 # this is the sampling time
STEPS = int(SIMULATION_TIME_IN_SECONDS / DELTA_T)

# control constants
K_X = 0.90
K_Y = 0.90
K_THETA = 0.90
