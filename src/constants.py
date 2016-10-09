#!/usr/bin/env python
TRAJECTORY_TYPE = 'circular'

if TRAJECTORY_TYPE == 'linear':
    SIMULATION_TIME_IN_SECONDS = 40
elif TRAJECTORY_TYPE == 'circular':
    SIMULATION_TIME_IN_SECONDS = 120

DELTA_T = 0.1 # this is the sampling time
STEPS = int(SIMULATION_TIME_IN_SECONDS / DELTA_T)
K_V = 0.90
K_W = 0.90
