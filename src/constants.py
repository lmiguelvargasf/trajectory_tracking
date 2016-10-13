#!/usr/bin/env python
TRAJECTORY = 'circular'
CONTROLLER = 'pid'

if TRAJECTORY == 'linear':
    SIMULATION_TIME_IN_SECONDS = 60.0
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

# PID control constants
K_P_V = 0.2
K_I_V = 1.905
K_D_V = 0.00

K_P_W = 0.45
K_I_W = 1.25
K_D_W = 0.000
