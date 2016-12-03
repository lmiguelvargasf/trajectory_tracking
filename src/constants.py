#!/usr/bin/env python
TRAJECTORY = 'astroid'
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
elif TRAJECTORY == 'astroid':
    SIMULATION_TIME_IN_SECONDS = 120.0
    MAX_V = 0.105
    MAX_W = 1.25

DELTA_T = 0.1 # this is the sampling time
STEPS = int(SIMULATION_TIME_IN_SECONDS / DELTA_T)

# directory from where data is imported to or exported from
RESULTS_DIRECTORY = '../txt_results/'
PATH_TO_EXPORT_DATA = RESULTS_DIRECTORY + CONTROLLER + '/' + TRAJECTORY + '/'
PATH_TO_IMPORT_EULER_DATA = RESULTS_DIRECTORY + 'euler' + '/' + TRAJECTORY + '/'
PATH_TO_IMPORT_PID_DATA = RESULTS_DIRECTORY + 'pid' + '/' + TRAJECTORY + '/'
