#!/usr/bin/env python

def print_error_message():
    print('Error using module plotter!')
    print('Try python -m plotter --help for more information')


def print_usage_message():
    print('Usage:')
    print('python -m plotter /path/to/database/results.db [simulation_A] [simulation_B]')
    print('\tPlots the results of the last simulation when both simulation_A and simulation_B are not provided.')
    print('\tPlots the results of simulation_A when just this parameter is provided.')
    print('\tPlots the results of simulation_A and simulation_B when both parameters are provided.')
    print('python -m plotter /path/to/database/results.db --sims')
    print('\tShows the list of simulations.')
