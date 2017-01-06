import os
import sqlite3

import sys

from .simulation_plotter import SimulationPlotter
from .plotter import get_data_container
from .constants import QUERIES, ARRAY_NAMES


def print_error_message():
    print('Error using module plotter!')
    print('Try python -m plotter --help for more information')


def print_usage():
    print('Usage:')
    print('python -m plotter /path/to/database/results.db [simulation_A] [simulation_B]')
    print('\tPlots the results of the last simulation when both simulation_A and simulation_B are not provided.')
    print('\tPlots the results of simulation_A when just this parameter is provided.')
    print('\tPlots the results of simulation_A and simulation_B when both parameters are provided.')
    print('python -m plotter /path/to/database/results.db --sims')
    print('\tShows the list of simulations.')


def plot_simulation(simulation_name):
    controller = str(simulation_name).split('_')[0]
    number_of_columns = len(ARRAY_NAMES) - 1
    cursor.execute(QUERIES['select_data'].format(simulation_name))
    data_container = get_data_container()
    for row in cursor.fetchall():
        for i in range(number_of_columns):
            data_container[ARRAY_NAMES[i]].append(row[i])
        data_container[ARRAY_NAMES[number_of_columns]].append(0)

    plotter = SimulationPlotter(data_container, controller)
    plotter.plot_results()


if __name__ == '__main__':
    parameters = sys.argv[1:]

    if len(parameters) not in (1, 2, 3):
        print_error_message()
        sys.exit(1)

    if len(parameters) == 1 and parameters[0] == '--help':
        print_usage()
        sys.exit(0)

    path_to_database = parameters[0]

    if not os.path.isfile(path_to_database):
        print('Error: database does not exist!')
        sys.exit(2)

    connection = sqlite3.connect(path_to_database)
    cursor = connection.cursor()
    cursor.execute(QUERIES['select_sim'])

    if len(parameters) == 1:
        print("Plotting results of the last simulation...")
        simulation_name = cursor.fetchone()[0]
        plot_simulation(simulation_name)

    if len(parameters) == 2:
        rows = cursor.fetchall()
        if parameters[1] == '--sims':
            for row in rows:
                print(row[0])
            sys.exit(0)

        simulations = [row[0] for row in rows]
        simulation_name = parameters[1]

        if simulation_name not in simulations:
            print('Error: simulation does not exists')
            sys.exit(3)
        print("Plotting the results of the simulation: {}...".format(simulation_name))
        plot_simulation(simulation_name)

    cursor.close()
    connection.close()
