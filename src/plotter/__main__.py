import os
import sqlite3

import sys

from .simulation_plotter import SimulationPlotter
from .plotter import PlotData
from .constants import QUERIES


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
        controller = str(simulation_name).split('_')[0]

        cursor.execute(QUERIES['select_data'].format(simulation_name))
        plot_data = PlotData()

        for row in cursor.fetchall():
            plot_data.t.append(row[0])
            plot_data.x.append(row[1])
            plot_data.x_ref.append(row[2])
            plot_data.y.append(row[3])
            plot_data.y_ref.append(row[4])
            plot_data.theta.append(row[5])
            plot_data.theta_ref.append(row[6])
            plot_data.v_c.append(row[7])
            plot_data.w_c.append(row[8])
            plot_data.zeros.append(0)

        plotter = SimulationPlotter(plot_data, controller)
        plotter.plot_results()

    cursor.close()
    connection.close()
