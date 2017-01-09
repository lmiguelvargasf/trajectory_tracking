#!/usr/bin/env python
import os
import sys

from context_manager.db_context_manager import DBContextManager
from .printer import print_error_message, print_usage_message
from .constants import QUERIES, ARRAY_NAMES
from .plotter import get_data_container
from .simulation_plotter import SimulationPlotter


def get_filled_data_container(sim_name, table):
    data_container = get_data_container(str(sim_name).split('_')[0])
    number_of_columns = len(ARRAY_NAMES) - 1
    for row in table:
        for i in range(number_of_columns):
            data_container[ARRAY_NAMES[i]].append(row[i])
        data_container[ARRAY_NAMES[number_of_columns]].append(0)
    return data_container


def plot_last_sim_results(path_to_db):
    with DBContextManager(path_to_db) as cursor:
        cursor.execute(QUERIES['select_sim'])
        print("Plotting results of the last simulation...")
        simulation_name = cursor.fetchone()[0]
        cursor.execute(QUERIES['select_data'].format(simulation_name))
        data_container = get_filled_data_container(simulation_name, cursor.fetchall())
        SimulationPlotter(data_container).plot_results()


def print_sim_names(path_to_db):
    with DBContextManager(path_to_db) as cursor:
        cursor.execute(QUERIES['select_sim'])
        for row in cursor.fetchall():
            print(row[0])


def get_sim_names(path_to_db):
    with DBContextManager(path_to_db) as cursor:
        cursor.execute(QUERIES['select_sim'])
        simulations = [row[0] for row in cursor.fetchall()]
    return simulations


def plot_specific_simulation(path_to_db, sim_name):
    with DBContextManager(path_to_db) as cursor:
        cursor.execute(QUERIES['select_data'].format(sim_name))
        data_container = get_filled_data_container(sim_name, cursor.fetchall())
        SimulationPlotter(data_container).plot_results()


if __name__ == '__main__':
    parameters = sys.argv[1:]

    if len(parameters) not in (1, 2, 3, 4):
        print_error_message()
        sys.exit(1)

    if len(parameters) == 1 and parameters[0] == '--help':
        print_usage_message()
        sys.exit(0)

    path_to_database = parameters[0]

    if not os.path.isfile(path_to_database):
        print('Error: database does not exist!')
        sys.exit(2)

    if len(parameters) == 1:
        plot_last_sim_results(path_to_database)
        sys.exit(0)

    if len(parameters) == 2 and parameters[1] == '--sims':
        print_sim_names(path_to_database)
        sys.exit(0)
    else:
        simulation_name = parameters[1]
        if simulation_name not in get_sim_names(path_to_database):
            print('Error: simulation does not exists')
            sys.exit(3)

        print("Plotting the results of the simulation: {}...".format(simulation_name))
        plot_specific_simulation(path_to_database, simulation_name)
