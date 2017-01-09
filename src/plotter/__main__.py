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


def plot_simulation(path_to_db, sim_name):
    with DBContextManager(path_to_db) as cursor:
        cursor.execute(QUERIES['select_data'].format(sim_name))
        data_container = get_filled_data_container(sim_name, cursor.fetchall())
        SimulationPlotter(data_container).plot_results()


def plot_simulation_comparison(path_to_db, sim1, sim2):
    pass


def plot_forced_simulation_comparison(path_to_db, sim1, sim2):
    pass


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

    if len(parameters) == 2:
        if parameters[1] == '--sims':
            print_sim_names(path_to_database)
            sys.exit(0)
        else:
            simulation_name = parameters[1]
            if simulation_name not in get_sim_names(path_to_database):
                print('Error: simulation does not exists!')
                sys.exit(3)

            print("Plotting the results of the simulation: {}...".format(simulation_name))
            plot_simulation(path_to_database, simulation_name)
            sys.exit(0)

    if len(parameters) in (3, 4):
        if len(parameters) == 4 and parameters[3] != '--f':
            print_error_message()
            sys.exit(0)

        provided_sims = parameters[1:3]
        simulations = get_sim_names(path_to_database)

        for sim in provided_sims:
            if sim not in simulations:
                print('Error: simulation "{}" does not exists!'.format(sim))
                sys.exit(3)

        if provided_sims[0] == provided_sims[1]:
            print('Error: simulations must be different!')
            sys.exit(4)


        if len(parameters) == 3:
            print('Plotting simulation comparison...')
            plot_simulation_comparison(path_to_database, *provided_sims)
        else:
            print('Plotting forced simulation comparison...')
            plot_forced_simulation_comparison(path_to_database, *provided_sims)
