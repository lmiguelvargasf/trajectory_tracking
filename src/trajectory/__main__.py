#!/usr/bin/env python
import os

import sys


def print_usage():
    print('Usage:')
    print('python -m trajectory trajectory_name')
    print('\tShows a plot of trajectory_name.')
    print('python -m trajectory --trajectories')
    print('\tShows the list of trajectories.')


def print_error_message():
    print('Error using module trajectory!')
    print('Try python -m trajectory --help for more information.')


def print_files_in_path(path):
    print('Available trajectories:')
    for trajectory in sorted(get_trajectories_in_path(path)):
            print(trajectory)


def get_trajectories_in_path(path):
    return [file.replace('_trajectory.py', '')
            for file in os.listdir(path)
            if '_trajectory' in file and '.pyc' not in file]


def plot_trajectory():
    pass


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print_error_message()
        sys.exit(1)

    arg = sys.argv[1]

    if arg == '--help':
        print_usage()
        sys.exit(0)

    path = os.sep.join(__file__.split(os.sep)[:-1])

    if arg == '--trajectories':
        print_files_in_path(path)
        sys.exit(0)

    if arg not in get_trajectories_in_path(path):
        print('Error: trajectory does not exist!')
        sys.exit(2)

    plot_trajectory()

