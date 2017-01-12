#!/usr/bin/env python
import os


def print_usage():
    print('Usage:')
    print('python -m trajectory trajectory_name')
    print('\tShows a plot of trajectory_name.')
    print('python -m trajectory --list')
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
