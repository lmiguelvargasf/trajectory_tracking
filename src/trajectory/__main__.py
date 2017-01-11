#!/usr/bin/env python
import os
import sys

from .util import print_error_message, print_usage
from .util import print_files_in_path, get_trajectories_in_path

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
