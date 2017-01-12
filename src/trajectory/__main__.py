#!/usr/bin/env python
import os
import sys

import matplotlib.pylab as plt

from .builder import create_trajectory
from .util import print_error_message, print_usage
from .util import print_files_in_path, get_trajectories_in_path


def plot_trajectory(name):
    STEPS = 600
    DELTA = 1 if name != 'linear' else 0.1
    trajectory = create_trajectory(name, STEPS)

    x = [trajectory.get_position_at(i * DELTA).x for i in range(STEPS)]
    y = [trajectory.get_position_at(i * DELTA).y for i in range(STEPS)]

    trajectory_fig, trajectory_plot = plt.subplots(1, 1)
    trajectory_plot.plot(x, y, label='trajectory', lw=3)
    trajectory_plot.set_title(name.title() + ' Trajectory', fontsize=20)
    trajectory_plot.set_xlabel(r'$x{\rm[m]}$', fontsize=18)
    trajectory_plot.set_ylabel(r'$y{\rm[m]}$', fontsize=18)
    trajectory_plot.legend(loc=0)
    trajectory_plot.grid()
    plt.show()


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print_error_message()
        sys.exit(1)

    arg = sys.argv[1]

    if arg == '--help':
        print_usage()
        sys.exit(0)

    path = os.sep.join(__file__.split(os.sep)[:-1])

    if arg == '--list':
        print_files_in_path(path)
        sys.exit(0)

    if arg not in get_trajectories_in_path(path):
        print('Error: trajectory does not exist!')
        sys.exit(2)

    plot_trajectory(arg)
