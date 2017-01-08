#!/usr/bin/env python
import matplotlib.pyplot as plt

from .plotter import Plotter
from .constants import PLOT


class ComparisonPlotter(Plotter):
    def __init__(self, data_list):
        temp_data = data_list[0]
        Plotter.__init__(self, temp_data['t'], temp_data['zeros'])

        self.trajectory_fig, self.trajectory_plot = plt.subplots(1, 1)
        self.position_fig, self.position_plot = plt.subplots(2, 1, sharex=True)
        self.position_error_fig, self.position_error_plot = plt.subplots(2, 1, sharex=True)
        self.control_action_fig, self.control_action_plot = plt.subplots(2, 1, sharex=True)

        self.x_ref = temp_data['x_ref']
        self.y_ref = temp_data['y_ref']

    def plot_comparison(self):
        self.trajectory_plot.plot(self.x_ref, self.y_ref, 'r--', label=r'${\rm reference}$', lw=PLOT['line_width'])
        plt.show()
