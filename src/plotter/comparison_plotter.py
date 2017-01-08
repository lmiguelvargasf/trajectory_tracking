#!/usr/bin/env python
import matplotlib.pyplot as plt

from .constants import PLOT


class ComparisonPlotter:
    def __init__(self, data_list):
        self.trajectory_fig, self.trajectory_plot = plt.subplots(1, 1)
        self.position_fig, self.position_plot = plt.subplots(2, 1, sharex=True)
        self.position_error_fig, self.position_error_plot = plt.subplots(2, 1, sharex=True)
        self.control_action_fig, self.control_action_plot = plt.subplots(2, 1, sharex=True)

        temp_data = data_list[0]
        self.t = temp_data['t']
        self.x_ref = temp_data['x_ref']
        self.y_ref = temp_data['y_ref']
        self.zeros = temp_data['zeros']

    def plot_comparison(self):
        self.trajectory_plot.plot(self.x_ref, self.y_ref, 'r--', label=r'${\rm reference}$', lw=PLOT['line_width'])
        plt.show()

if __name__ == '__main__':
    steps = 100
    plotter = ComparisonPlotter(
        [
            {'t': [i for i in range(steps)],
             'x_ref': [0.5 * i for i in range(steps)],
             'y_ref': [2.0 * i for i in range(steps)],
             'zeros': [0.0 for _ in range(steps)],}
        ]
    )
    plotter.plot_comparison()
