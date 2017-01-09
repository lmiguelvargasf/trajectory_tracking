#!/usr/bin/env python
import matplotlib.pyplot as plt

from .constants import COLORS, LABELS, TITLES
from .plotter import Plotter, get_error


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
        self.data_list = data_list

    def plot_comparison(self):
        self.plot_reference(self.trajectory_plot, r'{\rm reference}', self.y_ref, self.x_ref)
        self.plot_reference(self.position_plot[0], 'x', self.x_ref)
        self.plot_reference(self.position_plot[1], 'y', self.y_ref)

        self.plot_zeros(self.position_error_plot[0])
        self.plot_zeros(self.position_error_plot[1])

        for i, data in enumerate(self.data_list):
            x_error = get_error(data['x_ref'], data['x'])
            y_error = get_error(data['y_ref'], data['y'])

            tag = r'{\rm' + self.data_list[i]['controller_name'] + r'}'
            color = COLORS['line_' + str(i)]

            self.plot_actual_data(self.trajectory_plot, tag, data['y'], data['x'], color)
            self.plot_actual_data(self.position_plot[0], tag, data['x'], color=color)
            self.plot_actual_data(self.position_plot[1], tag, data['y'], color=color)

            self.plot_error(self.position_error_plot[0], x_error, tag, color)
            self.plot_error(self.position_error_plot[1], y_error, tag, color)

            self.plot_actual_data(self.control_action_plot[0], tag, data['v_c'], color=color)
            self.plot_actual_data(self.control_action_plot[1], tag, data['w_c'], color=color)

        self.decorate_plot(self.trajectory_plot, TITLES['trajectories'], LABELS['x'], LABELS['y'])

        self.decorate_plot(self.position_plot[0], TITLES['x_vs_t'], y_label=LABELS['x'])
        self.decorate_plot(self.position_plot[1], TITLES['y_vs_t'], LABELS['t'], LABELS['y'])

        self.decorate_plot(self.position_error_plot[0], TITLES['x_error'], y_label=LABELS['x_error'])
        self.decorate_plot(self.position_error_plot[1], TITLES['y_error'], LABELS['t'], LABELS['y_error'])

        self.decorate_plot(self.control_action_plot[0], TITLES['v_vs_t'], y_label=LABELS['v'])
        self.decorate_plot(self.control_action_plot[1], TITLES['w_vs_t'], LABELS['t'], LABELS['w'])
        plt.show()
