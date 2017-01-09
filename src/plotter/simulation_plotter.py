#!/usr/bin/env python
# coding=utf-8
import matplotlib.pyplot as plt

from .constants import TITLES, LABELS, PLOT
from .plotter import get_error, Plotter


class SimulationPlotter(Plotter):
    def __init__(self, data):
        Plotter.__init__(self, data.pop('t'), data.pop('zeros'), )
        self.data = data
        self.fig_part_0, self.plots_part_0 = plt.subplots(2, 2, sharex=True)
        self.fig_part_1, self.plots_part_1 = plt.subplots(2, 2, sharex=True)
        self.fig_part_2, self.plots_part_2 = plt.subplots(1, 2)

    def plot_results(self):
        x_error = get_error(self.data['x_ref'], self.data['x'])
        y_error = get_error(self.data['y_ref'], self.data['y'])
        theta_error = get_error(self.data['theta_ref'], self.data['theta'])

        self.plot_reference(self.plots_part_0[0, 0], 'x', self.data['x_ref'])
        self.plot_reference(self.plots_part_0[1, 0], 'y', self.data['y_ref'])
        self.plot_reference(self.plots_part_1[0, 0], r'\theta', self.data['theta_ref'])

        self.plot_actual_data(self.plots_part_0[0, 0], 'x', self.data['x'])
        self.plot_actual_data(self.plots_part_0[1, 0], 'y', self.data['y'], )
        self.plot_actual_data(self.plots_part_1[0, 0], r'\theta', self.data['theta'])

        self.plot_zeros(self.plots_part_0[0, 1])
        self.plot_zeros(self.plots_part_0[1, 1])
        self.plot_zeros(self.plots_part_1[1, 0])

        self.plot_error(self.plots_part_0[0, 1], x_error, 'x')
        self.plot_error(self.plots_part_0[1, 1], y_error, 'y')
        self.plot_error(self.plots_part_1[1, 0], theta_error, r'\theta')

        plt.figure(self.fig_part_1.number)
        trajectory_plot = plt.subplot(122)
        self.plot_reference(trajectory_plot, r'{\rm reference}', self.data['y_ref'], self.data['x_ref'])
        self.plot_actual_data(trajectory_plot, r'{\rm followed}', self.data['y'], self.data['x'])

        self.plot_actual_data(self.plots_part_2[0], r'v_{c}', self.data['v_c'])
        self.plot_actual_data(self.plots_part_2[1], r'\omega_{c}', self.data['w_c'])

        self.decorate_plot(self.plots_part_0[0, 0], TITLES['x_vs_t'], LABELS['t'], LABELS['x'])
        self.decorate_plot(self.plots_part_0[0, 1], TITLES['x_error'], LABELS['t'], LABELS['x_error'])
        self.decorate_plot(self.plots_part_0[1, 0], TITLES['y_vs_t'], LABELS['t'], LABELS['y'])
        self.decorate_plot(self.plots_part_0[1, 1], TITLES['y_error'], LABELS['t'], LABELS['y_error'])

        self.decorate_plot(self.plots_part_1[0, 0], TITLES['theta_vs_t'], LABELS['t'], LABELS['theta'])
        self.decorate_plot(self.plots_part_1[1, 0], TITLES['theta_error'], LABELS['t'], LABELS['theta_error'])
        self.decorate_plot(trajectory_plot, TITLES['trajectory'], LABELS['x'], LABELS['y'])

        self.decorate_plot(self.plots_part_2[0], TITLES['v_vs_t'], LABELS['t'], LABELS['v'])
        self.decorate_plot(self.plots_part_2[1], TITLES['w_vs_t'], LABELS['t'], LABELS['w'])

        title = ''
        if self.data['controller_name'] == 'euler':
            title = r'${\rm Euler\ method\ controller}\ $'
        elif self.data['controller_name'] == 'pid':
            title = r'${\rm PID\ controller}\ $'

        self.fig_part_0.suptitle(title + TITLES['x_n_y'], fontsize=PLOT['fig_title_size'])
        self.fig_part_1.suptitle(title + TITLES['theta_n_trajectory'], fontsize=PLOT['fig_title_size'])
        self.fig_part_2.suptitle(title + TITLES['v_n_w'], fontsize=PLOT['fig_title_size'])

        plt.show()
