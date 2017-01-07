#!/usr/bin/env python
# coding=utf-8
import matplotlib.pyplot as plt

from .constants import TITLES, LABELS, PLOT, COLORS
from .plotter import get_error


class SimulationPlotter:
    def __init__(self, data, controller_name):
        self.controller = controller_name
        self.data = data

        self.fig_part_0, self.plots_part_0 = plt.subplots(2, 2, sharex=True)
        self.fig_part_1, self.plots_part_1 = plt.subplots(2, 2, sharex=True)
        self.fig_part_2, self.plots_part_2 = plt.subplots(1, 2)

    def _plot_reference(self, plot, ys, tag):
        label = r'$' + tag + r'_{ref}$'
        plot.plot(self.data['t'], ys, COLORS['ref'], label=label, lw=PLOT['line_width'])

    def _plot_zeros(self, plot):
        plot.plot(self.data['t'], self.data['zeros'], COLORS['ref'], label=r'$e=0$', lw=PLOT['line_width'])

    def _plot_actual_data(self, plot, ys, tag):
        label= r'$' + tag + r'$'
        plot.plot(self.data['t'], ys, COLORS['actual'], label=label)

    def _plot_error(self, plot, error, tag):
        label = r'$' + tag + r'_{error}$'
        plot.plot(self.data['t'], error, COLORS['actual'], label=label)

    def plot_results(self):
        x_error = get_error(self.data['x_ref'], self.data['x'])
        y_error = get_error(self.data['y_ref'], self.data['y'])
        theta_error = get_error(self.data['theta_ref'], self.data['theta'])

        self._plot_reference(self.plots_part_0[0, 0], self.data['x_ref'], 'x')
        self._plot_reference(self.plots_part_0[1, 0], self.data['y_ref'], 'y')
        self._plot_reference(self.plots_part_1[0, 0], self.data['theta_ref'], r'\theta')

        self._plot_actual_data(self.plots_part_0[0, 0], self.data['x'], 'x')
        self._plot_actual_data(self.plots_part_0[1, 0], self.data['y'], 'y')
        self._plot_actual_data(self.plots_part_1[0, 0], self.data['theta'], r'\theta')

        self._plot_zeros(self.plots_part_0[0, 1])
        self._plot_zeros(self.plots_part_0[1, 1])
        self._plot_zeros(self.plots_part_1[1, 0])

        self._plot_error(self.plots_part_0[0, 1], x_error, 'x')
        self._plot_error(self.plots_part_0[1, 1], y_error, 'y')
        self._plot_error(self.plots_part_1[1, 0], theta_error, r'\theta')

        plt.figure(self.fig_part_1.number)
        trajectory_plot = plt.subplot(122)
        trajectory_plot.plot(
            self.data['x_ref'], self.data['y_ref'],
            COLORS['ref'], label=r'${\rm reference}$', lw=PLOT['line_width'])

        trajectory_plot.plot(
            self.data['x'], self.data['y'],
            COLORS['actual'], label=r'${\rm followed}$')

        self.plots_part_2[0].plot(
            self.data['t'], self.data['v_c'],
            COLORS['actual'], label=r'$v_{c}$')

        self.plots_part_2[1].plot(
            self.data['t'], self.data['w_c'],
            COLORS['actual'], label=r'$\omega_{c}$')

        self._decorate_plot(self.plots_part_0[0, 0], TITLES['x_vs_t'], LABELS['t'], LABELS['x'])
        self._decorate_plot(self.plots_part_0[0, 1], TITLES['x_error'], LABELS['t'], LABELS['x_error'])
        self._decorate_plot(self.plots_part_0[1, 0], TITLES['y_vs_t'], LABELS['t'], LABELS['y'])
        self._decorate_plot(self.plots_part_0[1, 1], TITLES['y_error'], LABELS['t'], LABELS['y_error'])

        self._decorate_plot(self.plots_part_1[0, 0], TITLES['theta_vs_t'], LABELS['t'], LABELS['theta'])
        self._decorate_plot(self.plots_part_1[1, 0], TITLES['theta_error'], LABELS['t'], LABELS['theta_error'])
        self._decorate_plot(trajectory_plot, TITLES['trajectory'], LABELS['x'], LABELS['y'])

        self._decorate_plot(self.plots_part_2[0], TITLES['v_vs_t'], LABELS['t'], LABELS['v'])
        self._decorate_plot(self.plots_part_2[1], TITLES['w_vs_t'], LABELS['t'], LABELS['w'])

        title = ''
        if self.controller == 'euler':
            title = r'${\rm Euler\ method\ controller}\ $'
        elif self.controller == 'pid':
            title = r'${\rm PID\ controller}\ $'

        self.fig_part_0.suptitle(title + TITLES['x_n_y'], fontsize=PLOT['fig_title_size'])
        self.fig_part_1.suptitle(title + TITLES['theta_n_trajectory'], fontsize=PLOT['fig_title_size'])
        self.fig_part_2.suptitle(title + TITLES['v_n_w'], fontsize=PLOT['fig_title_size'])

        plt.show()

    def _decorate_plot(self, plot, title, x_label, y_label):
        plot.set_title(title, fontsize=PLOT['plot_title_size'])
        plot.set_xlabel(x_label, fontsize=PLOT['axis_label_size'])
        plot.set_ylabel(y_label, fontsize=PLOT['axis_label_size'])
        plot.legend(loc=0)
        plot.grid()
