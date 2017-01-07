#!/usr/bin/env python
# coding=utf-8
import matplotlib.pyplot as plt

from .constants import TITLES, LABELS, PLOT
from .plotter import Plotter, get_error


class SimulationPlotter(Plotter):
    def __init__(self, plot_data, controller_name):
        self.controller = controller_name
        self.data = plot_data

        self.fig_part_0, self.plots_part_0 = plt.subplots(2, 2, sharex=True)
        self.fig_part_1, self.plots_part_1 = plt.subplots(2, 2, sharex=True)
        self.fig_part_2, self.plots_part_2 = plt.subplots(1, 2)

    def plot_results(self):
        e_x = get_error(self.data['x_ref'], self.data['x'])
        e_y = get_error(self.data['y_ref'], self.data['y'])

        self.plots_part_0[0, 0].plot(
            self.data['t'], self.data['x_ref'],
            'r--', label=r'$x_{ref}$', lw=PLOT['line_width'])

        self.plots_part_0[0, 0].plot(
            self.data['t'], self.data['x'],
            'b', label=r'$x$')

        self.plots_part_0[0, 1].plot(
            self.data['t'], self.data['zeros'],
            'r--', label=r'$e=0$', lw=PLOT['line_width'])

        self.plots_part_0[0, 1].plot(
            self.data['t'], e_x, 'b', label=r'$x_{error}$')

        self.plots_part_0[1, 0].plot(
            self.data['t'], self.data['y_ref'],
            'r--', label=r'$y_{ref}$', lw=PLOT['line_width'])

        self.plots_part_0[1, 0].plot(
            self.data['t'], self.data['y'],
            'b', label=r'$y$')

        self.plots_part_0[1, 1].plot(
            self.data['t'], self.data['zeros'],
            'r--', label=r'$e=0$', lw=PLOT['line_width'])

        self.plots_part_0[1, 1].plot(
            self.data['t'], e_y, 'b', label=r'$y_{error}$')

        e_theta = get_error(self.data['theta_ref'], self.data['theta'])
        self.plots_part_1[0, 0].plot(
            self.data['t'], self.data['theta_ref'],
            'r--', label=r'$\theta_{ref}$', lw=PLOT['line_width'])

        self.plots_part_1[0, 0].plot(
            self.data['t'], self.data['theta'],
            'b', label=r'$\theta$')

        self.plots_part_1[1, 0].plot(
            self.data['t'], self.data['zeros'],
            'r--', label=r'$e=0$', lw=PLOT['line_width'])

        self.plots_part_1[1, 0].plot(
            self.data['t'], e_theta, 'b', label=r'$\theta_{error}$')

        plt.figure(self.fig_part_1.number)
        trajectory_plot = plt.subplot(122)
        trajectory_plot.plot(
            self.data['x_ref'], self.data['y_ref'],
            'r--', label=r'${\rm reference}$', lw=PLOT['line_width'])

        trajectory_plot.plot(
            self.data['x'], self.data['y'],
            'b', label=r'${\rm followed}$')

        self.plots_part_2[0].plot(
            self.data['t'], self.data['v_c'],
            'b', label=r'$v_{c}$')

        self.plots_part_2[1].plot(
            self.data['t'], self.data['w_c'],
            'b', label=r'$\omega_{c}$')

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
        if self.controller == 'euler':
            title = r'${\rm Euler\ method\ controller}\ $'
        elif self.controller == 'pid':
            title = r'${\rm PID\ controller}\ $'

        self.fig_part_0.suptitle(title + TITLES['x_n_y'], fontsize=PLOT['fig_title_size'])
        self.fig_part_1.suptitle(title + TITLES['theta_n_trajectory'], fontsize=PLOT['fig_title_size'])
        self.fig_part_2.suptitle(title + TITLES['v_n_w'], fontsize=PLOT['fig_title_size'])

        plt.show()
