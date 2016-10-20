#!/usr/bin/env python
# coding=utf-8
from __future__ import unicode_literals

import matplotlib.pyplot as plt
from errno import EEXIST
from os import makedirs
from os.path import exists, dirname

from constants import STEPS, DELTA_T, CONTROLLER, PATH_TO_EXPORT_DATA, PATH_TO_IMPORT_EULER_DATA, \
    PATH_TO_IMPORT_PID_DATA
from trajectory import create_trajectory

def get_error(reference, actual):
    return [(b_i - a_i) for b_i , a_i in zip(reference, actual)]


class PlotData:
    def __init__(self):
        self.t = []
        self.x_ref = []
        self.y_ref = []
        self.x = []
        self.y = []
        self.theta = []
        self.theta_ref = []
        self.v_c = []
        self.w_c = []

        self.file_array_name = {
            't.txt': self.t,
            'x.txt': self.x,
            'x_ref.txt': self.x_ref,
            'y.txt': self.y,
            'y_ref.txt': self.y_ref,
            'theta.txt': self.theta,
            'theta_ref.txt': self.theta_ref,
            'v_c.txt': self.v_c,
            'w_c.txt': self.w_c,
        }


class Plotter:
    def __init__(self):
        self.zeros = [0 for _ in range(STEPS)]

        self.LINE_WIDTH = 2
        self.FIGURE_TITLE_SIZE = 21
        self.PLOT_TITLE_SIZE = 19
        self.PLOT_AXIS_LABEL_SIZE = 17

    def decorate_plot(self, plot, title, x_label, y_label):
        plot.set_title(title, fontsize=self.PLOT_TITLE_SIZE)
        plot.set_xlabel(x_label, fontsize=self.PLOT_AXIS_LABEL_SIZE)
        plot.set_ylabel(y_label, fontsize=self.PLOT_AXIS_LABEL_SIZE)
        plot.legend(loc=0)
        plot.grid()

    def plot_results(self):
        pass

class SimulationPlotter(Plotter):
    def __init__(self):
        Plotter.__init__(self)
        self.plot_data = PlotData()
        trajectory = create_trajectory()
        self.plot_data.t = [i * DELTA_T for i in range(STEPS)]
        self.plot_data.x_ref = [trajectory.get_position_at(i * DELTA_T).x for i in range(STEPS)]
        self.plot_data.y_ref = [trajectory.get_position_at(i * DELTA_T).y for i in range(STEPS)]

        self.fig_part_0, self.plots_part_0 = plt.subplots(2, 2, sharex=True)
        self.fig_part_1, self.plots_part_1 = plt.subplots(2, 2, sharex=True)
        self.fig_part_2, self.plots_part_2 = plt.subplots(1, 2)

    def add_point(self, pose):
        self.plot_data.x.append(pose.position.x)
        self.plot_data.y.append(pose.position.y)

    def plot_results(self):
        e_x = get_error(self.plot_data.x_ref, self.plot_data.x)
        e_y = get_error(self.plot_data.y_ref, self.plot_data.y)
        self.plots_part_0[0, 0].plot(self.plot_data.t, self.plot_data.x_ref, 'r--', label=r'$x_{ref}$', lw=self.LINE_WIDTH)
        self.plots_part_0[0, 0].plot(self.plot_data.t, self.plot_data.x, 'b', label=r'$x$')
        self.plots_part_0[0, 1].plot(self.plot_data.t, self.zeros, 'r--', label=r'$e=0$', lw=self.LINE_WIDTH)
        self.plots_part_0[0, 1].plot(self.plot_data.t, e_x, 'b', label=r'$x_{error}$')
        self.plots_part_0[1, 0].plot(self.plot_data.t, self.plot_data.y_ref, 'r--', label=r'$y_{ref}$', lw=self.LINE_WIDTH)
        self.plots_part_0[1, 0].plot(self.plot_data.t, self.plot_data.y, 'b', label=r'$y$')
        self.plots_part_0[1, 1].plot(self.plot_data.t, self.zeros, 'r--', label=r'$e=0$', lw=self.LINE_WIDTH)
        self.plots_part_0[1, 1].plot(self.plot_data.t, e_y, 'b', label=r'$y_{error}$')

        e_theta = get_error(self.plot_data.theta_ref, self.plot_data.theta)
        self.plots_part_1[0, 0].plot(self.plot_data.t, self.plot_data.theta_ref, 'r--', label=r'$\theta_{ref}$', lw=self.LINE_WIDTH)
        self.plots_part_1[0, 0].plot(self.plot_data.t, self.plot_data.theta, 'b', label=r'$\theta$')
        self.plots_part_1[1, 0].plot(self.plot_data.t, self.zeros, 'r--', label=r'$e=0$', lw=self.LINE_WIDTH)
        self.plots_part_1[1, 0].plot(self.plot_data.t, e_theta, 'b', label=r'$\theta_{error}$')

        plt.figure(self.fig_part_1.number)
        trajectory_plot = plt.subplot(122)
        trajectory_plot.plot(self.plot_data.x_ref, self.plot_data.y_ref, 'r--', label=r'${\rm reference}$', lw=self.LINE_WIDTH)
        trajectory_plot.plot(self.plot_data.x, self.plot_data.y, 'b', label=r'${\rm followed}$')

        self.plots_part_2[0].plot(self.plot_data.t, self.plot_data.v_c, 'b', label=r'$v_{c}$')
        self.plots_part_2[1].plot(self.plot_data.t, self.plot_data.w_c, 'b', label=r'$\omega_{c}$')

        self.decorate_plot(self.plots_part_0[0, 0], r'$x\ {\rm and}\ x_{ref}\ {\rm vs}\ t$', r'$t[{\rm s}]$', r'$x[{\rm m}]$')
        self.decorate_plot(self.plots_part_0[0, 1], r'$x_{error}\ {\rm vs}\ t$', r'$t[{\rm s}]$', r'$x_{error}[{\rm m}]$')
        self.decorate_plot(self.plots_part_0[1, 0], r'$y\ {\rm and}\ y_{ref}\ {\rm vs}\ t$', r'$t[{\rm s}]$', r'$y[{\rm m}]$')
        self.decorate_plot(self.plots_part_0[1, 1], r'$y_{error}\ {\rm vs}\ t$', r'$t[{\rm s}]$', r'$y_{error}[{\rm m}]$')

        self.decorate_plot(self.plots_part_1[0, 0], r'$\theta,\ \theta_{ref}\ {\rm vs}\ t$', r'$t[{\rm s}]$', r'$\theta[{\rm rad}]$')
        self.decorate_plot(self.plots_part_1[1, 0], r'$\theta_{error}\ {\rm vs}\ t$', r'$t[{\rm s}]$', r'$\theta_{error}[{\rm rad}]$')
        self.decorate_plot(trajectory_plot, r'${\rm followed\ trajectory\ vs\ reference\ trajectory}$',
                           r'$x[{\rm m}]$', r'$y[{\rm m}]$')

        self.decorate_plot(self.plots_part_2[0], r'$v_{c}\ {\rm vs}\ t$', r'$t[{\rm s}]$', r'$v_{c}[{\rm m/s}]$')
        self.decorate_plot(self.plots_part_2[1], r'$\omega_{c}\ {\rm vs}\ t$', r'$t[{\rm s}]$', r'$\omega_{c}[{\rm rad/s}]$')

        title = ''
        if CONTROLLER == 'euler':
            title = r'${\rm Euler\ method\ controller}\ $'
        elif CONTROLLER == 'pid':
            title = r'${\rm PID\ controller}\ $'

        self.fig_part_0.suptitle(title + r'${\rm results - }\ x\ {\rm and}\ y$', fontsize=self.FIGURE_TITLE_SIZE)
        self.fig_part_2.suptitle(title + r'${\rm results - }\ v_{c}\ {\rm and}\ \omega_{c}$',
                                 fontsize=self.FIGURE_TITLE_SIZE)
        self.fig_part_1.suptitle(title + r'${\rm results - }\ \theta\ {\rm and\ trajectory}$',
                                 fontsize=self.FIGURE_TITLE_SIZE)

        plt.show()

    def export_results(self):
        for file_name, a_list in self.plot_data.file_array_name.items():
            self.export_list(PATH_TO_EXPORT_DATA + file_name, a_list)

    def export_list(self, path_to_file, a_list):
        if not exists(dirname(path_to_file)):
            try:
                makedirs(dirname(path_to_file))
            except OSError as exc:  # Guard against race condition
                if exc.errno != EEXIST:
                    raise

        with open(path_to_file, 'w') as file:
            for e in a_list:
                file.write(str(e) + '\n')


class PaperPlotter(Plotter):
    def __init__(self):
        Plotter.__init__(self)
        self.euler_plot_data = PlotData()
        self.pid_plot_data = PlotData()

        for file_name, a_list in self.euler_plot_data.file_array_name.items():
            self.import_data(PATH_TO_IMPORT_EULER_DATA + file_name, a_list)

        for file_name, a_list in self.pid_plot_data.file_array_name.items():
            self.import_data(PATH_TO_IMPORT_PID_DATA + file_name, a_list)

        self.fig_part_0, self.plots_part_0 = plt.subplots(1, 1)
        self.fig_part_1, self.plots_part_1 = plt.subplots(3, 1, sharex=True)
        self.fig_part_2, self.plots_part_2 = plt.subplots(3, 1, sharex=True)
        self.fig_part_3, self.plots_part_3 = plt.subplots(2, 1, sharex=True)

