#!/usr/bin/env python
# coding=utf-8
import matplotlib.pyplot as plt

from constants import PATH_TO_IMPORT_EULER_DATA
from constants import PATH_TO_IMPORT_PID_DATA
from plotter import Plotter, PlotData, get_error


class PaperPlotter(Plotter):
    def __init__(self, steps):
        Plotter.__init__(self, steps)
        self.LINE_WIDTH = 3
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

    def plot_results(self):
        e_euler_x = get_error(self.euler_plot_data.x_ref, self.euler_plot_data.x)
        e_euler_y = get_error(self.euler_plot_data.y_ref, self.euler_plot_data.y)
        e_euler_theta = get_error(self.euler_plot_data.theta_ref, self.euler_plot_data.theta)

        e_pid_x = get_error(self.pid_plot_data.x_ref, self.pid_plot_data.x)
        e_pid_y = get_error(self.pid_plot_data.y_ref, self.pid_plot_data.y)
        e_pid_theta = get_error(self.pid_plot_data.theta_ref, self.pid_plot_data.theta)

        self.plots_part_0.plot(self.euler_plot_data.x_ref, self.euler_plot_data.y_ref, 'r--', label=r'${\rm reference}$', lw=self.LINE_WIDTH)
        self.plots_part_0.plot(self.euler_plot_data.x, self.euler_plot_data.y, 'g', label=r'${\rm euler}$')
        self.plots_part_0.plot(self.pid_plot_data.x, self.pid_plot_data.y, 'b', label=r'${\rm pid}$')

        self.plots_part_1[0].plot(self.euler_plot_data.t, self.euler_plot_data.x_ref, 'r--', label=r'$x_{ref}$', lw=self.LINE_WIDTH)
        self.plots_part_1[0].plot(self.euler_plot_data.t, self.euler_plot_data.x, 'g', label=r'$x-{\rm euler}$')
        self.plots_part_1[0].plot(self.pid_plot_data.t, self.pid_plot_data.x, 'b', label=r'$x-{\rm pid}$')

        self.plots_part_1[1].plot(self.euler_plot_data.t, self.euler_plot_data.y_ref, 'r--', label=r'$y_{ref}$', lw=self.LINE_WIDTH)
        self.plots_part_1[1].plot(self.euler_plot_data.t, self.euler_plot_data.y, 'g', label=r'$y-{\rm euler}$')
        self.plots_part_1[1].plot(self.pid_plot_data.t, self.pid_plot_data.y, 'b', label=r'$y-{\rm pid}$')

        self.plots_part_1[2].plot(self.euler_plot_data.t, self.euler_plot_data.theta_ref, 'r--', label=r'$\theta_{ref}$', lw=self.LINE_WIDTH)
        self.plots_part_1[2].plot(self.euler_plot_data.t, self.euler_plot_data.theta, 'g', label=r'$\theta-{\rm euler}$')
        self.plots_part_1[2].plot(self.pid_plot_data.t, self.pid_plot_data.theta, 'b', label=r'$\theta-{\rm pid}$')

        self.plots_part_2[0].plot(self.euler_plot_data.t, self.zeros, 'r--', label=r'$e=0$', lw=self.LINE_WIDTH)
        self.plots_part_2[0].plot(self.euler_plot_data.t, e_euler_x, 'g', label=r'$x_{error}-{\rm euler}$')
        self.plots_part_2[0].plot(self.pid_plot_data.t, e_pid_x, 'b', label=r'$x_{error}-{\rm pid}$')

        self.plots_part_2[1].plot(self.euler_plot_data.t, self.zeros, 'r--', label=r'$e=0$', lw=self.LINE_WIDTH)
        self.plots_part_2[1].plot(self.euler_plot_data.t, e_euler_y, 'g', label=r'$y_{error}-{\rm euler}$')
        self.plots_part_2[1].plot(self.pid_plot_data.t, e_pid_y, 'b', label=r'$y_{error}-{\rm pid}$')

        self.plots_part_2[2].plot(self.euler_plot_data.t, self.zeros, 'r--', label=r'$e=0$', lw=self.LINE_WIDTH)
        self.plots_part_2[2].plot(self.euler_plot_data.t, e_euler_theta, 'g', label=r'$\theta_{error}-{\rm euler}$')
        self.plots_part_2[2].plot(self.pid_plot_data.t, e_pid_theta, 'b', label=r'$\theta_{error}-{\rm pid}$')

        self.plots_part_3[0].plot(self.euler_plot_data.t, self.euler_plot_data.v_c, 'b', label=r'$v_{c}-{\rm euler}$')
        self.plots_part_3[0].plot(self.pid_plot_data.t, self.pid_plot_data.v_c, 'g', label=r'$v_{c}-{\rm pid}$')

        self.plots_part_3[1].plot(self.euler_plot_data.t, self.euler_plot_data.w_c, 'b', label=r'$\omega_{c}-{\rm euler}$')
        self.plots_part_3[1].plot(self.pid_plot_data.t, self.pid_plot_data.w_c, 'g', label=r'$\omega_{c}-{\rm pid}$')


        self.decorate_plot(self.plots_part_0, r'${\rm followed\ trajectories\ vs\ reference\ trajectory}$',
                           r'$x[{\rm m}]$', r'$y[{\rm m}]$')

        self.decorate_plot(self.plots_part_1[0], r'$x\ {\rm and}\ x_{ref}\ {\rm vs}\ t$', r'',
                           r'$x[{\rm m}]$')
        self.decorate_plot(self.plots_part_1[1], r'$y\ {\rm and}\ y_{ref}\ {\rm vs}\ t$', r'',
                           r'$y[{\rm m}]$')
        self.decorate_plot(self.plots_part_1[2], r'$\theta\ {\rm and}\ \theta_{ref}\ {\rm vs}\ t$', r'$t[{\rm s}]$',
                           r'$\theta[{\rm rad}]$')

        self.decorate_plot(self.plots_part_2[0], r'$x_{error}\ {\rm vs}\ t$', r'', r'$x_{error}[{\rm m}]$')
        self.decorate_plot(self.plots_part_2[1], r'$y_{error}\ {\rm vs}\ t$', r'', r'$y_{error}[{\rm m}]$')
        self.decorate_plot(self.plots_part_2[2], r'$\theta_{error}\ {\rm vs}\ t$', r'$t[{\rm s}]$', r'$\theta_{error}[{\rm m}]$')

        self.decorate_plot(self.plots_part_3[0], r'$v_{c}\ {\rm vs}\ t$', r'', r'$v_{c}[{\rm m/s}]$')
        self.decorate_plot(self.plots_part_3[1], r'$\omega_{c}\ {\rm vs}\ t$', r'$t[{\rm s}]$', r'$\omega_{c}[{\rm m/s}]$')

        plt.show()

    def import_data(self, path_to_file, a_list):
        with open(path_to_file)  as file:
            for line in file:
                a_list.append(float(line))