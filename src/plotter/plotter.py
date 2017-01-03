#!/usr/bin/env python
# coding=utf-8
from __future__ import unicode_literals


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
        self.zeros = []

class Plotter:
    def __init__(self):
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
