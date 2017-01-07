#!/usr/bin/env python
# coding=utf-8
from __future__ import unicode_literals

from .constants import ARRAY_NAMES, PLOT


def get_error(reference, actual):
    return [(b_i - a_i) for b_i , a_i in zip(reference, actual)]


def get_data_container():
    return {array: [] for array in ARRAY_NAMES}


class Plotter:
    def decorate_plot(self, plot, title, x_label, y_label):
        plot.set_title(title, fontsize=PLOT['plot_title_size'])
        plot.set_xlabel(x_label, fontsize=PLOT['axis_label_size'])
        plot.set_ylabel(y_label, fontsize=PLOT['axis_label_size'])
        plot.legend(loc=0)
        plot.grid()

    def plot_results(self):
        pass
