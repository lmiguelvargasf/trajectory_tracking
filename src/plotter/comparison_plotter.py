#!/usr/bin/env python
from plotter.plotter import Plotter


class ComparisonPlotter(Plotter):
    def __init__(self, data_list):
        Plotter.__init__(self)
        self.data_list = data_list
