#!/usr/bin/env python
# coding=utf-8
from __future__ import unicode_literals

from .constants import ARRAY_NAMES


def get_error(reference, actual):
    return [(b_i - a_i) for b_i , a_i in zip(reference, actual)]


def get_data_container():
    return {array: [] for array in ARRAY_NAMES}


class Plotter:
    def __init__(self):
        pass
