#!/usr/bin/env python
import math

from geometry_msgs.msg import Point


class NegativeTimeException(Exception):
    pass

class Trajectory:
    def __init__(self):
        self.position = Point()

    def get_position_at(self, t):
        if t < 0:
            raise NegativeTimeException()
