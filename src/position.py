#!/usr/bin/env python
from geometry_msgs.msg import Point


class Position:
    def get_position_at(self, t):
        position = Point()
        position.x = 0.05 * t
        position.y = 0.05 * t
        position.z = 0.0

        return position
