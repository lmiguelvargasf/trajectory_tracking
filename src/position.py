#!/usr/bin/env python
from geometry_msgs.msg import Point


class Position:
    @staticmethod
    def get_position_at(t):
        position = Point()
        position.x = 0.05 * t
        position.y = 0.05 * t
        position.z = 0.0

        return position

    @staticmethod
    def get_position_from_pose(pose):
        return pose.position
