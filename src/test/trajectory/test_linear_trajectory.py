#!/usr/bin/env python
import unittest

from geometry_msgs.msg import Point

from trajectory.linear_trajectory import LinearTrajectory


class LinearTrajectoryTest(unittest.TestCase):

    def setUp(self):
        self.trajectory = LinearTrajectory(1, 0, 2, 0)
        self.expected_position = Point()

    def test_when_getting_trajectory_name_then_linear_is_returned(self):
        self.assertEqual('linear', self.trajectory.get_name())

    def test_given_trajectory_at_origin_when_getting_position_after_1s_then_position_is_returned(self):
        self.expected_position.x = 1
        self.expected_position.y = 2
        self.assertEqual(self.expected_position, self.trajectory.get_position_at(1))

    def test_given_trajectory_not_at_origin_when_getting_position_after_1s_then_position_is_returned(self):
        self.trajectory.x_0 = 1
        self.trajectory.y_0 = 2
        self.expected_position.x = 2
        self.expected_position.y = 4
        self.assertEqual(self.expected_position, self.trajectory.get_position_at(1))
