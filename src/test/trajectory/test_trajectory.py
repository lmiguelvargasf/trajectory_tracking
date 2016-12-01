#!/usr/bin/env python
import unittest

from geometry_msgs.msg import Point

from trajectory.squared_trajectory import SquaredTrajectory
from trajectory.trajectory import NegativeTimeException, Trajectory


class TrajectoryTest(unittest.TestCase):

    def test_when_time_is_negative_then_an_exception_is_raised(self):
        trajectory = Trajectory()
        self.assertRaises(NegativeTimeException, trajectory.get_position_at, -1)


class SquaredTrajectoryTest(unittest.TestCase):

    def setUp(self):
        self.trajectory = SquaredTrajectory(3, 4)
        self.expected_position = Point()

    def test_given_squared_trajectory_with_corner_at_the_origin_when_getting_position_then_position_is_returned(self):
        self.expected_position.x = 3
        self.expected_position.y = 3
        self.assertEqual(self.expected_position, self.trajectory.get_position_at(2))

    def test_given_squared_trajectory_with_corner_not_at_the_origin_when_getting_position_then_position_is_returned(self):
        self.trajectory.x_0 = 1
        self.trajectory.y_0 = 1
        self.expected_position.x = 4
        self.expected_position.y = 4
        self.assertEqual(self.expected_position, self.trajectory.get_position_at(2))
