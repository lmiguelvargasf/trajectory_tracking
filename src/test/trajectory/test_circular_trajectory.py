#!/usr/bin/env python
import unittest

from geometry_msgs.msg import Point

from trajectory.circular_trajectory import CircularTrajectory


class CircularTrajectoryTest(unittest.TestCase):

    def setUp(self):
        self.trajectory = CircularTrajectory(5, 4)
        self.expected_position = Point()

    def test_when_getting_trajectory_name_then_circular_is_returned(self):
        self.assertEqual('circular', self.trajectory.get_name())

    def test_given_circular_trajectory_centered_at_origin_when_getting_position_then_position_is_returned(self):
        self.expected_position.x = 5
        self.expected_position.y = 0
        self.assertPositionAlmostEqual(self.expected_position, self.trajectory.get_position_at(1), 0.01)

    def test_given_circular_trajectory_not_centered_at_origin_when_getting_position_then_position_is_returned(self):
        self.trajectory.x_0 = 5
        self.trajectory.y_0 = 5
        self.expected_position.x = 10
        self.expected_position.y = 5
        self.assertPositionAlmostEqual(self.expected_position, self.trajectory.get_position_at(1), 0.01)

    def assertPositionAlmostEqual(self, expected, actual, delta):
        self.assertAlmostEqual(expected.x, actual.x, delta=delta)
        self.assertAlmostEqual(expected.y, actual.y, delta=delta)

