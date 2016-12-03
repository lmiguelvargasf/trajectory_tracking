#!/usr/bin/env python
import unittest

from geometry_msgs.msg import Point

from trajectory.astroid_trajectory import AstroidTrajectory


class AstroidTrajectoryTest(unittest.TestCase):

    def setUp(self):
        self.delta = 0.000001
        self.radius = 5
        self.period = 4
        self.expected_position = Point()
        self.trajectory = AstroidTrajectory(self.radius, self.period)


    def test_when_creating_trajectory_the_radius_and_period_are_set(self):
        self.assertEqual(self.radius, self.trajectory.radius)
        self.assertEqual(self.period, self.trajectory.period)

    def test_when_getting_position_after_1s_then_position_at_1s_is_returned(self):
        self.expected_position.x = 0
        self.expected_position.y = self.radius
        self.assertAlmostEqual(self.expected_position, self.trajectory.get_position_at(1))

    def test_when_getting_position_after_2s_then_position_at_2s_is_returned(self):
        self.expected_position.x = -self.radius
        self.expected_position.y = 0
        self.assertAlmostEqual(self.expected_position, self.trajectory.get_position_at(2))
