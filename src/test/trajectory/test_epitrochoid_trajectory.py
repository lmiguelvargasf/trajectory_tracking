#!/usr/bin/env python
import unittest

from geometry_msgs.msg import Point

from trajectory.epitrochoid_trajectory import EpitrochoidTrajectory


class EpitrochoidTrajectoryTest(unittest.TestCase):

    def setUp(self):
        self.trajectory = EpitrochoidTrajectory(5, 1, 3, 4, 1.0 / 3)
        self.expected_position = Point()

    def test_when_getting_trajectory_name_then_epitrochoid_is_returned(self):
        self.assertEqual('epitrochoid', self.trajectory.get_name())

    def test_given_epitrochoid_trajectory_when_getting_position_after_0s_then_position_is_returned(self):
        self.expected_position.x = 1.0
        self.expected_position.y = 0.0
        self.assertPositionAlmostEqual(self.expected_position, self.trajectory.get_position_at(0), 0.01)

    def test_given_epitrochoid_trajectory_when_getting_position_after_1s_then_position_is_returned(self):
        self.expected_position.x = 1.0
        self.expected_position.y = 2.0
        self.assertPositionAlmostEqual(self.expected_position, self.trajectory.get_position_at(1), 0.01)


    def test_given_epitrochoid_trajectory_when_getting_position_after_2s_then_position_is_returned(self):
        self.expected_position.x = -3.0
        self.expected_position.y = 0.0
        self.assertPositionAlmostEqual(self.expected_position, self.trajectory.get_position_at(2), 0.01)

    def test_given_epitrochoid_trajectory_when_getting_position_after_3s_then_position_is_returned(self):
        self.expected_position.x = 1
        self.expected_position.y = -2
        self.assertPositionAlmostEqual(self.expected_position, self.trajectory.get_position_at(3), 0.01)

    def test_given_epitrochoid_trajectory_when_getting_position_after_4s_then_position_is_returned(self):
        self.expected_position.x = 1.0
        self.expected_position.y = 0.0
        self.assertPositionAlmostEqual(self.expected_position, self.trajectory.get_position_at(4), 0.01)

    def assertPositionAlmostEqual(self, expected, actual, delta):
        self.assertAlmostEqual(expected.x, actual.x, delta=delta)
        self.assertAlmostEqual(expected.y, actual.y, delta=delta)
