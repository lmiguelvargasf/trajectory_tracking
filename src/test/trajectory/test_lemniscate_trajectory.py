#!/usr/bin/env python
import unittest

from geometry_msgs.msg import Point

from trajectory.lemniscate_trajectory import LemniscateTrajectory


class LemniscateTrajectoryTest(unittest.TestCase):

    def setUp(self):
        self.trajectory = LemniscateTrajectory(5, 4)
        self.expected_position = Point()

    def test_when_getting_trajectory_name_then_lemniscate_is_returned(self):
        self.assertEqual('lemniscate', self.trajectory.get_name())

    def test_given_lemniscate_trajectory_when_getting_position_after_0s_then_position_is_returned(self):
        self.expected_position.x = 2.0
        self.expected_position.y = 0.0
        self.assertPositionAlmostEqual(self.expected_position, self.trajectory.get_position_at(0), 0.01)

    def test_given_lemniscate_trajectory_when_getting_position_after_1s_then_position_is_returned(self):
        self.expected_position.x = 0
        self.expected_position.y = 0
        self.assertPositionAlmostEqual(self.expected_position, self.trajectory.get_position_at(1), 0.01)


    def test_given_lemniscate_trajectory_when_getting_position_after_2s_then_position_is_returned(self):
        self.expected_position.x = -2.0
        self.expected_position.y = 0
        self.assertPositionAlmostEqual(self.expected_position, self.trajectory.get_position_at(2), 0.01)

    def test_given_lemniscate_trajectory_when_getting_position_after_3s_then_position_is_returned(self):
        self.expected_position.x = 0
        self.expected_position.y = 0
        self.assertPositionAlmostEqual(self.expected_position, self.trajectory.get_position_at(3), 0.01)

    def test_given_lemniscate_trajectory_when_getting_position_after_4s_then_position_is_returned(self):
        self.expected_position.x = 2.0
        self.expected_position.y = 0
        self.assertPositionAlmostEqual(self.expected_position, self.trajectory.get_position_at(4), 0.01)

    def assertPositionAlmostEqual(self, expected, actual, delta):
        self.assertAlmostEqual(expected.x, actual.x, delta=delta)
        self.assertAlmostEqual(expected.y, actual.y, delta=delta)
