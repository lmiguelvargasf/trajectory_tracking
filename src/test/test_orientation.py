#!/usr/bin/env python
import unittest
from math import pi

from util.angle import get_angle_between_0_and_2_pi


class OrientationTest(unittest.TestCase):

    def setUp(self):
        self.delta = 0.000001

    def test_when_angle_is_between_0_and_2_pi_then_angle_is_returned(self):
        angle = 50 * pi / 180.0
        self.assertEqual(angle, get_angle_between_0_and_2_pi(angle))

    def test_when_angle_is_negative_then_a_positive_angle_is_returned(self):
        angle = -pi / 2
        expected_angle = 3 * pi / 2
        self.assertEqual(expected_angle, get_angle_between_0_and_2_pi(angle))

    def test_when_angle_is_2_pi_then_0_is_returned(self):
        angle = 2 * pi
        expected_angle = 0
        self.assertEqual(expected_angle, get_angle_between_0_and_2_pi(angle))

    def test_when_angle_is_greater_than_2_pi_then_an_angle_between_0_and_2_pi_is_returned(self):
        angle = 370.5 * pi / 180
        expected_angle = 10.5 * pi / 180
        self.assertAlmostEqual(expected_angle, get_angle_between_0_and_2_pi(angle), delta=self.delta)

    def test_when_angle_is_less_than_minus_2_pi_then_an_angle_between_0_and_2_pi_is_returned(self):
        angle = -370 * pi / 180
        expected_angle = 350 * pi / 180
        self.assertAlmostEqual(expected_angle, get_angle_between_0_and_2_pi(angle), delta=self.delta)
