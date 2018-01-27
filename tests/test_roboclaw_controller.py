#!/usr/bin/env python
from __future__ import print_function
import sys
import unittest

# import rospy
import rostest

from ros_roboclaw.roboclaw_stub import RoboclawStub
from ros_roboclaw.roboclaw_control import RoboclawControl

PKG = 'roboclaw_driver'
NAME = 'roboclaw_controller_unittest'


class TestRoboclawController(unittest.TestCase):

    def __init__(self, *args):
        super(TestRoboclawController, self).__init__(*args)
        self.roboclaw = RoboclawStub('/dev/ttyACM0', 115200)
        self.rbc_ctl = RoboclawControl(self.roboclaw)

    # Normal tests
    def test_fowardbackward(self):
        tests = [
            (1.0, 127),
            (-1.0, 0),
            (0, 64),
            (0.5, 96),
            (-0.5, 32)
        ]
        for input_pct, expected_val in tests:
            actual_val = self.rbc_ctl.forward_backward(input_pct)
            self.assertEqual(actual_val, expected_val)
            print("Success -- input: {}, val: {}".format(input_pct, actual_val))

    def test_leftright(self):
        tests = [
            (1.0, 127),
            (-1.0, 0),
            (0, 64),
            (0.5, 96),
            (-0.5, 32)
        ]
        for input_pct, expected_val in tests:
            actual_val = self.rbc_ctl.left_right(input_pct)
            self.assertEqual(actual_val, expected_val)
            print("Success -- input: {}, val: {}".format(input_pct, actual_val))

    # Edge cases / likely mistakes
    def test_fowardbackward_edge(self):
        tests = [
            (100, 127),
            (9999, 127),
            (-100, 0),
            (-9999, 0),
            (0.01, 64),
            (-0.01, 64),
        ]
        for input_pct, expected_val in tests:
            actual_val = self.rbc_ctl.forward_backward(input_pct)
            self.assertEqual(actual_val, expected_val)
            print("Success -- input: {}, val: {}".format(input_pct, actual_val))

    def test_leftright_edge(self):
        tests = [
            (100, 127),
            (9999, 127),
            (-100, 0),
            (-9999, 0),
            (0.01, 64),
            (-0.01, 64),
        ]
        for input_pct, expected_val in tests:
            actual_val = self.rbc_ctl.left_right(input_pct)
            self.assertEqual(actual_val, expected_val)
            print("Success -- input: {}, val: {}".format(input_pct, actual_val))

    # Error cases
    def test_fowardbackward_error(self):
        tests = ['A', "100%", "0.5", "-0.5"]
        for input_pct in tests:
            try:
                actual_val = self.rbc_ctl.forward_backward(input_pct)
                self.fail(
                    msg="Method should have thrown an exeption for input: {} (returned: {})"
                    .format(input_pct, actual_val)
                )
            except Exception:
                print("Success -- Exception was thrown for input: {}".format(input_pct))

    def test_leftright_error(self):
        tests = ['A', "100%", "0.5", "-0.5"]
        for input_pct in tests:
            try:
                actual_val = self.rbc_ctl.left_right(input_pct)
                self.fail(
                    msg="Method should have thrown an exeption for input: {} (returned: {})"
                    .format(input_pct, actual_val)
                )
            except Exception:
                print("Success -- Exception was thrown for input: {}".format(input_pct))


if __name__ == "__main__":
    rostest.rosrun(PKG, NAME, TestRoboclawController, sys.argv)
