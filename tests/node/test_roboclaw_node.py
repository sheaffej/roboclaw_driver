#!/usr/bin/env python
from __future__ import print_function
import threading
import unittest

import rospy

from roboclaw_driver.msg import SpeedCommand, Stats

PKG = 'roboclaw_driver'
NAME = 'roboclaw_node_nodetest'

DEFAULT_STATS_TOPIC = "/roboclaw/stats"
DEFAULT_CMD_TOPIC = "/roboclaw/speed_command"


class TestRoboclawNode(unittest.TestCase):

    def __init__(self, *args):
        super(TestRoboclawNode, self).__init__(*args)

        self.lock = threading.RLock()
        self.stats = Stats()

        rospy.init_node("roboclaw_test", log_level=rospy.DEBUG, anonymous=True)

        rospy.Subscriber(
            rospy.get_param("~stats_topic", DEFAULT_STATS_TOPIC),
            Stats,
            self._stats_callback
        )

        self.pub_speed_cmd = rospy.Publisher(
            rospy.get_param("~speed_cmd_topic", DEFAULT_CMD_TOPIC),
            SpeedCommand,
            queue_size=1
        )

        rospy.sleep(1)  # Let subscribers connect

    def test_forward_normal(self):
        rospy.sleep(1)
        with self.lock:
            start_m1_dist = self.stats.m1_enc_val
            start_m2_dist = self.stats.m2_enc_val

        m1_qpps, m2_qpps = 1000, 1000
        max_secs = 2
        qpps_delta = 0
        dist_delta = max(abs(m1_qpps), abs(m2_qpps))/2

        cmd = self._create_speed_command(m1_qpps, m2_qpps, max_secs)
        self.pub_speed_cmd.publish(cmd)

        rospy.sleep(6)
        self._check_stats(
            0, 0, qpps_delta,
            start_m1_dist + (m1_qpps * max_secs),
            start_m2_dist + (m2_qpps * max_secs),
            dist_delta
        )

    def test_reverse_normal(self):
        rospy.sleep(1)
        with self.lock:
            start_m1_dist = self.stats.m1_enc_val
            start_m2_dist = self.stats.m2_enc_val

        m1_qpps, m2_qpps = -2000, -2000
        max_secs = 2
        qpps_delta = 0
        dist_delta = max(abs(m1_qpps), abs(m2_qpps))/2

        cmd = self._create_speed_command(m1_qpps, m2_qpps, max_secs)
        self.pub_speed_cmd.publish(cmd)

        rospy.sleep(6)
        self._check_stats(
            0, 0, qpps_delta,
            start_m1_dist + (m1_qpps * max_secs),
            start_m2_dist + (m2_qpps * max_secs),
            dist_delta
        )

    def test_left_normal(self):
        rospy.sleep(1)
        with self.lock:
            start_m1_dist = self.stats.m1_enc_val
            start_m2_dist = self.stats.m2_enc_val

        m1_qpps, m2_qpps = 1000, -1000
        max_secs = 2
        qpps_delta = 0
        dist_delta = max(abs(m1_qpps), abs(m2_qpps))/2

        cmd = self._create_speed_command(m1_qpps, m2_qpps, max_secs)
        self.pub_speed_cmd.publish(cmd)

        rospy.sleep(6)
        self._check_stats(
            0, 0, qpps_delta,
            start_m1_dist + (m1_qpps * max_secs),
            start_m2_dist + (m2_qpps * max_secs),
            dist_delta
        )

    def test_right_normal(self):
        rospy.sleep(1)
        with self.lock:
            start_m1_dist = self.stats.m1_enc_val
            start_m2_dist = self.stats.m2_enc_val

        m1_qpps, m2_qpps = -1000, 1000
        max_secs = 2
        qpps_delta = 0
        dist_delta = max(abs(m1_qpps), abs(m2_qpps))/2

        cmd = self._create_speed_command(m1_qpps, m2_qpps, max_secs)
        self.pub_speed_cmd.publish(cmd)

        rospy.sleep(6)
        self._check_stats(
            0, 0, qpps_delta,
            start_m1_dist + (m1_qpps * max_secs),
            start_m2_dist + (m2_qpps * max_secs),
            dist_delta
        )

    def _stats_callback(self, cmd):
        with self.lock:
            self.stats = cmd

    def _create_speed_command(self, m1_qpps, m2_qpps, max_secs):
        cmd = SpeedCommand()
        cmd.m1_qpps = m1_qpps
        cmd.m2_qpps = m2_qpps
        cmd.max_secs = max_secs
        return cmd

    def _check_stats(self, m1_qpps, m2_qpps, qpps_delta, m1_val, m2_val, val_delta):
        """Check actual stats values against expected values.
        Uses assertAlmostEqual comparison

        Parameters:
            :param int m1_qpps: Expected motor 1 QPPS value
            :param int m2_qpps: Expected motor 2 QPPS value
            :param int qpps_delta: Allowed difference between expected & actual QPPS
            :param int m1_val:  Expected motor 1 encoder value
            :param int m2_val:  Expected motor 2 encoder value
            :param int val_delta: Allowed difference between expected & actual encoder value
        """
        with self.lock:
            tests = [
                ("M1 QPPS", self.stats.m1_enc_qpps, m1_qpps, qpps_delta),
                ("M2 QPPS", self.stats.m2_enc_qpps, m2_qpps, qpps_delta),
                ("M1 encoder value", self.stats.m1_enc_val, m1_val, val_delta),
                ("M1 encoder value", self.stats.m2_enc_val, m2_val, val_delta),
            ]
            for label, actual_val, expected_val, delta in tests:
                self.assertAlmostEqual(
                    actual_val, expected_val, delta=delta,
                    msg="{} expected/delta: {}/{}, actual: {}".format(
                        label, expected_val, delta, actual_val
                    )
                )


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestRoboclawNode)
