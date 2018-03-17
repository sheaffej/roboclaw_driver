#!/usr/bin/env python
from __future__ import print_function
import unittest

from roboclaw_driver import RoboclawStub, RoboclawControl

PKG = 'roboclaw_driver'
NAME = 'roboclaw_stub_unittest'


class TestRoboclawStub(unittest.TestCase):

    def __init__(self, *args):
        super(TestRoboclawStub, self).__init__(*args)
        self.roboclaw = None
        self.rbc_ctl = None

    def test_normal_fwd_bwd(self):
        self.roboclaw = RoboclawStub('/dev/ttyACM0', 115200)
        self.rbc_ctl = RoboclawControl(self.roboclaw)

        # Check not moving
        self.roboclaw._simulate()
        self.check_stats(0, 0, 0, 0)

        # Drive forward
        self.rbc_ctl.driveM1M2qpps(1000, 1000, 2)

        self.roboclaw._simulate(sim_secs=0.5)
        self.check_stats(1000, 1000, 500, 500)

        self.roboclaw._simulate(sim_secs=0.5)
        self.check_stats(1000, 1000, 1000, 1000)

        # Stop after reaching distance
        self.roboclaw._simulate(sim_secs=1.0)
        self.roboclaw._simulate()  # need to run simulate once more
        self.check_stats(0, 0, 2000, 2000)

        # Drive backward
        self.rbc_ctl.driveM1M2qpps(-2000, -2000, 2)

        self.roboclaw._simulate(sim_secs=0.5)
        self.check_stats(-2000, -2000, 1000, 1000)

        self.roboclaw._simulate(sim_secs=0.5)
        self.check_stats(-2000, -2000, 0, 0)

        self.roboclaw._simulate(sim_secs=0.5)
        self.check_stats(-2000, -2000, -1000, -1000)

        # Stop after reaching distance
        self.roboclaw._simulate(sim_secs=0.5)
        self.roboclaw._simulate()  # need to run simulate once more
        self.check_stats(0, 0, -2000, -2000)

    def test_normal_left_right(self):
        self.roboclaw = RoboclawStub('/dev/ttyACM0', 115200)
        self.rbc_ctl = RoboclawControl(self.roboclaw)

        # Turn left
        self.rbc_ctl.driveM1M2qpps(1000, -1000, 2)

        self.roboclaw._simulate(sim_secs=0.5)
        self.check_stats(1000, -1000, 500, -500)

        # Check that stopped
        self.roboclaw._simulate(sim_secs=1.5)
        self.roboclaw._simulate()  # need to run simulate once more
        self.check_stats(0, 0, 2000, -2000)

        # Turn right
        self.rbc_ctl.driveM1M2qpps(-2000, 2000, 2)

        self.roboclaw._simulate(sim_secs=0.5)
        self.check_stats(-2000, 2000, 1000, -1000)

        # Check that stopped
        self.roboclaw._simulate(sim_secs=1.5)  # 0.5 seconds
        self.roboclaw._simulate()  # need to run simulate once more
        self.check_stats(0, 0, -2000, 2000)

    def test_normal_stop(self):
        self.roboclaw = RoboclawStub('/dev/ttyACM0', 115200)
        self.rbc_ctl = RoboclawControl(self.roboclaw)

        # Start moving forward
        self.rbc_ctl.driveM1M2qpps(1000, 1000, 5)
        self.roboclaw._simulate(sim_secs=0.5)
        self.check_stats(1000, 1000, 500, 500)

        # Issue stop command
        self.rbc_ctl.stop()
        self.roboclaw._simulate()
        # Stop command was received before simulate() so the loop of simulate should not
        # add any distance
        self.check_stats(0, 0, 500, 500)

    def test_normal_diag(self):
        self.roboclaw = RoboclawStub('/dev/ttyACM0', 115200)
        self.rbc_ctl = RoboclawControl(self.roboclaw)

        # Set up the diag values
        m1_current = 0.1  # Amps
        m2_current = 0.2  # Amps
        temp1 = 28.1  # Celsius
        temp2 = 38.4  # Celsius
        main_volts = 7.3  # Volts
        logic_volts = 4.9  # Volts
        error_bitmap = 0x0040 | 0x2000  # Low logic battery + Temp2 warning

        self.roboclaw._m1_current = int(m1_current * 100)
        self.roboclaw._m2_current = int(m2_current * 100)
        self.roboclaw._temp1 = int(temp1 * 10)
        self.roboclaw._temp2 = int(temp2 * 10)
        self.roboclaw._main_bat_voltage = int(main_volts * 10)
        self.roboclaw._logic_bat_voltage = int(logic_volts * 10)
        self.roboclaw._error_bitmap = error_bitmap

        # Check reading diagnostics
        diag = self.rbc_ctl.read_diag()
        tests = [
            ("M1 current", diag.m1_current, m1_current),
            ("M2 current", diag.m2_current, m2_current),
            ("Temp1", diag.temp1, temp1),
            ("Temp2", diag.temp2, temp2),
            ("Main voltage", diag.main_battery_v, main_volts),
            ("Logic voltage", diag.logic_battery_v, logic_volts),
        ]
        for label, actual_val, expected_val in tests:
            self.assertEqual(
                actual_val, expected_val,
                msg="{} expected: {}, actual: {}".format(label, expected_val, actual_val)
            )

        tests = [
            "Logic batt voltage high error",
            "Temperature2 warning",
        ]
        for test in tests:
            self.assertIn(
                test, diag.error_messages,
                msg="error_message {} not found [error_bitmap: {}]"
                    .format(test, self.roboclaw._error_bitmap)
            )

    def check_stats(self, m1_qpps, m2_qpps, m1_val, m2_val):
        """Check actual stats values against expected values.

        Parameters:
            :param int m1_qpps: Expected motor 1 QPPS value
            :param int m2_qpps: Expected motor 2 QPPS value
            :param int m1_val:  Expected motor 1 encoder value
            :param int m2_val:  Expected motor 2 encoder value
        """
        stats = self.rbc_ctl.read_stats()
        tests = [
            ("M1 QPPS", stats.m1_enc_qpps, m1_qpps),
            ("M2 QPPS", stats.m2_enc_qpps, m2_qpps),
            ("M1 encoder value", stats.m1_enc_val, m1_val),
            ("M1 encoder value", stats.m2_enc_val, m2_val),
        ]
        for label, actual_val, expected_val in tests:
            self.assertEqual(
                round(actual_val), round(expected_val),
                msg="{} expected: {}, actual: {}".format(label, expected_val, actual_val)
            )


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(PKG, NAME, TestRoboclawStub)
