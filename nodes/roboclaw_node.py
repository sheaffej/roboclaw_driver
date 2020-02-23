#!/usr/bin/env python

from __future__ import print_function
import traceback
import threading

import rospy
import diagnostic_updater
import diagnostic_msgs

from roboclaw_driver.msg import Stats, SpeedCommand
from roboclaw_driver import RoboclawControl, Roboclaw, RoboclawStub


DEFAULT_DEV_NAMES = "/dev/ttyACM0"
DEFAULT_BAUD_RATE = 115200
DEFAULT_NODE_NAME = "roboclaw"
DEFAULT_LOOP_HZ = 100
DEFAULT_ADDRESS = 0x80
DEFAULT_DEADMAN_SEC = 3
DEFAULT_STATS_TOPIC = "~stats"
DEFAULT_SPEED_CMD_TOPIC = "~speed_command"


class RoboclawNode:
    def __init__(self, node_name):
        """
        Parameters:
            :param str node_name: ROS node name
            :param RoboclawControl rbc_ctl: RoboclawControl object that controls the hardware
            :param int loop_rate: Integer rate in Hz of the main loop
        """
        self._node_name = node_name
        self._rbc_ctls = []  # Populated by the connect() method

        # Records the values of the last speed command
        self._last_cmd_time = rospy.get_rostime()
        self._last_cmd_m1_qpps = 0
        self._last_cmd_m2_qpps = 0
        self._last_cmd_accel = 0
        self._last_cmd_max_secs = 0
        self._speed_cmd_lock = threading.RLock()  # To serialize access to cmd variables

        # Set up the Publishers
        self._stats_pub = rospy.Publisher(
            rospy.get_param("~stats_topic", DEFAULT_STATS_TOPIC),
            Stats,
            queue_size=1
        )

        # Set up the Diagnostic Updater
        self._diag_updater = diagnostic_updater.Updater()
        self._diag_updater.setHardwareID(node_name)
        self._diag_updater.add("Read Diagnostics", self._publish_diagnostics)

        # Set up the SpeedCommand Subscriber
        rospy.Subscriber(
            rospy.get_param("~speed_cmd_topic", DEFAULT_SPEED_CMD_TOPIC),
            SpeedCommand,
            self._speed_cmd_callback
        )

        # For logdebug
        self.prev_m1_val = 0
        self.prev_m2_val = 0

    @property
    def roboclaw_control(self):
        return self._rbc_ctl

    def connect(self, dev_name, baud_rate, address, test_mode=False):
        """Connects the node to the Roboclaw controller, or the test stub
        Parameters:
            :param str dev_name: Serial device name (e.g. /dev/ttyACM0)
            :param int baud_rate: Serial baud rate (e.g. 115200)
            :param int address: Serial address (default 0x80)
            :param bool test_mode: True if connecting to the controller test stub
        """
        rospy.loginfo("Connecting to roboclaw")
        if not test_mode:
            roboclaw = Roboclaw(dev_name, baud_rate)
        else:
            roboclaw = RoboclawStub(dev_name, baud_rate)
        self._rbc_ctls.append(RoboclawControl(roboclaw, address))

    def run(self, loop_hz=10, deadman_secs=1):
        """Runs the main loop of the node
        Parameters:
            :param int loop_hz: Loops per sec of main loop (default 10 hertz)
            :param int deadman_secs: Seconds that the Roboclaw will continue the last command
                before stopping if another command is not received
        """
        rospy.loginfo("Running node")
        looprate = rospy.Rate(loop_hz)

        try:
            rospy.loginfo("Starting main loop")

            while not rospy.is_shutdown():

                # Read and publish encoder readings
                read_success, stats = self._rbc_ctls[0].read_stats()
                for error in stats.error_messages:
                    rospy.logwarn(error)
                if read_success:
                    self._publish_stats(stats)
                else:
                    rospy.logwarn("Error reading stats from Roboclaw: {}".format(stats))

                # Stop motors if running and no commands are being received
                if (stats.m1_enc_qpps != 0 or stats.m2_enc_qpps != 0):
                    if (rospy.get_rostime() - self._last_cmd_time).to_sec() > deadman_secs:
                        rospy.loginfo("Did not receive a command for over 1 sec: Stopping motors")
                        decel = max(abs(stats.m1_enc_qpps), abs(stats.m2_enc_qpps)) * 2
                        for rbc_ctl in self._rbc_ctls:
                            rbc_ctl.stop(decel=decel)

                # Publish diagnostics
                self._diag_updater.update()

                looprate.sleep()

        except rospy.ROSInterruptException:
            rospy.logwarn("ROSInterruptException received in main loop")

    def _publish_stats(self, stats):
        """Publish stats to the <node_name>/stats topic

        Parameters:
            :param roboclaw_control.RoboclawStats stats: Stats from the Roboclaw controller
        """
        msg = Stats()
        msg.header.stamp = rospy.get_rostime()

        msg.m1_enc_val = stats.m1_enc_val
        msg.m1_enc_qpps = stats.m1_enc_qpps

        msg.m2_enc_val = stats.m2_enc_val
        msg.m2_enc_qpps = stats.m2_enc_qpps

        # rospy.logdebug("Encoder diffs M1:{}, M2:{}".format(
        #     stats.m1_enc_val - self.prev_m1_val,
        #     stats.m2_enc_val - self.prev_m2_val
        # ))
        self.prev_m1_val = stats.m1_enc_val
        self.prev_m2_val = stats.m2_enc_val

        self._stats_pub.publish(msg)

    def _publish_diagnostics(self, stat):
        """Function called by the diagnostic_updater to fetch and publish diagnostics
        from the Roboclaw controller

        Parameters:
        :param diagnostic_updater.DiagnosticStatusWrapper stat:
            DiagnosticStatusWrapper provided by diagnostic_updater when called

        Returns: The updated DiagnosticStatusWrapper
        :rtype: diagnostic_updater.DiagnosticStatusWrapper
        """
        for i, rbc_ctl in enumerate(self._rbc_ctls):
            diag = rbc_ctl.read_diag()

            stat.add("[{}] Temperature 1 (C):".format(i), diag.temp1)
            stat.add("[{}] Temperature 2 (C):".format(i), diag.temp2)
            stat.add("[{}] Main Battery (V):".format(i), diag.main_battery_v)
            stat.add("[{}] Logic Battery (V):".format(i), diag.logic_battery_v)
            stat.add("[{}] Motor 1 current (Amps):".format(i), diag.m1_current)
            stat.add("[{}] Motor 2 current (Amps):".format(i), diag.m2_current)

            for msg in diag.error_messages:
                level = diagnostic_msgs.msg.DiagnosticStatus.WARN
                if "error" in msg:
                    level = diagnostic_msgs.msg.DiagnosticStatus.ERROR
                stat.summary(level, "[{}]: msg".format(i))

        return stat

    def _speed_cmd_callback(self, command):
        """
        Parameters:
            :param SpeedCommand command: The forward/turn command message
        """
        with self._speed_cmd_lock:
            self._last_cmd_time = rospy.get_rostime()

            # Skip if the new command is not different than the last command
            if (command.m1_qpps == self._last_cmd_m1_qpps
                and command.m2_qpps == self._last_cmd_m2_qpps
                and command.accel == self._last_cmd_accel
                and command.max_secs == self._last_cmd_max_secs):
                rospy.logdebug("Speed Command received, but no change in command values")

            else:
                rospy.logdebug(
                    "M1 speed: {} | M2 speed: {} | Accel: {} | Max Secs: {}"
                    .format(command.m1_qpps, command.m2_qpps, command.accel, command.max_secs)
                )

                for rbc_ctl in self._rbc_ctls:
                    success = rbc_ctl.driveM1M2qpps(
                        command.m1_qpps, command.m2_qpps,
                        command.accel, command.max_secs
                    )

                    if not success:
                        rospy.logerr("RoboclawControl SpeedAccelDistanceM1M2 failed")

    def shutdown_node(self):
        """Performs Node shutdown tasks
        """
        rospy.loginfo("Shutting down...")


if __name__ == "__main__":

    # Setup the ROS node
    rospy.init_node(DEFAULT_NODE_NAME, log_level=rospy.DEBUG)
    node_name = rospy.get_name()
    print("node_name: {}".format(node_name))

    # Read the input parameters
    dev_names = rospy.get_param("~dev_names", DEFAULT_DEV_NAMES)
    baud_rate = int(rospy.get_param("~baud", DEFAULT_BAUD_RATE))
    address = int(rospy.get_param("~address", DEFAULT_ADDRESS))
    loop_hz = int(rospy.get_param("~loop_hz", DEFAULT_LOOP_HZ))
    deadman_secs = int(rospy.get_param("~deadman_secs", DEFAULT_DEADMAN_SEC))
    test_mode = bool(rospy.get_param("~test_mode", False))

    rospy.logdebug("node_name: {}".format(node_name))
    rospy.logdebug("dev_names: {}".format(dev_names))
    rospy.logdebug("baud: {}".format(baud_rate))
    rospy.logdebug("address: {}".format(address))
    rospy.logdebug("loop_hz: {}".format(loop_hz))
    rospy.logdebug("deadman_secs: {}".format(deadman_secs))
    rospy.logdebug("test_mode: {}".format(test_mode))

    node = RoboclawNode(node_name)
    rospy.on_shutdown(node.shutdown_node)

    try:
        # Initialize the Roboclaw controllers
        for dev in dev_names.split(','):
            node.connect(dev, baud_rate, address, test_mode)
        node.run(loop_hz=loop_hz, deadman_secs=DEFAULT_DEADMAN_SEC)

    except Exception as e:
        rospy.logfatal("Unhandled exeption...printing stack trace then shutting down node")
        rospy.logfatal(traceback.format_exc())

    # Shutdown and cleanup
    if node:
        node.shutdown_node()
    rospy.loginfo("Shutdown complete")
