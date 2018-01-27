#!/usr/bin/env python

from __future__ import print_function
import traceback

import rospy
import diagnostic_updater
import diagnostic_msgs

from roboclaw_driver.msg import Stats, SpeedCommand
from ros_roboclaw.roboclaw_control import RoboclawControl
from ros_roboclaw.roboclaw import Roboclaw
from ros_roboclaw.roboclaw_stub import RoboclawStub

DEFAULT_DEV_NAME = "/dev/ttyACM0"
DEFAULT_BAUD_RATE = 115200
DEFAULT_NODE_NAME = "roboclaw1"
DEFAULT_LOOP_RATE = 30
DEFAULT_ADDRESS = 0x80
DEFAULT_DEADMAN_SEC = 30


class RoboclawNode:
    def __init__(self, node_name):
        """
        Parameters:
            :param str node_name: ROS node name
            :param RoboclawControl rbc_ctl: RoboclawControl object that controls the hardware
            :param int loop_rate: Integer rate in Hz of the main loop
        """
        self._node_name = node_name
        self._rbc_ctl = None  # Set by the connect() method below

        # Records the time of the last velocity command
        # Initialize this now so we don't have to check for None values later
        self._last_cmd_time = rospy.get_rostime()

        # Set up the Publishers
        self._stats_pub = rospy.Publisher(
            '{}/stats'.format(self._node_name),
            Stats,
            queue_size=1
        )

        # Set up the Diagnostic Updater
        self._diag_updater = diagnostic_updater.Updater()
        self._diag_updater.setHardwareID(node_name)
        self._diag_updater.add("Read Diagnostics", self._publish_diagnostics)

        # Set up the SpeedCommand Subscriber
        rospy.Subscriber(
            "{}/speed_command".format(self._node_name),
            SpeedCommand,
            self._speed_cmd_callback
        )

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
            roboclaw.start_simulator()
        self._rbc_ctl = RoboclawControl(roboclaw, address)

    def run(self, loop_rate=30, deadman_secs=1):
        """Runs the main loop of the node
        Parameters:
            :param int loop_rate: Seconds between main loop iterations (default 30 secs)
            :param int deadman_secs: Seconds that the Roboclaw will continue the last command
                before stopping if another command is not received
        """
        rospy.loginfo("Running node")
        looprate = rospy.Rate(loop_rate)

        try:
            rospy.loginfo("Starting main loop")

            while not rospy.is_shutdown():
                if (rospy.get_rostime() - self._last_cmd_time).to_sec() > deadman_secs:
                    rospy.loginfo("Did not receive a command for over 1 sec: Stopping motors")
                    self._rbc_ctl.stop()

                # Read and publish encoder readings
                stats = self._rbc_ctl.read_stats()
                for error in stats.error_messages:
                    rospy.logwarn(error)
                self._publish_stats(stats)

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

        msg.m1_enc_val = stats.m1_enc_val
        msg.m1_enc_qpps = stats.m1_enc_qpps

        msg.m2_enc_val = stats.m2_enc_val
        msg.m2_enc_qpps = stats.m2_enc_qpps

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
        diag = self._rbc_ctl.read_diag()

        stat.add("Temperature 1 (C):", diag.temp1)
        stat.add("Temperature 2 (C):", diag.temp2)
        stat.add("Main Battery (V):", diag.main_battery_v)
        stat.add("Logic Battery (V):", diag.logic_battery_v)
        stat.add("Motor 1 current (Amps):", diag.m1_current)
        stat.add("Motor 2 current (Amps):", diag.m2_current)

        for msg in diag.error_messages:
            level = diagnostic_msgs.msg.DiagnosticStatus.WARN
            if "error" in msg:
                level = diagnostic_msgs.msg.DiagnosticStatus.ERROR
            stat.summary(level, msg)

        return stat

    def _speed_cmd_callback(self, command):
        """
        Parameters:
            :param SpeedCommand command: The forward/turn command message
        """
        rospy.logdebug("Received SpeedCommand message")
        rospy.logdebug(
            "[M1 speed: {}, dist: {}] [M2 speed: {}, dist: {}] [Accel: {}]"
            .format(
                command.m1_speed, command.m1_dist,
                command.m2_speed, command.m2_dist,
                command.accel
            )
        )
        self._last_cmd_time = rospy.get_rostime()

        success = self._rbc_ctl.SpeedAccelDistanceM1M2(
            command.accel, command.m1_speed, command.m1_dist,
            command.m2_speed, command.m2_dist, reset_buffer=1
        )
        if not success:
            rospy.logerr(
                "RoboclawControl SpeedAccelDistanceM1M2({}) failed".format(command.forward_pct)
            )

    def shutdown_node(self):
        """Performs Node shutdown tasks
        """
        rospy.loginfo("Shutting down...")
        if isinstance(self._rbc_ctl.roboclaw, RoboclawStub):
            self._rbc_ctl.roboclaw.shutdown()


if __name__ == "__main__":

    # Setup the ROS node
    rospy.init_node(DEFAULT_NODE_NAME, log_level=rospy.DEBUG)
    node_name = rospy.get_name()
    print("node_name: {}".format(node_name))

    # Read the input parameters
    dev_name = rospy.get_param("~dev_name", DEFAULT_DEV_NAME)
    baud_rate = int(rospy.get_param("~baud", DEFAULT_BAUD_RATE))
    address = int(rospy.get_param("~address", DEFAULT_ADDRESS))
    loop_rate = int(rospy.get_param("~loop_secs", DEFAULT_LOOP_RATE))
    deadman_secs = int(rospy.get_param("~deadman_secs", DEFAULT_DEADMAN_SEC))
    test_mode = bool(rospy.get_param("~test_mode", False))

    rospy.logdebug("node_name: {}".format(node_name))
    rospy.logdebug("dev_name: {}".format(dev_name))
    rospy.logdebug("baud: {}".format(baud_rate))
    rospy.logdebug("address: {}".format(address))
    rospy.logdebug("loop_secs: {}".format(loop_rate))
    rospy.logdebug("deadman_secs: {}".format(deadman_secs))
    rospy.logdebug("test_mode: {}".format(test_mode))

    node = RoboclawNode(node_name)
    rospy.on_shutdown(node.shutdown_node)

    try:
        # Initialize the Roboclaw controller
        node.connect(dev_name, baud_rate, address, test_mode)
        node.run(loop_rate=loop_rate, deadman_secs=DEFAULT_DEADMAN_SEC)

    except Exception as e:
        rospy.logfatal("Unhandled exeption...printing stack trace then shutting down node")
        rospy.logfatal(traceback.format_exc())

    # Shutdown and cleanup
    if node:
        node.shutdown_node()
    rospy.loginfo("Shutdown complete")
