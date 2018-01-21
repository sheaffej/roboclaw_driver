#!/usr/bin/env python

from __future__ import print_function
import threading

import rospy
# from geometry_msgs.msg import Twist

from roboclaw_driver.msg import Stats, SpeedCommand
from ros_roboclaw.roboclaw_control import RoboclawControl
from ros_roboclaw.roboclaw import Roboclaw
from ros_roboclaw.roboclaw_stub import RoboclawStub

DEFAULT_DEV_NAME = "/dev/ttyACM0"
DEFAULT_BAUD_RATE = 115200
DEFAULT_NODE_NAME = "roboclaw1"
DEFAULT_LOOP_RATE = 30
DEFAULT_ADDRESS = 0x80


class RoboclawNode:
    def __init__(self, node_name):
        """
        Parameters:
            node_name: ROS node name
                :type node_name: str
            rbc_ctl: RoboclawControl object that controls the hardware
                :type rbc_ctl: RoboclawControl
            loop_rate: Integer rate in Hz of the main loop
                :type loop_rate: int
        """
        self._node_name = node_name
        self._rbc_ctl = None  # Set by the connect() method below
        self._loop_rate = 30  # Default of 30 sec

        # Lock to coordiate reads & writes to the serial port
        self._serial_lock = threading.Lock()

        # Records the time of the last velocity command
        # Initialize this now so we don't have to check for None values later
        self._last_cmd_time = rospy.get_rostime()

        # Set up the Subscriber
        self._speed_sub = rospy.Subscriber(
            "{}/speed_command".format(self._node_name),
            SpeedCommand,
            self._speed_cmd_callback
        )

        # Set up the Publishers
        self._stats_pub = rospy.Publisher(
            '/{}/stats'.format(self._node_name),
            Stats,
            queue_size=1
        )

    def connect(self, dev_name, baud_rate, address, test_mode=False):
        """Connects the node to the Roboclaw controller, or the test stub
        Parameters:
            dev_name: Serial device name (e.g. /dev/ttyACM0)
            :type dev_name: str

            baud_rate: Serial baud rate (e.g. 115200)
            :type baud_rate int

            address: Serial address (default 0x80)
            :type address: int

            test_mode: True if connecting to the controller test stub
            :type test_mode: bool
        """
        rospy.loginfo("Connecting to roboclaw")
        if not test_mode:
            roboclaw = Roboclaw(dev_name, baud_rate)
        else:
            roboclaw = RoboclawStub(dev_name, baud_rate)
        self._rbc_ctl = RoboclawControl(roboclaw, address)

    def run(self, loop_rate=30):
        """Runs the main loop of the node
        Parameters:
            loop_rate: Seconds between main loop iterations (default 30 secs)
            :type loop_rate: int

        Returns: None
        """
        rospy.loginfo("Running node")
        self._loop_rate = loop_rate
        rospy.Rate(self._loop_rate)

        try:
            rospy.loginfo("Starting main loop")

            while not rospy.is_shutdown():
                with self._serial_lock:
                    if (rospy.get_rostime() - self._last_cmd_time).to_sec() > 1:
                        rospy.loginfo("Did not receive a command for over 1 sec: Stopping motors")
                        self._rbc_ctl.stop()

                    # Read and publish encoder readings
                    stats = self._rbc_ctl.read_stats()
                    for error in stats.error_messages:
                        rospy.logwarn(error)
                    self._publish_stats(stats)

                self._loop_rate.sleep()

        except rospy.ROSInterruptException:
            rospy.loginfo("ROSInterruptException received in main loop")

    def _publish_stats(self, stats):
        """Publish stats to the /<node_name>/stats topic
        Parameters:
            :type stats: roboclaw_control.RoboclawStats
        """
        msg = Stats()

        msg.m1_enc_val = stats.m1_enc_val
        msg.m1_enc_qpps = stats.m1_enc_qpps
        msg.m1_current = stats.m1_current

        msg.m2_enc_val = stats.m2_enc_val
        msg.m2_enc_qpps = stats.m2_enc_qpps
        msg.m2_current = stats.m2_current

        msg.temp = stats.temp
        msg.main_battery_v = stats.main_battery_v

        self._stats_pub.publish(msg)

    def _speed_cmd_callback(self, command):
        """
        Parameters:
            command: The forward/turn command message
                :type command: SpeedCommand
        """
        with self._serial_lock:
            self._last_cmd_time = rospy.get_rostime()

            success = self._rbc_ctl.forward(command.forward_pct)
            if not success:
                rospy.logerr("RoboclawControl forward({}) failed".format(command.forward_pct))

            if command.turn_pct >= 0:
                success = self._rbc_ctl.right(command.turn_pct)
                if not success:
                    rospy.logerr("RoboclawControl right({}) failed".format(command.turn_pct))
            else:
                success = self._rbc_ctl.left(abs(command.turn_pct))
                if not success:
                    rospy.logerr("RoboclawControl left({}) failed".format(abs(command.turn_pct)))

    def shutdown_node(self):
        """Performs Node shutdown tasks
        Returns: None
        """
        rospy.loginfo("Shutting down...")


if __name__ == "__main__":

    # Read the input parameters
    node_name = rospy.get_param("~name", DEFAULT_NODE_NAME)
    dev_name = rospy.get_param("~dev", DEFAULT_DEV_NAME)
    baud_rate = int(rospy.get_param("~baud", DEFAULT_BAUD_RATE))
    loop_rate = int(rospy.get_param("~looprate", DEFAULT_LOOP_RATE))
    address = int(rospy.get_param("~address", DEFAULT_ADDRESS))
    test_mode = bool(rospy.get_param("~test", False))

    # Setup the ROS node
    rospy.init_node(node_name)

    rospy.logdebug("node_name: {}".format(node_name))
    rospy.logdebug("dev_name: {}".format(dev_name))
    rospy.logdebug("baud_rate: {}".format(baud_rate))
    rospy.logdebug("loop_rate: {}".format(loop_rate))
    rospy.logdebug("address: {}".format(address))
    rospy.logdebug("test_mode: {}".format(test_mode))

    node = RoboclawNode(node_name)
    rospy.on_shutdown(node.shutdown_node)

    try:
        # Initialize the Roboclaw controller
        node.connect(dev_name, baud_rate, address, test_mode)
        node.run(loop_rate)

    except Exception as e:
        rospy.logfatal("Unhandled exeption...exiting")
        rospy.logdebug(e)
        rospy.signal_shutdown("Unhandled exception: {}".format(e.message))

    # Shutdown and cleanup
    if node:
        node.shutdown_node()
    rospy.loginfo("Shutdown complete")
