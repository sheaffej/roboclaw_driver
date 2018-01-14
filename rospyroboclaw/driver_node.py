import rospy

# from .roboclaw import Roboclaw

if __name__ == "__main__":

    node_name = rospy.get_param("~name", "roboclaw1")
    dev_name = rospy.get_param("~dev", "/dev/ttyACM0")
    baud_rate = int(rospy.get_param("~baud", "115200"))

    rospy.init_node(node_name)
    # rospy.on_shutdown(self.shutdown)
    # roboclaw = Roboclaw(dev_name, baud_rate)
    rospy.loginfo("Connecting to roboclaw")

    try:
        # node = Node()
        # node.run()
        pass
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")

