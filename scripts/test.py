#!/usr/bin/env python


# import rospy
print "====== roboclaw_driver.msg ======"
import roboclaw_driver.msg
print dir(roboclaw_driver.msg)


print "====== sys.path ======"
import sys
for p in sorted(sys.path):
    print p


print "====== ros_roboclaw ======"
import ros_roboclaw.roboclaw_node
print dir(ros_roboclaw.roboclaw_node)

print __name__