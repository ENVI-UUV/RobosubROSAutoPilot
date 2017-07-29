#!/usr/bin/env python

import rospy
import os
from std_msgs.msg import Bool
from mavros_msgs.srv import CommandBool


def main():
    rospy.init_node("kill_switch")
    ks_pub = rospy.Publisher("kill_switch", Bool, queue_size = 10)
    while ks_pub.get_num_connections() <= 0:
        x=0
    while not rospy.is_shutdown():
        exists = os.path.exists("/dev/video0") or os.path.exists("/dev/video1")
        msg = Bool(exists)
        ks_pub.publish(msg)
        rospy.sleep(0.5)

if __name__ == "__main__":
    main()
