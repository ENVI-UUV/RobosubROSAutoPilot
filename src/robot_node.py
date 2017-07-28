#!/usr/bin/env python

import time
import rospy
from mavros_msgs.msg import OverrideRCIn
from robot import Robot


def main():
    rospy.init_node("robot")
    robosub = Robot()
    motor_pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size = 1000)
    while motor_pub.get_num_connections() < 1:
        rospy.sleep()
    motor_msg = OverrideRCIn()
    lim = len(motor_msg.channels)
    for i in range(lim):
        motor_msg.channels[i] = 1500
    rospy.loginfo("---------------------TEST!")
    time.sleep(3)
    robosub.arm_pixhawk()
    time.sleep(8)
    start = time.time()
    while not rospy.is_shutdown():
        
        if time.time() - start < 90:
            motor_msg.channels[3] = 1800
 #           motor_msg.channels[2] = 1600
        else:
            motor_msg.channels[3] = 1500
  #          motor_msg.channels[2] = 1500
        

        #rospy.loginfo("publishing rcoverride msg")
        motor_pub.publish(motor_msg)
        time.sleep(0.1)


if __name__=="__main__":
    main()
