#!/usr/bin/env python

import time
import rospy
from mavros_msgs.msg import OverrideRCIn
from robot import Robot


def main():
    rospy.init_node("robot")
    robosub = Robot()
    rospy.spin()

if __name__=="__main__":
    main()




'''

    motor_pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size = 1000)
    while motor_pub.get_num_connections() < 1:
        x=0
    motor_msg = OverrideRCIn()
    lim = len(motor_msg.channels)
    for i in range(lim):
        motor_msg.channels[i] = 1500
    rospy.loginfo("---------------------TEST!")
    time.sleep(3)
    robosub.arm_pixhawk()
    time.sleep(6)
    start = time.time()
    while not rospy.is_shutdown():
        
        if time.time() - start < 3:
            rospy.logwarn("Trying channel 5...")
            motor_msg.channels[4] = 1750
        elif time.time() - start < 6:
            motor_msg.channels[4] = 1250
        else:
            for i in range(lim):
                motor_msg.channels[i] = 1500
        

        #rospy.loginfo("publishing rcoverride msg")
        motor_pub.publish(motor_msg)
        time.sleep(0.1)



elif time.time() - start < 6:
    rospy.logwarn("Trying channel 2...")
    motor_msg.channels[0] = 1500
    motor_msg.channels[1] = 1600
elif time.time() - start < 9:
    rospy.logwarn("Trying channel 3...")
    motor_msg.channels[1] = 1500
    motor_msg.channels[2] = 1600
elif time.time() - start < 12:
    rospy.logwarn("Trying channel 4...")
    motor_msg.channels[2] = 1500
    motor_msg.channels[3] = 1600
elif time.time() - start < 15:
    rospy.logwarn("Trying channel 5...")
    motor_msg.channels[3] = 1500
    motor_msg.channels[4] = 1600
elif time.time() - start < 18:
    rospy.logwarn("Trying channel 6...")
    motor_msg.channels[4] = 1500
    motor_msg.channels[5] = 1600
elif time.time() - start < 21:
    rospy.logwarn("Trying channel 7...")
    motor_msg.channels[5] = 1500
    motor_msg.channels[6] = 1600
elif time.time() - start < 24:
    rospy.logwarn("Trying channel 8...")
            motor_msg.channels[6] = 1500
            motor_msg.channels[7] = 1600
        elif time.time() - start < 27:
            rospy.logwarn("Trying channel 9...")
            motor_msg.channels[7] = 1500
            motor_msg.channels[8] = 1600
'''
