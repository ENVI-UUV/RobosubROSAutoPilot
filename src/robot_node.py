#!/usr/bin/env python

import time
import os
import rospy
from std_msgs.msg import Bool
from mavros_msgs.msg import OverrideRCIn
from robot import Robot


killswitch = False
motor_pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size = 1000)
motor_msg = OverrideRCIn()
lim = len(motor_msg.channels)
for i in range(lim):
    motor_msg.channels[i] = 1500

def ks_cb(msg):
    if msg.data:
        motor_msg.channels[4] = 1600
    else:
        motor_msg.channels[4] = 1500

def main():
    rospy.init_node("robot")
    robosub = Robot()
    course_sub = rospy.Subscriber("kill_switch", Bool, ks_cb)
    while motor_pub.get_num_connections() < 1:
        x=0
    rospy.loginfo("---------------------TEST!")
    time.sleep(1)
    robosub.arm_pixhawk()
    time.sleep(2)
    start = time.time()
    step = True
    while not rospy.is_shutdown():
        temp = os.path.exists("/dev/video0") or os.path.exists("/dev/video1") or os.path.exists("/dev/video2")
        rospy.loginfo(temp)
        if temp:
            '''
            if int(str(time.time())[0]) % 5 == 0:
                if step:
                    motor_msg.channels[3] = 1575
                    step = False
                else:
                    #motor_msg.channels[3] = 1500
                    step = True
            '''
            motor_msg.channels[3] = 1505
            motor_msg.channels[2] = 1500
            motor_msg.channels[4] = 1650
        else:
            motor_msg.channels[3] = 1500
            motor_msg.channels[2] = 1500
            motor_msg.channels[4] = 1500
        motor_pub.publish(motor_msg)
        time.sleep(0.1)
    rospy.spin()

if __name__=="__main__":
    main()

