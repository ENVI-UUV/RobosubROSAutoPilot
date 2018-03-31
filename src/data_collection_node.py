#!/usr/bin/env python

import time
import os
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import State
from sensor_msgs.msg import Image
from robot import Robot
import numpy as np

def main():
    rospy.init_node("robot")
    robosub = Robot(True)
    rospy.loginfo("---------------------Starting Data Collection!")
    while not rospy.is_shutdown():
        rospy.spin();

if __name__=="__main__":
    main()

