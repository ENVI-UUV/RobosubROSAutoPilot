#!/usr/bin/env python
import os
import sys
import rospy
import cv2
import mavros
from mavros import command
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from mavros_msgs.srv import CommandBool


class Robot:
    def __init__(self):
        rospy.loginfo("Starting robot...")
        self.fwd_image_sub = rospy.Subscriber("camera/image_raw", Image, self.fwd_image_cb)
        self.dwn_image_sub = rospy.Subscriber("webcam/image_raw", Image, self.dwn_image_cb)
        self.ks_sub = rospy.Subscriber("kill_switch", Bool, self.ks_cb)
        self.bridge = CvBridge()
        self.fwd_img_num = 0
        self.dwn_img_num = 0
        self.fwd_img_csv_fname = "/media/nvidia/fwd_img_timestamp.csv"
        self.dwn_img_csv_fname = "/media/nvidia/dwn_img_timestamp.csv"
        self.img_path = "/media/nvidia/images/"
        f = open(self.fwd_img_csv_fname, "w+")
        f.write("timestamp (seconds from epoch), forward camera image filename\n")
        f.close()
        f = open(self.dwn_img_csv_fname, "w+")
        f.write("timestamp (seconds from epoch), downward camera image filename\n")
        f.close()
        mavros.set_namespace()
        rospy.wait_for_service("/mavros/cmd/arming")
        self.arm_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

    def arm_pixhawk(self):
        rospy.logerr(self.arm_srv(True))

    def fwd_image_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        fwd_img_filename ="fwd_img_" + str(self.fwd_img_num) + ".png"
        if self.fwd_img_num < 10666666:
            cv2.imwrite(self.img_path + fwd_img_filename, cv_image)
            csv = open(self.fwd_img_csv_fname, "a")
            timestamp = msg.header.stamp
            csv.write(str(timestamp) + ", " + fwd_img_filename + "\n")
            csv.close()
            self.fwd_img_num += 1
    
    def dwn_image_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        dwn_img_filename = "dwn_img_" + str(self.dwn_img_num) + ".png"
        if self.dwn_img_num < 10666666:
            cv2.imwrite(self.img_path + dwn_img_filename, cv_image)
            csv = open(self.dwn_img_csv_fname, "a")
            timestamp = msg.header.stamp
            csv.write(str(timestamp) + ", " + dwn_img_filename + "\n")
            csv.close()
            self.dwn_img_num += 1

    def ks_cb(self, msg):
        if msg.data == False:
            rospy.loginfo("Disarming Pixhawk...")
            #result = self.arm_srv(msg.data)
            #rospy.loginfo(result)
            command.arming(False)



