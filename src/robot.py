#!/usr/bin/env python
import os
import time
import sys
import rospy
import cv2
import mavros
from mavros import command
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import RCOut
from mavros_msgs.msg import State


class Robot:
    def __init__(self, collect = False):
        rospy.loginfo("Starting robot...")
        self.controls = [0,0,0,0,0,0,0]
        self.fwd_image_sub = rospy.Subscriber("camera/image_raw", Image, self.fwd_image_cb)
        self.dwn_image_sub = rospy.Subscriber("webcam/image_raw", Image, self.dwn_image_cb)
        self.controls_sub = rospy.Subscriber("mavros/rc/out", RCOut, self.controls_cb)
        self.arm_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.motor_pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size = 1000)
        self.bridge = CvBridge()
        self.fwd_img_num = 0
        self.dwn_img_num = 0
        self.fwd_img_csv_fname = "/media/nvidia/fwd_img_timestamp.csv"
        self.dwn_img_csv_fname = "/media/nvidia/dwn_img_timestamp.csv"
        self.img_path = "/media/nvidia/images/"
        self.data_collection = collect
        if self.data_collection:
            f = open(self.fwd_img_csv_fname, "w+")
            f.write("timestamp (seconds from epoch), forward camera image filename\n")
            f.close()
            f = open(self.dwn_img_csv_fname, "w+")
            f.write("timestamp (seconds from epoch), downward camera image filename\n")
            f.close()
        mavros.set_namespace()
        self.armed = False
        rospy.wait_for_service("/mavros/cmd/arming")        

    def arm(self):
        rospy.loginfo("Arming pixhawk...")
        command.arming(True)
        self.armed = True

    def disarm(self):
        rospy.loginfo("Disarming pixhawk...")
        command.arming(False)
        self.armed = False

    def is_armed(self):
        return self.armed 

    def fwd_image_cb(self, msg):
        if self.data_collection: 
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as e:
                rospy.logerr(e)
            fwd_img_filename ="fwd_img_" + str(self.fwd_img_num) + ".png"
            if self.fwd_img_num < 10666666:
                if not os.path.exists(self.img_path):
                    os.makedirs(self.img_path)
                cv2.imwrite(self.img_path + fwd_img_filename, cv_image)
                csv = open(self.fwd_img_csv_fname, "a")
                timestamp = msg.header.stamp
                csv.write(str(timestamp) + ", [" + "; ".join(map(str,self.controls)) + "], " + fwd_img_filename + "\n")
                csv.close()
                self.fwd_img_num += 1
    
    def dwn_image_cb(self, msg):
        if self.data_collection:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as e:
                rospy.logerr(e)
            dwn_img_filename = "dwn_img_" + str(self.dwn_img_num) + ".png"
            if self.dwn_img_num < 10666666:
                if not os.path.exists(self.img_path):
                    os.makedirs(self.img_path)
                cv2.imwrite(self.img_path + dwn_img_filename, cv_image)
                csv = open(self.dwn_img_csv_fname, "a")
                timestamp = msg.header.stamp
                csv.write(str(timestamp) + ", [" + "; ".join(map(str, self.controls)) + "], " + dwn_img_filename + "\n")
                csv.close()
                self.dwn_img_num += 1

    def controls_cb(self, data):
        self.controls = data.channels




