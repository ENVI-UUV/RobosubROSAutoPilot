#!/usr/bin/env python
import os
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Robot:
    def __init__(self):
        rospy.loginfo("Starting robot...")
        self.fwd_image_sub = rospy.Subscriber("camera/image_raw", Image, self.fwd_image_cb)
        self.dwn_image_sub = rospy.Subscriber("webcam/image_raw", Image, self.dwn_image_cb)
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

    def fwd_image_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        fwd_img_filename ="fwd_img_" + str(self.fwd_img_num) + ".png"
        if self.fwd_img_num < 10666666:
            print "saving forward image in " + self.img_path
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
            print(e)
        dwn_img_filename = "dwn_img_" + str(self.dwn_img_num) + ".png"
        if self.dwn_img_num < 10666666:
            print "saving down image in " + self.img_path
            cv2.imwrite(self.img_path + dwn_img_filename, cv_image)
            csv = open(self.dwn_img_csv_fname, "a")
            timestamp = msg.header.stamp
            csv.write(str(timestamp) + ", " + dwn_img_filename + "\n")
            csv.close()
            self.dwn_img_num += 1


def main(args):
    rospy.loginfo("TEST1")
    print "TEST2"
    robot = Robot()
    rospy.init_node("robot")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


        
