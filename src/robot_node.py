#!/usr/bin/env python

import time
import os
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Image
from neuralnet import inference_runner
from robot import Robot
import numpy as np



killswitch = False
model_path = "/home/nvidia/catkin_ws/src/robosub/include/InceptionV3_fc1024_X2_pos83_epoch.hdf5"
non_img_inputs = [.5,.5,.5,.5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
motor_pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size = 1000)
motor_msg = OverrideRCIn()
lim = len(motor_msg.channels)
for i in range(lim):
    motor_msg.channels[i] = 1500
bridge = CvBridge()
curr_fwd_img = np.zeros((640,480,3), dtype=np.uint8)
prev_fwd_img = np.zeros((640,480,3), dtype=np.uint8)
curr_dwn_img = np.zeros((640,480,3), dtype=np.uint8)
prev_dwn_img = np.zeros((640,480,3), dtype=np.uint8)

def ks_cb(msg):
    if msg.data:
        motor_msg.channels[4] = 1500
    else:
        motor_msg.channels[4] = 1500

def fwd_img_sub(msg):
    prev_fwd_img = curr_fwd_img
    try:
        curr_fwd_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)
    

def dwn_img_sub(msg):
    prev_dwn_img = curr_dwn_img
    try:
        curr_dwn_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)

fwd_ img_sub = rospy.Subscriber("/camera/image_raw", Image, fwd_img_sub)
dwn_img_sub = rospy.Subscriber("/webcam/image_raw", Image, dwn_img_sub)

def main():
    rospy.init_node("robot")
    robosub = Robot()
    course_sub = rospy.Subscriber("kill_switch", Bool, ks_cb)
    model = inference_runner(model_path)
    while motor_pub.get_num_connections() < 1:
        x=0
    rospy.loginfo("---------------------TEST!")
    time.sleep(1)
    robosub.arm_pixhawk()
    time.sleep(2)
    while not rospy.is_shutdown():
        temp = os.path.exists("/dev/video0") or os.path.exists("/dev/video1") or os.path.exists("/dev/video2")
        rospy.loginfo(temp)
        if temp:
            controls = model.run_inference(non_img_inputs, prev_fwd_img, prev_dwn_img, curr_fwd_img, curr_dwn_img)
            motor_msg.channels[3] = controls[0]
            motor_msg.channels[2] = controls[1]
            motor_msg.channels[5] = controls[3]
            motor_msg.channels[4] = controls[4]
        else:
            motor_msg.channels[3] = 1500
            motor_msg.channels[2] = 1500
            motor_msg.channels[5] = 1500
            motor_msg.channels[4] = 1500
        motor_pub.publish(motor_msg)
        time.sleep(0.1)
    rospy.spin()

if __name__=="__main__":
    main()

