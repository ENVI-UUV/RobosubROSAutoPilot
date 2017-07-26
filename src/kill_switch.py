import rospy
import os

while os.path.exists("/dev/video0"):
    x = 0
rospy.signal_shutdown("killswitch")
