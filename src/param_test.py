#!/usr/bin/env python
import rospy
from mavros_msgs.srv import ParamGet

get_param = rospy.ServiceProxy("/mavros/param/get", ParamGet)
rospy.loginfo(get_param("FRAME_CONFIG"))
print get_param("FRAME_CONFIG")
