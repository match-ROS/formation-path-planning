#!/usr/bin/env python

import dynamic_reconfigure
import geometry_msgs
import rospy
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32

from dynamic_reconfigure.client import Client
from rospy.core import rospyinfo

def callback(config):
    rospy.loginfo("CONFIG SET")
    rospy.loginfo(config)

if __name__ == '__main__':
    try:
        rospy.init_node('Dynamic_Footprint_Change', anonymous=True)
    
        # Dynamic client not working?
        rospy.loginfo("before client")
        client = Client("/robot1/move_base_flex/global_costmap/inflation", timeout=30, config_callback=callback)
        rospy.loginfo("updating")
        # returnvalue = client.update_configuration({"footprint":"[[0.506,-0.42],[0.506,0.42],[-0.454,0.42],[-0.454,-0.42]]"})
        returnvalue = client.update_configuration({"inflation_radius":"1.0"})
        rospy.loginfo(returnvalue)
        rospy.loginfo("DONE")

        # The loop is just to get the callback method outputs
        r = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            r.sleep()
    except rospy.ROSInterruptException:
        pass