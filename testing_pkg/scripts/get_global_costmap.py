#!/usr/bin/env python

import dynamic_reconfigure
import geometry_msgs
import rospy
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32
from nav_msgs.msg import OccupancyGrid

from dynamic_reconfigure.client import Client
from rospy.core import rospyinfo

def callback(data):
    for x in range(512*1024, 512*1024+1024):
        rospy.loginfo("index: %i, data: %f", (x-512*1024), (data.data[x]*(254.0/100.0)))
        
if __name__ == '__main__':
    try:
        rospy.init_node('Dynamic_Footprint_Change', anonymous=True)

        sub = rospy.Subscriber("/robot1_ns/move_base_flex/global_costmap/costmap", OccupancyGrid, callback=callback)

        # The loop is just to get the callback method outputs
        # r = rospy.Rate(0.1)
        # while not rospy.is_shutdown():
        #     r.sleep()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass