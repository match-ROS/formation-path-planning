#!/usr/bin/env python

import rospy
from collections import defaultdict

from fpp_msgs.msg import GlobalPlanPoseMetaData

master_name = "robot0"
robot_name_list = ["robot0", "robot1"]

first_message = False # type: Boolean
timer = None # type: Timer
latest_time_stamp = None # type: Time
timeout_length = 1.0

message_list = {}
formation_plan_list = None
publisher_list = {}
formation_plan_meta_data_publisher = None

def robot_meta_data_callback(data):
    # type: (GlobalPlanPoseMetaData) -> None
    global first_message
    global latest_time_stamp
    global timer
    global message_list

    message_list[data.robot_name.data].append(data)
    
    latest_time_stamp = rospy.Time.now()
    if not first_message:
        first_message = True

    # rospy.loginfo("received msg")

def formation_meta_data_callback(data):
    global formation_plan_list
    formation_plan_list.append(data)

def timeout_callback(data):
    global latest_time_stamp
    global message_list
    
    if not first_message:
        return
    if (rospy.Time.now() - latest_time_stamp).to_sec() > timeout_length:
        rospy.loginfo("fpp_meta_data_sync_node: Publish time synced data")
        publish_all_meta_data()
        timer.shutdown()

def publish_all_meta_data():
    global message_list
    
    for counter in range(0,len(message_list[robot_name_list[0]])):
        formation_plan_meta_data_publisher.publish(formation_plan_list[counter])
        for robot_name in robot_name_list:
            # rospy.loginfo(message_list[robot_name])
            publisher_list[robot_name].publish(message_list[robot_name][counter])
            rospy.sleep(0.001)

if __name__ == '__main__':
    rospy.init_node("fpp_meta_data_sync_node")

    robot_meta_data_list = list()
    publisher_list = dict()
    message_list = dict()
    formation_plan_list = list()
    
    master_robot_name = "robot0"

    rospy.Subscriber("/" + master_robot_name + "/move_base_flex/FormationPathPlanner/formation_plan_meta_data",
                     GlobalPlanPoseMetaData,
                     formation_meta_data_callback)

    formation_plan_meta_data_publisher = rospy.Publisher("/" + master_robot_name + "/formation_plan_meta_data_relay",
                                                         GlobalPlanPoseMetaData,
                                                         queue_size=10)

    for robot_name in robot_name_list:
        rospy.Subscriber("/" + robot_name + "/move_base_flex/FormationPathPlanner/robot_plan_meta_data",
                         GlobalPlanPoseMetaData,
                         robot_meta_data_callback)

        publisher_list[robot_name] = rospy.Publisher("/" + robot_name + "/global_plan_meta_data_relay",
                                                     GlobalPlanPoseMetaData,
                                                     queue_size=10)

        message_list[robot_name] = list()

    timer = rospy.Timer(rospy.Duration(0.1), timeout_callback)

    rospy.spin()
