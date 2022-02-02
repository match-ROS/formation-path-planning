#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Pose

from tf import transformations

class set_robot_pose:

    def __init__(self):
        rospy.init_node("set_robot_pose_node")
        rospy.loginfo("robot pose set")
        self.config()
        self.robot_pose = Pose()

        rospy.Subscriber(self.follower_pose_topic, Pose, self.set_pose_cb)
        self.pub = rospy.Publisher(self.set_pose_topic, Pose, queue_size=10)

        rospy.sleep(1)


    def config(self):
        self.set_pose_topic = rospy.get_param('~set_pose_topic')
        self.follower_pose_topic = rospy.get_param('~real_robot_pose_topic')
        # self.rel_pose = rospy.get_param('~rel_pose')
        self.rel_pose = [1,0]
        print(self.rel_pose)
        print(type(self.rel_pose))


    def set_pose_cb(self,data):
        R = transformations.quaternion_matrix([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w])
        print(R[0,0])
        self.robot_pose.position.x = data.position.x + R[0,0] * self.rel_pose[0]  + R[0,1] * self.rel_pose[1]
        self.robot_pose.position.y = data.position.y + R[1,0] * self.rel_pose[0]  + R[1,1] * self.rel_pose[1]
        self.robot_pose.orientation = data.orientation

        self.pub.publish(self.robot_pose)


if __name__=="__main__":
    set_robot_pose()