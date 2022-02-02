#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Twist, PoseStamped, TransformStamped, Transform, TwistStamped
from nav_msgs.msg import Odometry
import tf
import geometry_msgs
import math
from tf import transformations
from rostopic import get_topic_type
import tf2_ros

class fake_pose_publisher_node:

    def __init__(self):
        rospy.init_node("fake_pose_publisher")
        rospy.loginfo("fake_pose_publisher running")
        self.config()
        self.time_old = rospy.get_time()
        self.robot_pose = Pose()
        self.robot_vel = Twist()
        self.d_pose = [0,0,0]
        self.d_pose_R = [0,0,0]
        self.robot_orientation = 0

        rospy.Subscriber(self.set_pose_topic, Pose, self.set_pose_cb)
        rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmd_vel_cb)
        self.pub        = rospy.Publisher(self.robot_pose_topic, Pose, queue_size=10)
        # self.pub_vel    = rospy.Publisher(self.robot_vel_topic, TwistStamped, queue_size=10)
        self.run()

        rospy.spin()


    def config(self):
        self.rate = rospy.get_param('~rate')
        self.set_pose_topic = rospy.get_param('~set_pose_topic')
        # self.robot_vel_topic = rospy.get_param('~robot_vel_topic')
        self.robot_pose_topic = rospy.get_param('~robot_pose_topic')
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic')
        self.frame_id = rospy.get_param('~frame_id')
        
    def run(self):
        Rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            time_current = rospy.get_time()
            duration = time_current - self.time_old
            self.time_old = time_current
            
            self.d_pose[0] = self.robot_vel.linear.x * duration
            self.d_pose[1] = self.robot_vel.linear.y * duration
            self.d_pose[2] = self.robot_vel.angular.z * duration
            
            R = transformations.euler_matrix(0,0,self.robot_orientation)
            
            self.d_pose_R[0] = R[0,0] * self.d_pose[0] + R[0,1] * self.d_pose[1] 
            self.d_pose_R[1] = R[1,0] * self.d_pose[0] + R[1,1] * self.d_pose[1] 
            self.d_pose_R[2] = self.d_pose[2]
            
            self.robot_pose.position.x = self.robot_pose.position.x + self.d_pose_R[0]
            self.robot_pose.position.y = self.robot_pose.position.y + self.d_pose_R[1]
            self.robot_orientation = self.robot_orientation + self.d_pose_R[2]
            
            q = transformations.quaternion_from_euler(0,0,self.robot_orientation)
            self.robot_pose.orientation.x = q[0]
            self.robot_pose.orientation.y = q[1]
            self.robot_pose.orientation.z = q[2]
            self.robot_pose.orientation.w = q[3]
            
            # self.robot_pose.header.stamp = rospy.Time.now()
            # self.robot_vel.header.stamp = rospy.Time.now()
            
            self.pub.publish(self.robot_pose)
            # self.pub_vel.publish(self.robot_vel)
            
            br = tf2_ros.TransformBroadcaster()
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "map"
            t.child_frame_id = self.frame_id
            t.transform.translation = self.robot_pose.position
            t.transform.rotation = self.robot_pose.orientation
            br.sendTransform(t)
            Rate.sleep()

    def set_pose_cb(self,data):
        self.robot_pose = data
        print(data)
        orientation = transformations.euler_from_quaternion([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w])
        self.robot_orientation = orientation[2]
        

    def cmd_vel_cb(self,data):
        self.robot_vel = data


if __name__=="__main__":
    fake_pose_publisher_node()