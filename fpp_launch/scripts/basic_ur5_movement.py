#!/usr/bin/env python

import sys
import math

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


from std_msgs.msg import String

class BasicUR5Movement:
    #_robot #: moveit_commander.RobotCommander  # This is the outer level interface to the robot
    #_scene #: moveit_commander.PlanningSceneInterface  # This object is an interface to the world surronding the robot
    #_group #: moveit_commander.MoveGroupCommander  # This object is an interface to one group of joints

    #_display_trajectory_publisher #: rospy.Publisher  # Publisher to show trajectorys in RVIZ

    #_planning_frame #: str  # Get reference frame for this robot
    #_eef_link #: str  # Name of the end effector link
    

    def __init__(self): 
        #super(BasicUR5Movement, self).__init__()

        # First initialize `moveit_commander` and a `rospy` node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("BasicUR5Movement", anonymous=True)

        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()

        group_name = "ur5_arm"  # This is the planning group that was defined in the moveit_configuration of the UR5 and is also displayed in the RVIZ visualization
        self._group = moveit_commander.MoveGroupCommander(group_name)

        self._display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path",
                                                             moveit_msgs.msg.DisplayTrajectory,
                                                             queue_size=20)

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

    def go_to_joint_state(self):
        joint_goal = self._group.get_current_joint_values()
        joint_goal[0] = -math.pi/2
        joint_goal[1] = -math.pi/2
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
        joint_goal[5] = 0

        self._group.go(joint_goal, wait=True)

        self._group.stop()
