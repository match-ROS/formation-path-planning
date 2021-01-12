#!/usr/bin/env python

import rospy
from basic_ur5_movement import BasicUR5Movement

if __name__ == "__main__":
    basic_ur5_movement_interface= BasicUR5Movement()
    basic_ur5_movement_interface.go_to_joint_state()
