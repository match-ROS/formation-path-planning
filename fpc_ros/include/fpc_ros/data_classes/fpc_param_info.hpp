#pragma once 

#include "ros/ros.h"

#include <string>
#include <vector>
#include <memory>

#include <fpc_ros/data_classes/fpc_robot_info.hpp>
#include <fpc_ros/data_classes/lyapunov_params.hpp>

namespace fpc_data_classes
{
	struct FPCParamInfo
	{
		//! Parameter is defining the tolerance when arriving at the goal in the x and y direction
		double xy_default_tolerance;
		//! Parameter is defining the tolerance when arriving at the goal for the orientation.
		double yaw_default_tolerance;
		
		//! Prune the plan so every position of the original plan that was passed 1m ago will be deleted
		bool prune_plan;
		//! Params for the controller
		fpc_data_classes::ControllerParams controller_params;
		
		//! Param info for all robots included in the formation
		std::vector<std::shared_ptr<fpc_data_classes::FPCRobotInfo>> robot_info_list;
		//! Param info for the current robot
		std::shared_ptr<fpc_data_classes::FPCRobotInfo> current_robot_info;
		//! Param info for the master robot
		std::shared_ptr<fpc_data_classes::FPCRobotInfo> master_robot_info;

		std::string getCurrentRobotName() { return this->current_robot_info->robot_name; }
		std::string getCurrentRobotNamespace() { return this->current_robot_info->robot_namespace; }
		std::string getCurrentRobotPoseTopic() { return this->current_robot_info->robot_pose_topic; }
		std::string getCurrentRobotOdomTopic() { return this->current_robot_info->robot_odom_topic; }
		std::string getCurrentRobotCmdVelTopic() { return this->current_robot_info->robot_cmd_vel_topic; }
		LyapunovParams getLyapunovParams() { return this->current_robot_info->lyapunov_params; }
	};
}