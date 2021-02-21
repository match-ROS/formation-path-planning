#pragma once

#include <string>

namespace fpc_data_classes
{
	/**
     * @brief Contains all information for each robot that is in the formation
     * Data is read from the local_planner_config.yaml and loaded by move_base_flex
     * 
     */
	struct LocalPlannerRobotInfo
	{
		//! Name of the robot
		std::string robot_name;
		//! Namespace where the robot and all topics are included in
		std::string robot_namespace;
		//! True if the robot is the master of the formation
		bool fpc_master;
		//! Topic name where the pose of the robot can be received
		std::string robot_pose_topic;
	};
}