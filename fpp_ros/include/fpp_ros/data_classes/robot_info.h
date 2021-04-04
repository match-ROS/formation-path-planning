#pragma once

#include <string>
#include <vector>
#include <Eigen/Dense>

namespace fpp_data_classes
{
    /**
     * @brief Contains all information for each robot that is in the formation
     * Data is read from the global_planner_config.yaml and loaded by move_base_flex
     * 
     */
    struct RobotInfo
    {
        //! Name of the robot for identification
        std::string robot_name;
        //! Namespace of the robot so it is possible to send topics/services/actions to the robot
        std::string robot_namespace;
        //! Definition if this robot is the master of the formation
        bool fpp_master;
        //! Offset from the master robot. This vector is seen from the masterrobot. -1/-1 means left and behind the master robot.
        Eigen::Vector2f offset;
		//! Offset from the master robot after the reconfiguration
		Eigen::Vector2f reconfig_offset;
        //! Definition for the outline of this robot
        std::vector<Eigen::Vector2f> robot_outline;

		//! This is only important for the footprint generation. 
		//! After putting this into an own costmap layer this can be removed.
		std::string robot_pose_topic_name;
    };
}