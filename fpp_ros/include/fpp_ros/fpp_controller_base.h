#pragma once

#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <mbf_msgs/MoveBaseAction.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>

#include <iostream>
#include <string>
#include <memory> // Usage of smart pointers
#include <vector>
#include <list>
#include <set>
#include <map>
#include <algorithm>
#include <Eigen/Dense>

#include <fpp_msgs/DynReconfigure.h>
#include <fpp_ros/data_classes/robot_info.h>

#include <fpp_ros/geometry_info/geometry_contour.h>
#include <fpp_ros/geometry_info/robot_contour.h>
#include <fpp_ros/geometry_info/formation_contour.h>

namespace fpp
{
    class FPPControllerBase
    {
        public:
            /**
             * @brief Constructor for the BaseObject of the FPPController. 
             * This stores basic information that is needed for the FPPController
             * 
             * @param robot_info_list Information about all robots in the formation
             * @param robot_info Reference to pointer that points to element in the robot_info_list that defines the information about the robot this controller is running on.
             * @param nh Nodehandle for topics/services and actions
             */
            FPPControllerBase(std::vector<std::shared_ptr<fpp_data_classes::RobotInfo>> &robot_info_list,
                              std::shared_ptr<fpp_data_classes::RobotInfo> &robot_info,
                              ros::NodeHandle &nh,
                              ros::NodeHandle &planner_nh);

            virtual void initialize(std::string planner_name, costmap_2d::Costmap2D *costmap, std::string global_frame);

            /**
             * 
             * @brief This method is called when the planning should happen
             * This has to be implemented by the master and slave as it is pure virtual
             */
            virtual void execute(const geometry_msgs::PoseStamped &start,
                                 const geometry_msgs::PoseStamped &goal,
                                 std::vector<geometry_msgs::PoseStamped> &plan) = 0;

        protected:
            //! NodeHandle from the node that initializes the fpp controller classes
            ros::NodeHandle &nh_;
            //! NodeHandle that is in the namespace of the planner
            ros::NodeHandle &planner_nh_;
			
			std::vector<std::shared_ptr<actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>>> slave_move_base_as_list_;

            //! This is all the information that was read from the config file about each robot
            std::vector<std::shared_ptr<fpp_data_classes::RobotInfo>> &robot_info_list_;

            //! This points to the object in the robot_info_list_ that contains the information about this robot
            std::shared_ptr<fpp_data_classes::RobotInfo> &robot_info_;
			//! This point to the object in the robot_info_list that represents the master in the formation path planner
			const std::shared_ptr<fpp_data_classes::RobotInfo> master_robot_info_;

            // Information for used planner
            //! Name of the planer that is used to generate the plan for the formation
            std::string planner_name_;
            //! Direct pointer to the costmap to get updates instantly without the usage of topics
            costmap_2d::Costmap2D *costmap_;
            //! Global frame which is used to transform points into map coordinate system
            std::string global_frame_;

			//! Topic to publish the plan of the current robot.
            ros::Publisher robot_plan_pub_;

			std::shared_ptr<fpp_data_classes::RobotInfo> getMasterRobotInfo(const std::vector<std::shared_ptr<fpp_data_classes::RobotInfo>> &robot_info_list);

			/**
             * @brief Helper method for intializing all services
             * 
             */
            virtual void initServices();
            /**
             * @brief Helper method for initializing all topics
             * 
             */
            virtual void initTopics();
            /**
             * @brief Helper method for initializing all timers
             * 
             */
            virtual void initTimers();

			/**
             * @brief Publish a plan using the publisher that is handed in as parameter
             * 
             * @param plan_publisher Publisher that is used to publish the plan
             * @param plan Plan that contains all points the define the plan
             */
			void publishPlan(const ros::Publisher &plan_publisher, const std::vector<geometry_msgs::PoseStamped> &plan);

			/**
			 * @brief Method for calculating a plan that has a fix offset in x and y from the master_plan
			 * 
			 * @param master_plan Master plan that dictates the plan and where the offset will be added to
			 * @param offset_plan Plan that contains the moved plan
			 * @param offset The offset which should be applied to the master plan
			 */
			void calcOffsetPlan(const std::vector<geometry_msgs::PoseStamped> &master_plan,
								std::vector <geometry_msgs::PoseStamped> &offset_plan,
								Eigen::Vector2f offset);
	};
}