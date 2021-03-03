#pragma once

#include "ros/ros.h"

#include <costmap_2d/costmap_2d.h>

#include <nav_msgs/Path.h>

#include <fpp_ros/data_classes/fpp_param_manager.h>

#include <string>
#include <memory> // Usage of smart pointers
#include <vector>

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
            FPPControllerBase(const std::shared_ptr<fpp_data_classes::FPPParamManager> &fpp_params,
                              ros::NodeHandle &nh,
                              ros::NodeHandle &planner_nh);

			/**
			 * @brief Initialization method that should be overriten and called in the derived class
			 * 
			 * @param planner_name Name of the planner this controller is used in
			 * @param costmap Pointer to the costmap for planning collision-free path
			 * @param global_frame Name of the global frame the robot is in
			 */
            virtual void initialize(std::string planner_name, costmap_2d::Costmap2D *costmap, std::string global_frame);

            /**
             * 
             * @brief 
             * This has to be implemented by the master and slave as it is pure virtual
             */

			/**
			 * @brief This method is called when the planning should happen
			 * 
			 * @param start Start position of the individual robot
			 * @param goal Goal position of the individual robot
			 * @param plan Store the found plan in this list of poses
			 */
            virtual void execute(const geometry_msgs::PoseStamped &start,
                                 const geometry_msgs::PoseStamped &goal,
                                 std::vector<geometry_msgs::PoseStamped> &plan) = 0;

        protected:
            //! NodeHandle from the node that initializes the fpp controller classes
            ros::NodeHandle &nh_;
            //! NodeHandle that is in the namespace of the planner
            ros::NodeHandle &planner_nh_;
			
            //! This is all the information that was read from the config file about each robot
            std::shared_ptr<fpp_data_classes::FPPParamManager> fpp_params_;

            // Information for used planner
            //! Name of the planer that is used to generate the plan for the formation
            std::string planner_name_;
            //! Direct pointer to the costmap to get updates instantly without the usage of topics
            costmap_2d::Costmap2D *costmap_;
            //! Global frame which is used to transform points into map coordinate system
            std::string global_frame_;

			//! Topic to publish the plan of the current robot.
            ros::Publisher robot_plan_pub_;

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
	};
}