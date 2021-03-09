#pragma once

#include "ros/ros.h"

#include <costmap_2d/costmap_2d.h>

#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Path.h>

#include <fpp_ros/data_classes/fpp_param_manager.h>
#include <fpp_ros/data_classes/ras_param_manager.h>
#include <fpp_ros/data_classes/fpp_controller_param.h>
#include <fp_utils/geometry_info/robot_contour.h>
#include <fp_utils/geometry_info/formation_contour.h>
#include <fpp_msgs/RobotOutline.h>

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
							  const fpp_data_classes::FPPControllerParams &fpp_controller_params,
							  ros::NodeHandle &nh,
							  ros::NodeHandle &planner_nh);

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
			
			#pragma region Parameter
            //! This is all the information that was read from the config file about each robot
            std::shared_ptr<fpp_data_classes::FPPParamManager> fpp_params_;
			#pragma endregion

            #pragma region Process Info
            //! Name of the planer that is used to generate the plan for the formation
            std::string planner_name_;
            //! Direct pointer to the costmap to get updates instantly without the usage of topics
            costmap_2d::Costmap2D *costmap_;
            //! Global frame which is used to transform points into map coordinate system
            std::string global_frame_;
			//! Through this object the target distances from the robots to the formation centre can be calculated
			geometry_info::FormationContour target_formation_contour_;
			#pragma endregion

			#pragma region Topics/Services/Actions
			//! Topic to publish the plan of the current robot.
            ros::Publisher robot_plan_pub_;

			ros::ServiceClient get_robot_outline_src_client_;
			#pragma endregion

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
             * @brief Helper method for initializing all actions
             * 
             */
			virtual void initActions();
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

			std::vector<Eigen::Vector2f> convPolygonToEigenVector(geometry_msgs::Polygon polygon);
	};
}