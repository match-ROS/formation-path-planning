#pragma once

#include "ros/ros.h"

#include <costmap_2d/costmap_2d.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Path.h>

#include <fpp_ros/data_classes/fpp_param_manager.h>
#include <fpp_ros/data_classes/ras_param_manager.h>
#include <fpp_ros/data_classes/fpp_controller_param.h>
#include <fp_utils/geometry_info/robot_contour.h>
#include <fp_utils/geometry_info/formation_contour.h>
#include <fp_utils/bezier_splines/cubic_bezier_spline.h>
#include <fpp_msgs/RobotOutline.h>
#include <fpp_msgs/GlobalPlanPoseMetaData.h>
#include <fpp_ros/plan_transformation/rigid_plan_transformation.h>

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
			//! Offset to the formation centre. Viewed from the formation centre
			Eigen::Vector2f formation_to_robot_offset_;

			plan_transformation::RigidPlanTransformation formation_to_robot_trafo_;

			std::shared_ptr<bezier_splines::CubicBezierSplines> x_reconfiguration_spline_;
			std::shared_ptr<bezier_splines::CubicBezierSplines> y_reconfiguration_spline_;
			#pragma endregion

			#pragma region Topics/Services/Actions
			//! Topic to publish the plan of the current robot.
            ros::Publisher robot_plan_pub_;
			//! Topic to publish the meta data of the plan to so it can be plotted in plotjuggler
			ros::Publisher robot_plan_meta_data_pub_;

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

			void createReconfigurationSplines();
			Eigen::Vector2f calcReconfigurationStep(int reconfiguration_index,
													int reconfiguration_distance);

			/**
             * @brief Publish a plan using the publisher that is handed in as parameter
             * 
             * @param plan_publisher Publisher that is used to publish the plan
             * @param plan Plan that contains all points the define the plan
             */
			void publishPlan(const ros::Publisher &plan_publisher,
							 const std::vector<geometry_msgs::PoseStamped> &plan);

			void publishPlanMetaData(const ros::Publisher &plan_meta_data_publisher,
									 const std::vector<geometry_msgs::PoseStamped> &plan);

			std::vector<geometry_msgs::PoseStamped> transformFormationToRobotPlan(
				std::vector<geometry_msgs::PoseStamped> &formation_plan);

			#pragma region Conversion Methods
			std::vector<Eigen::Vector2f> convPolygonToEigenVector(geometry_msgs::Polygon polygon);
			geometry_msgs::Pose convEigenToPose(Eigen::Vector3f eigen_pose);
			Eigen::Vector3f convPoseToEigen(geometry_msgs::Pose pose);
			#pragma endregion

            #pragma region Helper Methods
            std::string getRobotNameByTopicNamespace(std::string topic);
            #pragma endregion
	};
}