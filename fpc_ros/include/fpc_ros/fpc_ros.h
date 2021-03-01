#pragma once

#include "ros/ros.h"

#include <mbf_costmap_core/costmap_controller.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_ros/buffer.h>

#include <memory>
#include <vector>
#include <string>
#include <XmlRpc.h>

#include <fpc_ros/data_classes/local_planner_robot_info.hpp>
#include <fpc_ros/data_classes/lyapunov_params.hpp>
#include <fpc_ros/fpc_controller_base.h>
#include <fpc_ros/fpc_controller_master.h>
#include <fpc_ros/fpc_controller_slave.h>

namespace fpc
{
	class FormationPathController : public mbf_costmap_core::CostmapController, public nav_core::BaseLocalPlanner
	{
		public:
			FormationPathController();


			/////////////////////////////////////////////////////////
			// Interface implementation
			/////////////////////////////////////////////////////////
			/**
			 * @brief Constructs the local planner
			 * @param name The name to give this instance of the local planner
			 * @param tf A pointer to a transform listener
			 * @param costmap_ros The cost map to use for assigning costs to local plans
			 */
			void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros) override;

			/**
			 * @brief  Set the plan that the local planner is following
			 * @param plan The plan to pass to the local planner
			 * @return True if the plan was updated successfully, false otherwise
			 */
			bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) override;

			/**
			 * @brief  Check if the goal pose has been achieved by the local planner
			 * @return True if achieved, false otherwise
			 */
			bool isGoalReached() override;

			/**
			 * @brief Check if the goal pose has been achieved by the local planner within tolerance limits
			 * @remark New on MBF API
			 * @param xy_tolerance Distance tolerance in meters
			 * @param yaw_tolerance Heading tolerance in radians
			 * @return True if achieved, false otherwise
			 */
			bool isGoalReached(double xy_tolerance, double yaw_tolerance) override;

			/**
			 * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
			 * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
			 * @return True if a valid velocity command was found, false otherwise
			 */
			bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;

			/**
			 * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands
			 * to send to the base.
			 * @param pose the current pose of the robot.
			 * @param velocity the current velocity of the robot.
			 * @param cmd_vel Will be filled with the velocity command to be passed to the robot base.
			 * @param message Optional more detailed outcome as a string
			 * @return Result code as described on ExePath action result:
			 *         SUCCESS         = 0
			 *         1..9 are reserved as plugin specific non-error results
			 *         FAILURE         = 100   Unspecified failure, only used for old, non-mfb_core based plugins
			 *         CANCELED        = 101
			 *         NO_VALID_CMD    = 102
			 *         PAT_EXCEEDED    = 103
			 *         COLLISION       = 104
			 *         OSCILLATION     = 105
			 *         ROBOT_STUCK     = 106
			 *         MISSED_GOAL     = 107
			 *         MISSED_PATH     = 108
			 *         BLOCKED_PATH    = 109
			 *         INVALID_PATH    = 110
			 *         TF_ERROR        = 111
			 *         NOT_INITIALIZED = 112
			 *         INVALID_PLUGIN  = 113
			 *         INTERNAL_ERROR  = 114
			 *         121..149 are reserved as plugin specific errors
			 */
			uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped &pose,
											 const geometry_msgs::TwistStamped &velocity,
											 geometry_msgs::TwistStamped &cmd_vel,
											 std::string &message) override;

			/**
			 * @brief Requests the planner to cancel, e.g. if it takes too much time
			 * @remark New on MBF API
			 * @return True if a cancel has been successfully requested, false if not implemented.
			 */
			bool cancel() override;

		private:
			#pragma region ControllerParams
			double xy_default_tolerance_;
			double yaw_default_tolerance_;
			std::vector<std::shared_ptr<fpc_data_classes::LocalPlannerRobotInfo>> robot_info_list_;
			std::shared_ptr<fpc_data_classes::LocalPlannerRobotInfo> current_robot_info_;
			#pragma endregion

			#pragma region ProcessMember
			ros::NodeHandle nh_;
			ros::NodeHandle controller_nh_;

			std::string robot_name_;
			std::string controller_name_;

			bool initialized_;

			std::shared_ptr<fpc::FPCControllerBase> fpc_controller_;

			int pose_index_;
			#pragma endregion

			////////////////////////////////////////////////
			// Private Helper Methods
			////////////////////////////////////////////////
			void getParams();
			double getNumberFromXMLRPC(XmlRpc::XmlRpcValue value, const std::string full_param_name);

			
			// Eigen::Vector2f getPosition(const geometry_msgs::Pose &pose);
			// Eigen::Vector3f getPose(const geometry_msgs::Pose &pose);
	};
}