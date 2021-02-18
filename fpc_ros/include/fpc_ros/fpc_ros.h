#pragma once

#include "ros/ros.h"

#include <mbf_costmap_core/costmap_controller.h>
#include <nav_core/base_local_planner.h>

namespace fpc
{
	class FormationPathController : public mbf_costmap_core::CostmapController, public nav_core::BaseLocalPlanner
	{
		public:
			FormationPathController();

			/**
			 * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
			 * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
			 * @return True if a valid velocity command was found, false otherwise
			 */
			bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

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
											 std::string &message);

			/**
			 * @brief  Check if the goal pose has been achieved by the local planner
			 * @return True if achieved, false otherwise
			 */
			bool isGoalReached();

			/**
			 * @brief Check if the goal pose has been achieved by the local planner within tolerance limits
			 * @remark New on MBF API
			 * @param xy_tolerance Distance tolerance in meters
			 * @param yaw_tolerance Heading tolerance in radians
			 * @return True if achieved, false otherwise
			 */
			bool isGoalReached(double xy_tolerance, double yaw_tolerance);

			/**
			 * @brief  Set the plan that the local planner is following
			 * @param plan The plan to pass to the local planner
			 * @return True if the plan was updated successfully, false otherwise
			 */
			bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan);

			/**
			 * @brief Requests the planner to cancel, e.g. if it takes too much time
			 * @remark New on MBF API
			 * @return True if a cancel has been successfully requested, false if not implemented.
			 */
			bool cancel();

			/**
			 * @brief Constructs the local planner
			 * @param name The name to give this instance of the local planner
			 * @param tf A pointer to a transform listener
			 * @param costmap_ros The cost map to use for assigning costs to local plans
			 */
			void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros);
	};
}