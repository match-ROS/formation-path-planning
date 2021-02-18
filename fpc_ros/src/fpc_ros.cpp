#include <fpc_ros/fpc_ros.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(fpc::FormationPathController, mbf_costmap_core::CostmapController)
PLUGINLIB_EXPORT_CLASS(fpc::FormationPathController, nav_core::BaseLocalPlanner)

namespace fpc
{
	FormationPathController::FormationPathController()
	{
		ROS_ERROR_STREAM("Constructor");
	}

	bool FormationPathController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
	{
		ROS_ERROR_STREAM("computeVelocityCommands1");
	}

	uint32_t FormationPathController::computeVelocityCommands(const geometry_msgs::PoseStamped &pose,
															  const geometry_msgs::TwistStamped &velocity,
															  geometry_msgs::TwistStamped &cmd_vel,
															  std::string &message)
	{
		ROS_ERROR_STREAM("computeVelocityCommands2");
	}

	bool FormationPathController::isGoalReached()
	{
		ROS_ERROR_STREAM("isGoalReached1");
	}

	bool FormationPathController::isGoalReached(double xy_tolerance, double yaw_tolerance)
	{
		ROS_ERROR_STREAM("isGoalReached2");
	}

	bool FormationPathController::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
	{
		ROS_ERROR_STREAM("setPlan");
		ROS_ERROR_STREAM("size: " << plan.size());
		return true;
	}

	bool FormationPathController::cancel()
	{
		ROS_ERROR_STREAM("cancel");
	}

	void FormationPathController::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
	{
		ROS_ERROR_STREAM("initialize");
	}
}