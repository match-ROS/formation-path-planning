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

	void FormationPathController::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
	{
		ROS_ERROR_STREAM("initialize");
	}

	bool FormationPathController::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
	{
		ROS_ERROR_STREAM("setPlan");
		// Save the complete global plan
		this->global_plan_ = std::make_shared<std::vector<geometry_msgs::PoseStamped>>(plan);

		// Save start position separatly
		this->start_pose_.header = this->global_plan_

		//Save end position separatly
		return true;
	}

	bool FormationPathController::isGoalReached()
	{
		return this->isGoalReached(this->xy_default_tolerance_, this->yaw_default_tolerance_);
	}

	bool FormationPathController::isGoalReached(double xy_tolerance, double yaw_tolerance)
	{
		// Glaube ich muss hier auch etwas übergreifendes implementieren, falls der Master meldet Ziel ist erreicht, dann müssen die anderen ROboter das auch melden
		// Außer ich will vllt noch versuchen den restlichen Regelfehler auszugleichen, aber da sollte ja keiner sein

		ROS_ERROR_STREAM("isGoalReached2");
		return false;
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

		return 0;
	}

	bool FormationPathController::cancel()
	{
		ROS_ERROR_STREAM("cancel");
	}
}