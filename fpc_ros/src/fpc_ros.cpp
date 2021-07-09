#include <fpc_ros/fpc_ros.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(fpc::FormationPathController, mbf_costmap_core::CostmapController)
PLUGINLIB_EXPORT_CLASS(fpc::FormationPathController, nav_core::BaseLocalPlanner)

namespace fpc
{
	FormationPathController::FormationPathController()
		: initialized_(false), pose_index_(1)
	{
		ROS_ERROR_STREAM("Constructor");
	}

	void FormationPathController::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
	{
		if(!this->initialized_)
		{
			ROS_ERROR_STREAM("initialize");

			// Store all process info
			this->controller_name_ = name;
			
			this->nh_ = ros::NodeHandle();
			this->controller_nh_ = ros::NodeHandle("~/" + this->controller_name_);
		
			// Read and store all params that are set by the yaml config
			this->fpc_param_manager_ = std::make_shared<fpc_data_classes::FPCParamManager>(this->nh_,
																						   this->controller_nh_);
			this->fpc_param_manager_->getParams();
			this->fpc_param_info_ = this->fpc_param_manager_->getLocalPlannerInfo();

			// Create controller object depenend on if the robot is master or slave in the formation
			if(this->fpc_param_manager_->getLocalPlannerInfo()->current_robot_info->fpc_master)
			{
				this->fpc_controller_ = std::make_shared<fpc::FPCControllerMaster>(this->fpc_param_info_,
																				   this->nh_,
																				   this->controller_nh_);
			}
			else
			{
				this->fpc_controller_ = std::make_shared<fpc::FPCControllerSlave>(this->fpc_param_info_,
																				  this->nh_,
																				  this->controller_nh_);
			}

			this->fpc_controller_->initialize(this->controller_name_, tf, costmap_ros);

			this->initialized_ = true;
			ROS_INFO_STREAM("FormationPathController::initialize: FPC has been succesfully initialized");
		}
		else
		{
			ROS_WARN_STREAM("FormationPathController::initialize: FPC has already been initialized");
		}
	}

	bool FormationPathController::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
	{
		return this->fpc_controller_->setPlan(plan);
	}

	bool FormationPathController::isGoalReached()
	{
		return this->fpc_controller_->isGoalReached(this->fpc_param_info_->xy_default_tolerance,
													this->fpc_param_info_->yaw_default_tolerance);
	}

	bool FormationPathController::isGoalReached(double xy_tolerance, double yaw_tolerance)
	{
		return this->fpc_controller_->isGoalReached(xy_tolerance, yaw_tolerance);
	}

	bool FormationPathController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
	{
		return this->fpc_controller_->computeVelocityCommands(cmd_vel);
	}

	uint32_t FormationPathController::computeVelocityCommands(const geometry_msgs::PoseStamped &pose,
															  const geometry_msgs::TwistStamped &velocity,
															  geometry_msgs::TwistStamped &cmd_vel,
															  std::string &message)
	{
		return this->fpc_controller_->computeVelocityCommands(pose, velocity, cmd_vel, message);
	}

	bool FormationPathController::cancel()
	{
		ROS_ERROR_STREAM("cancel");
	}
}