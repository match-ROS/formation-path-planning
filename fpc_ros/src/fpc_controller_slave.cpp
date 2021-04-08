#include <fpc_ros/fpc_controller_slave.h>

namespace fpc
{
	FPCControllerSlave::FPCControllerSlave(
		std::shared_ptr<fpc_data_classes::FPCParamInfo> fpc_param_info,
		ros::NodeHandle &nh,
		ros::NodeHandle &controller_nh)
		: FPCControllerBase(fpc_param_info, nh, controller_nh)
	{
		// Init saved target cmd request
		this->saved_target_cmd_req_.start_controller = false;
		this->saved_target_cmd_req_.next_target_pose_index = 0;
		this->saved_target_cmd_req_.velocity_factor = 0.0;
	}

	void FPCControllerSlave::initServices()
	{
		this->next_target_command_srv_ = this->controller_nh_.advertiseService(
			"next_target_pose",
			&FPCControllerSlave::onNextTargetCommand,
			this);
	}

	void FPCControllerSlave::runController()
	{
		if(!this->saved_target_cmd_req_.start_controller)
		{
			return;
		}

		geometry_msgs::Pose2D diff_vector = this->calcDiff(this->current_robot_amcl_pose_,
														   this->global_plan_[this->saved_target_cmd_req_.next_target_pose_index].pose);
		float output_v = this->calcLinVelocity(diff_vector, this->saved_target_cmd_req_.velocity_factor);
		float output_omega = this->calcRotVelocity(diff_vector);

		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = output_v;
		cmd_vel.linear.y = 0.0;
		cmd_vel.linear.z = 0.0;
		cmd_vel.angular.x = 0.0;
		cmd_vel.angular.y = 0.0;
		cmd_vel.angular.z = output_omega;

		this->meta_data_msg_.target_vel = cmd_vel;
		this->last_published_cmd_vel_ = cmd_vel;

		this->cmd_vel_publisher_.publish(cmd_vel);

		this->meta_data_msg_.velocity_factor = this->saved_target_cmd_req_.velocity_factor;
		this->publishMetaData(this->convertPose(this->global_plan_[this->saved_target_cmd_req_.next_target_pose_index].pose));
	}

	#pragma region Callback Method
	bool FPCControllerSlave::onNextTargetCommand(fpp_msgs::NextTargetPoseCommand::Request &req,
												 fpp_msgs::NextTargetPoseCommand::Response &res)
	{
		if(global_plan_.size() == 0)
		{
			res.diff_after_next_pose.x = 0.0;
			res.diff_after_next_pose.y = 0.0;
			res.diff_after_next_pose.theta = 0.0;
			return false;
		}

		res.diff_after_next_pose = this->calcDiff(this->global_plan_[0].pose, this->global_plan_[1].pose);

		if(!req.start_controller)
		{
			// This is the first call. Dont save the request, just report the distance to first pose on path
			return true;
		}

		this->saved_target_cmd_req_ = req;
		return true;
	}
	#pragma endregion
}