#include <fpc_ros/fpc_controller_master.h>

namespace fpc
{
	FPCControllerMaster::FPCControllerMaster(
		std::shared_ptr<fpc_data_classes::FPCParamInfo> fpc_param_info,
		ros::NodeHandle &nh,
		ros::NodeHandle &controller_nh)
		: FPCControllerBase(fpc_param_info, nh, controller_nh)
	{ 
		this->pose_index_ = 0;

		for(std::shared_ptr<fpc_data_classes::FPCRobotInfo> &fpc_robot_info: this->fpc_param_info_->robot_info_list)
		{
			std::pair<std::string, fpp_msgs::NextTargetPoseCommand::Response> dummy;
			dummy.first = fpc_robot_info->robot_name;
			dummy.second.diff_after_next_pose.x = 0.0;
			dummy.second.diff_after_next_pose.y = 0.0;
			dummy.second.diff_after_next_pose.theta = 0.0;
			this->saved_command_res_list_.insert(dummy);
		}
	}

	bool FPCControllerMaster::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
	{
		// Insert value for master robot
		this->saved_command_res_list_[this->fpc_param_info_->getCurrentRobotName()].diff_after_next_pose =
			this->calcDiff(plan[0].pose, plan[1].pose);

		// Get values for slave robot
		while(!this->allSlavesReady())
		{
			for(auto &target_command_client: this->next_target_command_clt_list_)
			{
				fpp_msgs::NextTargetPoseCommand cmd;
				cmd.request.start_controller = false;
				if(!this->isSlaveReady(target_command_client.first)) // Only call slave that were not ready
				{
					if(target_command_client.second->call(cmd))
					{
						this->saved_command_res_list_[target_command_client.first] = cmd.response;
						ROS_ERROR_STREAM(target_command_client.first << " " << cmd.response.diff_after_next_pose);
					}
				}
			}
			ros::Duration(0.1).sleep();
		}

		this->next_target_pose_ = ros::Time::now();
		return FPCControllerBase::setPlan(plan);
	}

	void FPCControllerMaster::runController()
	{
		// if(this->next_target_pose_ < ros::Time::now())
		if(this->pose_index_ < (this->locateRobotOnPath(this->current_robot_amcl_pose_)+1))
		{
			// this->pose_index_++;
			this->pose_index_ = this->locateRobotOnPath(this->current_robot_amcl_pose_)+1;

			float largest_pose_diff = this->getLargestDiff();

			for(auto &client: this->next_target_command_clt_list_)
			{
				fpp_msgs::NextTargetPoseCommand cmd_msg;
				cmd_msg.request.start_controller = true;
				cmd_msg.request.next_target_pose_index = this->pose_index_;
				float eucl_dist = std::sqrt(std::pow(this->saved_command_res_list_[client.first].diff_after_next_pose.x, 2) +
											std::pow(this->saved_command_res_list_[client.first].diff_after_next_pose.y, 2));
				cmd_msg.request.velocity_factor = eucl_dist / largest_pose_diff;
				client.second->call(cmd_msg);
				this->saved_command_res_list_[client.first] = cmd_msg.response;
			}

			float necesary_seconds = largest_pose_diff / this->fpc_param_info_->controller_params.max_vel_x;
			this->next_target_pose_ = ros::Time::now() + ros::Duration(necesary_seconds);

			this->velocity_factor_ =
				std::sqrt(std::pow(this->saved_command_res_list_[this->fpc_param_info_->getCurrentRobotName()].diff_after_next_pose.x, 2) +
						  std::pow(this->saved_command_res_list_[this->fpc_param_info_->getCurrentRobotName()].diff_after_next_pose.y, 2)) /
				largest_pose_diff;

			geometry_msgs::Pose2D test = this->calcDiff(this->global_plan_[this->pose_index_].pose, this->global_plan_[this->pose_index_ + 1].pose);
			this->saved_command_res_list_[this->fpc_param_info_->getCurrentRobotName()].diff_after_next_pose = test;
				
		}

		geometry_msgs::Pose2D diff_vector = this->calcDiff(this->current_robot_amcl_pose_,
														   this->global_plan_[this->pose_index_].pose);
		// ROS_ERROR_STREAM(this->fpc_param_info_->getCurrentRobotName() << " " << diff_vector.x << " " << diff_vector.y << " " << diff_vector.theta);
		float output_v = this->calcLinVelocity(diff_vector, this->velocity_factor_);
		float output_omega = this->calcRotVelocity(diff_vector);

		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = output_v;
		cmd_vel.linear.y = 0.0;
		cmd_vel.linear.z = 0.0;
		cmd_vel.angular.x = 0.0;
		cmd_vel.angular.y = 0.0;
		cmd_vel.angular.z = output_omega;
		// ROS_INFO_STREAM("x: " << cmd_vel.linear.x << " rot: " << cmd_vel.angular.z);

		this->meta_data_msg_.target_vel = cmd_vel;

		this->cmd_vel_publisher_.publish(cmd_vel);
		this->last_published_cmd_vel_ = cmd_vel;

		this->meta_data_msg_.velocity_factor = this->velocity_factor_;
		this->publishMetaData(this->convertPose(this->global_plan_[this->pose_index_].pose));

		#pragma region Old code
		// float period_duration = 1.0 / this->fpc_param_info_->controller_params.controller_frequency;

		// // Calculate current position of robot on path and next target pose
		// int current_pose_index = this->locateRobotOnPath(this->current_robot_amcl_pose_);
		// ROS_ERROR_STREAM(this->fpc_param_info_->getCurrentRobotName() << " index: " << current_pose_index);
		// int next_target_pose_index = current_pose_index + 1;
		// if(next_target_pose_index >= this->global_plan_.size())
		// {
		// 	next_target_pose_index = this->global_plan_.size();
		// }

		// geometry_msgs::PoseStamped next_target_pose = this->global_plan_[next_target_pose_index];

		// tf::Pose current_pose_tf;
		// tf::poseMsgToTF(this->current_robot_amcl_pose_, current_pose_tf);
		// tf::Pose next_target_pose_tf;
		// tf::poseMsgToTF(next_target_pose.pose, next_target_pose_tf);
		// tf::Transform pose_diff_tf=current_pose_tf.inverseTimes(next_target_pose_tf);
		// Eigen::Vector3f pose_diff;
		// pose_diff << pose_diff_tf.getOrigin().getX(),
		// 	pose_diff_tf.getOrigin().getY(),
		// 	tf::getYaw(pose_diff_tf.getRotation());

		// Eigen::Vector2f euclidean_pose_diff;
		// euclidean_pose_diff << std::sqrt(std::pow(pose_diff[0], 2) + std::pow(pose_diff[1], 2)), pose_diff[2];

		// ROS_ERROR_STREAM("Diff x: " << pose_diff_tf.getOrigin().getX() << " y: " << pose_diff_tf.getOrigin().getY() << " rot: " << tf::getYaw(pose_diff_tf.getRotation()));

		// // This is the theoretical velocity that the rebot must drive to reach its goal within the period time
		// // Scaling to cap it at the max values happens after
		// Eigen::Vector2f theoretical_vel;
		// theoretical_vel << euclidean_pose_diff[0] / period_duration,
		// 	euclidean_pose_diff[1] / period_duration;

		// if(theoretical_vel[0] > 0)
		// {
		// 	if(theoretical_vel[0] > this->fpc_param_info_->controller_params.max_vel_x)
		// 	{
		// 		theoretical_vel[0] = this->fpc_param_info_->controller_params.max_vel_x;
		// 	}
		// }
		// else // This should be irrelevant
		// {
		// 	if(theoretical_vel[0] < this->fpc_param_info_->controller_params.min_vel_x)
		// 	{
		// 		theoretical_vel[0] = this->fpc_param_info_->controller_params.min_vel_x;
		// 	}
		// }

		// float traveled_lin_distance = theoretical_vel[0] * period_duration;
		// float percentage_done = traveled_lin_distance / euclidean_pose_diff[0];
		// ROS_ERROR_STREAM("lin vel: " << theoretical_vel[0] << " dist: " << traveled_lin_distance << " percentage: " << percentage_done);

		// theoretical_vel[1] = (euclidean_pose_diff[1] * percentage_done) / period_duration;
		// ROS_ERROR_STREAM("rot_vel: " << theoretical_vel[1]);

		// if(std::abs(theoretical_vel[1]) > this->fpc_param_info_->controller_params.max_vel_theta)
		// {
		// 	if(theoretical_vel[1] > 0)
		// 	{
		// 		theoretical_vel[1] = this->fpc_param_info_->controller_params.max_vel_theta;
		// 	}
		// 	else
		// 	{
		// 		theoretical_vel[1] = -this->fpc_param_info_->controller_params.max_vel_theta;
		// 	}
		// }
		// float traveled_rot_distance = theoretical_vel[1] * period_duration;

		// ROS_ERROR_STREAM("capped rot_vel: " << theoretical_vel[1] << " traveled rot: " << traveled_rot_distance);

		// float output_v = this->fpc_param_info_->getLyapunovParams().kx * traveled_lin_distance*cos(traveled_rot_distance) +
		// 				 theoretical_vel[0] * cos(traveled_rot_distance);

		// float output_omega = theoretical_vel[1] +
		// 					 this->fpc_param_info_->getLyapunovParams().ky * theoretical_vel[0] * traveled_lin_distance * sin(traveled_rot_distance) +
		// 					 this->fpc_param_info_->getLyapunovParams().kphi * sin(traveled_rot_distance);

		// // Calculate controller output with Maltes control law
		// // float output_v = this->fpc_param_info_->getLyapunovParams().kx * pose_diff_tf.getOrigin().getX() +
		// // 				 theoretical_vel[0] * cos(tf::getYaw(pose_diff_tf.getRotation()));

		// // float output_omega = theoretical_vel[1] +
		// // 			   this->fpc_param_info_->getLyapunovParams().ky * theoretical_vel[0] * pose_diff_tf.getOrigin().getY() +
		// // 			   this->fpc_param_info_->getLyapunovParams().kphi * sin(tf::getYaw(pose_diff_tf.getRotation()));

		// // float lin_vel_scale = 1.0;
		// // float rot_vel_scale = 1.0;
		// // // Differ between backwards and forwards because backwards might be slower than forwards
		// // if(output_v > 0.0)
		// // {
		// // 	lin_vel_scale = this->fpc_param_info_->controller_params.max_vel_x / output_v;
		// // }
		// // else
		// // {
		// // 	lin_vel_scale = this->fpc_param_info_->controller_params.min_vel_x / output_v;
		// // }
		// // if(lin_vel_scale < 1.0)
		// // {
		// // 	output_v = output_v * lin_vel_scale;
		// // }
		
		// // // Use absolute of theoretical rot vel as left or right turning is equal.
		// // // THIS MIGHT BE A MISTAKE! IT SHOULD BE BETTER IF THE ROBOT IS ORIENTATED CORRECTLY!
		// // rot_vel_scale = this->fpc_param_info_->controller_params.max_vel_theta / std::abs(output_omega);
		// // if(rot_vel_scale < 1.0)
		// // {
		// // 	output_omega = output_omega * rot_vel_scale;
		// // }


		// geometry_msgs::Twist cmd_vel;
		// cmd_vel.linear.x = output_v;
		// cmd_vel.linear.y = 0.0;
		// cmd_vel.linear.z = 0.0;
		// cmd_vel.angular.x = 0.0;
		// cmd_vel.angular.y = 0.0;
		// cmd_vel.angular.z = output_omega;
		// this->last_published_cmd_vel_ = cmd_vel;

		// ROS_INFO_STREAM("x: " << cmd_vel.linear.x << " rot: " << cmd_vel.angular.z);

		// this->cmd_vel_publisher_.publish(cmd_vel);

		// // Compare scale values to other robots scale values
		// // 1. Ist ein Roboter hinterher? Also ist current pose von einem Roboter -1 oder mehr als der hier.
		// // Wenn ja muss ich den aktuellen Roboter verlangsamen. Einfach 50% langsamer als max vel?
		// // 2. Ist ein Roboter eine Position weiter vorne? -> max. vel.
		// // 3. Sind alle Roboter an der gleichen current pose. Dann schaue ob der größte Scale Faktor größer ist als der,
		// // den ich hier ausgerechnet habe und wende ihn auf meine theoretical velocity an
		#pragma endregion
	}

	void FPCControllerMaster::initServices()
	{
		for(std::shared_ptr<fpc_data_classes::FPCRobotInfo> &fpc_robot_info: this->fpc_param_info_->robot_info_list)
		{
			if(fpc_robot_info->robot_name != this->fpc_param_info_->getCurrentRobotName())
			{
				std::pair<std::string, std::shared_ptr<ros::ServiceClient>> command_client;
				command_client.first = fpc_robot_info->robot_name;
				ros::ServiceClient tmp_client = this->nh_.serviceClient<fpp_msgs::NextTargetPoseCommand>(
					"/" + fpc_robot_info->robot_namespace + "/move_base_flex/FormationPathController/next_target_pose");
				command_client.second = std::make_shared<ros::ServiceClient>(tmp_client);
				command_client.second->waitForExistence();
				this->next_target_command_clt_list_.insert(command_client);
			}
		}
	}

	#pragma region Callback Methods
	#pragma endregion

	#pragma region Helper Methods
	bool FPCControllerMaster::allSlavesReady()
	{
		for(auto &saved_response: this->saved_command_res_list_)
		{
			if (saved_response.second.diff_after_next_pose.x == 0.0 &&
				saved_response.second.diff_after_next_pose.y == 0.0 &&
				saved_response.second.diff_after_next_pose.theta == 0.0)
			{
				return false;
			}
		}
		return true;
	}

	bool FPCControllerMaster::isSlaveReady(std::string robot_name)
	{
		if (this->saved_command_res_list_[robot_name].diff_after_next_pose.x == 0.0 &&
			this->saved_command_res_list_[robot_name].diff_after_next_pose.y == 0.0 &&
			this->saved_command_res_list_[robot_name].diff_after_next_pose.theta == 0.0)
		{
			return false;
		}
		return true;
	}

	float FPCControllerMaster::getLargestDiff()
	{
		float greatest_distance = 0.0;
		for(auto &cmd_res: this->saved_command_res_list_)
		{
			float euclidean_dist = std::sqrt(std::pow(cmd_res.second.diff_after_next_pose.x, 2) +
											 std::pow(cmd_res.second.diff_after_next_pose.y, 2));
			if(euclidean_dist > greatest_distance)
			{
				greatest_distance = euclidean_dist;
			}
		}

		return greatest_distance;
	}

	// int FPCControllerMaster::getHighestPoseIndex()
	// {
	// 	int highest_pose_index = this->robot_scale_info_list_.begin()->second.current_pose_index;
	// 	for (std::map<std::string, fpp_msgs::FPCRobotScaleInfo>::iterator it = this->robot_scale_info_list_.begin();
	// 		 it != this->robot_scale_info_list_.end(); it++)
	// 	{
	// 		if(it->second.current_pose_index > highest_pose_index)
	// 		{
	// 			highest_pose_index = it->second.current_pose_index;
	// 		}
	// 	}

	// 	return highest_pose_index;
	// }

	// int FPCControllerMaster::getLowestPoseIndex()
	// {
	// 	int lowest_pose_index = this->robot_scale_info_list_.begin()->second.current_pose_index;
	// 	for (std::map<std::string, fpp_msgs::FPCRobotScaleInfo>::iterator it = this->robot_scale_info_list_.begin();
	// 		 it != this->robot_scale_info_list_.end(); it++)
	// 	{
	// 		if(it->second.current_pose_index < lowest_pose_index)
	// 		{
	// 			lowest_pose_index = it->second.current_pose_index;
	// 		}
	// 	}

	// 	return lowest_pose_index;
	// }

	// fpp_msgs::FPCRobotScaleInfo FPCControllerMaster::getHighestLinScaleValue(int next_target_pose)
	// {
	// 	fpp_msgs::FPCRobotScaleInfo highest_robot_scale_info = this->robot_scale_info_list_.begin()->second;
	// 	for (std::map<std::string, fpp_msgs::FPCRobotScaleInfo>::iterator it = this->robot_scale_info_list_.begin();
	// 		 it != this->robot_scale_info_list_.end(); it++)
	// 	{
	// 		if (it->second.next_target_pose_index == next_target_pose &&
	// 			it->second.lin_vel_scale > highest_robot_scale_info.lin_vel_scale)
	// 		{
	// 			highest_robot_scale_info = it->second;
	// 		}
	// 	}

	// 	return highest_robot_scale_info;
	// }
	#pragma endregion
}