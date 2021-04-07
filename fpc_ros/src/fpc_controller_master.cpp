#include <fpc_ros/fpc_controller_master.h>

namespace fpc
{
	FPCControllerMaster::FPCControllerMaster(
		std::shared_ptr<fpc_data_classes::FPCParamInfo> fpc_param_info,
		ros::NodeHandle &nh,
		ros::NodeHandle &controller_nh)
		: FPCControllerBase(fpc_param_info, nh, controller_nh)
	{
		ROS_ERROR_STREAM("FPC Constructor of " << fpc_param_info->getCurrentRobotName());
		// Initialize the robot_scale_info map with an entry for each robot
		for(std::shared_ptr<fpc_data_classes::FPCRobotInfo> &robot_info : this->fpc_param_info_->robot_info_list)
		{
			std::pair<std::string, fpp_msgs::FPCRobotScaleInfo> robot_scale_info;
			robot_scale_info.first = robot_info->robot_name;
			
			fpp_msgs::FPCRobotScaleInfo dummy_robot_scale_info;
			dummy_robot_scale_info.robot_name = robot_info->robot_name;
			dummy_robot_scale_info.current_pose_index = 0;
			dummy_robot_scale_info.next_target_pose_index = 1;
			dummy_robot_scale_info.distance_to_next_target = 0.0;
			dummy_robot_scale_info.lin_vel_scale = 1.0;
			dummy_robot_scale_info.rot_vel_scale = 1.0;

			this->robot_scale_info_list_.insert(robot_scale_info);
		}
	}

	void FPCControllerMaster::run_controller()
	{
		float period_duration = 1.0 / this->fpc_param_info_->controller_params.controller_frequency;

		// Calculate current position of robot on path and next target pose
		int current_pose_index = this->locateRobotOnPath(this->current_robot_amcl_pose_);
		ROS_ERROR_STREAM(this->fpc_param_info_->getCurrentRobotName() << " index: " << current_pose_index);
		int next_target_pose_index = current_pose_index + 1;
		if(next_target_pose_index >= this->global_plan_.size())
		{
			next_target_pose_index = this->global_plan_.size();
		}

		geometry_msgs::PoseStamped next_target_pose = this->global_plan_[next_target_pose_index];

		tf::Pose current_pose_tf;
		tf::poseMsgToTF(this->current_robot_amcl_pose_, current_pose_tf);
		tf::Pose next_target_pose_tf;
		tf::poseMsgToTF(next_target_pose.pose, next_target_pose_tf);
		tf::Transform pose_diff_tf=current_pose_tf.inverseTimes(next_target_pose_tf);
		Eigen::Vector3f pose_diff;
		pose_diff << pose_diff_tf.getOrigin().getX(),
			pose_diff_tf.getOrigin().getY(),
			tf::getYaw(pose_diff_tf.getRotation());

		Eigen::Vector2f euclidean_pose_diff;
		euclidean_pose_diff << std::sqrt(std::pow(pose_diff[0], 2) + std::pow(pose_diff[1], 2)), pose_diff[2];

		ROS_ERROR_STREAM("Diff x: " << pose_diff_tf.getOrigin().getX() << " y: " << pose_diff_tf.getOrigin().getY() << " rot: " << tf::getYaw(pose_diff_tf.getRotation()));

		// This is the theoretical velocity that the rebot must drive to reach its goal within the period time
		// Scaling to cap it at the max values happens after
		Eigen::Vector2f theoretical_vel;
		theoretical_vel << euclidean_pose_diff[0] / period_duration,
			euclidean_pose_diff[1] / period_duration;

		if(theoretical_vel[0] > 0)
		{
			if(theoretical_vel[0] > this->fpc_param_info_->controller_params.max_vel_x)
			{
				theoretical_vel[0] = this->fpc_param_info_->controller_params.max_vel_x;
			}
		}
		else // This should be irrelevant
		{
			if(theoretical_vel[0] < this->fpc_param_info_->controller_params.min_vel_x)
			{
				theoretical_vel[0] = this->fpc_param_info_->controller_params.min_vel_x;
			}
		}

		float traveled_lin_distance = theoretical_vel[0] * period_duration;
		float percentage_done = traveled_lin_distance / euclidean_pose_diff[0];
		ROS_ERROR_STREAM("lin vel: " << theoretical_vel[0] << " dist: " << traveled_lin_distance << " percentage: " << percentage_done);

		theoretical_vel[1] = (euclidean_pose_diff[1] * percentage_done) / period_duration;
		ROS_ERROR_STREAM("rot_vel: " << theoretical_vel[1]);

		if(std::abs(theoretical_vel[1]) > this->fpc_param_info_->controller_params.max_vel_theta)
		{
			if(theoretical_vel[1] > 0)
			{
				theoretical_vel[1] = this->fpc_param_info_->controller_params.max_vel_theta;
			}
			else
			{
				theoretical_vel[1] = -this->fpc_param_info_->controller_params.max_vel_theta;
			}
		}
		float traveled_rot_distance = theoretical_vel[1] * period_duration;

		ROS_ERROR_STREAM("capped rot_vel: " << theoretical_vel[1] << " traveled rot: " << traveled_rot_distance);

		float output_v = this->fpc_param_info_->getLyapunovParams().kx * traveled_lin_distance*cos(traveled_rot_distance) +
						 theoretical_vel[0] * cos(traveled_rot_distance);

		float output_omega = theoretical_vel[1] +
							 this->fpc_param_info_->getLyapunovParams().ky * theoretical_vel[0] * traveled_lin_distance * sin(traveled_rot_distance) +
							 this->fpc_param_info_->getLyapunovParams().kphi * sin(traveled_rot_distance);

		// Calculate controller output with Maltes control law
		// float output_v = this->fpc_param_info_->getLyapunovParams().kx * pose_diff_tf.getOrigin().getX() +
		// 				 theoretical_vel[0] * cos(tf::getYaw(pose_diff_tf.getRotation()));

		// float output_omega = theoretical_vel[1] +
		// 			   this->fpc_param_info_->getLyapunovParams().ky * theoretical_vel[0] * pose_diff_tf.getOrigin().getY() +
		// 			   this->fpc_param_info_->getLyapunovParams().kphi * sin(tf::getYaw(pose_diff_tf.getRotation()));

		// float lin_vel_scale = 1.0;
		// float rot_vel_scale = 1.0;
		// // Differ between backwards and forwards because backwards might be slower than forwards
		// if(output_v > 0.0)
		// {
		// 	lin_vel_scale = this->fpc_param_info_->controller_params.max_vel_x / output_v;
		// }
		// else
		// {
		// 	lin_vel_scale = this->fpc_param_info_->controller_params.min_vel_x / output_v;
		// }
		// if(lin_vel_scale < 1.0)
		// {
		// 	output_v = output_v * lin_vel_scale;
		// }
		
		// // Use absolute of theoretical rot vel as left or right turning is equal.
		// // THIS MIGHT BE A MISTAKE! IT SHOULD BE BETTER IF THE ROBOT IS ORIENTATED CORRECTLY!
		// rot_vel_scale = this->fpc_param_info_->controller_params.max_vel_theta / std::abs(output_omega);
		// if(rot_vel_scale < 1.0)
		// {
		// 	output_omega = output_omega * rot_vel_scale;
		// }


		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = output_v;
		cmd_vel.linear.y = 0.0;
		cmd_vel.linear.z = 0.0;
		cmd_vel.angular.x = 0.0;
		cmd_vel.angular.y = 0.0;
		cmd_vel.angular.z = output_omega;
		this->last_published_cmd_vel_ = cmd_vel;

		ROS_INFO_STREAM("x: " << cmd_vel.linear.x << " rot: " << cmd_vel.angular.z);

		this->cmd_vel_publisher_.publish(cmd_vel);

		// Compare scale values to other robots scale values
		// 1. Ist ein Roboter hinterher? Also ist current pose von einem Roboter -1 oder mehr als der hier.
		// Wenn ja muss ich den aktuellen Roboter verlangsamen. Einfach 50% langsamer als max vel?
		// 2. Ist ein Roboter eine Position weiter vorne? -> max. vel.
		// 3. Sind alle Roboter an der gleichen current pose. Dann schaue ob der größte Scale Faktor größer ist als der,
		// den ich hier ausgerechnet habe und wende ihn auf meine theoretical velocity an
	}

	void FPCControllerMaster::initServices()
	{
		ROS_ERROR_STREAM("init Services " << this->fpc_param_info_->getCurrentRobotName());
		this->fpc_vel_scale_info_srv_ = this->controller_nh_.advertiseService(
			"fpc_vel_scale_info",
			&FPCControllerMaster::onFPCVelScaleInfo, this);
	}

	#pragma region Callback Methods
	bool FPCControllerMaster::onFPCVelScaleInfo(fpp_msgs::FPCVelScaleInfo::Request &req,
												fpp_msgs::FPCVelScaleInfo::Response &res)
	{
		// Store new scaling of slave robot
		this->robot_scale_info_list_[req.robot_scale_info.robot_name] = req.robot_scale_info;

		// Return all scaling to slave so it can adjust its velocity
		for (std::map<std::string, fpp_msgs::FPCRobotScaleInfo>::iterator it = this->robot_scale_info_list_.begin();
			 it != this->robot_scale_info_list_.end(); it++)
		{
			res.robot_scale_info_list.push_back(it->second);
		}

		return true;
	}
	#pragma endregion

	#pragma region Helper Methods
	int FPCControllerMaster::getHighestPoseIndex()
	{
		int highest_pose_index = this->robot_scale_info_list_.begin()->second.current_pose_index;
		for (std::map<std::string, fpp_msgs::FPCRobotScaleInfo>::iterator it = this->robot_scale_info_list_.begin();
			 it != this->robot_scale_info_list_.end(); it++)
		{
			if(it->second.current_pose_index > highest_pose_index)
			{
				highest_pose_index = it->second.current_pose_index;
			}
		}

		return highest_pose_index;
	}

	int FPCControllerMaster::getLowestPoseIndex()
	{
		int lowest_pose_index = this->robot_scale_info_list_.begin()->second.current_pose_index;
		for (std::map<std::string, fpp_msgs::FPCRobotScaleInfo>::iterator it = this->robot_scale_info_list_.begin();
			 it != this->robot_scale_info_list_.end(); it++)
		{
			if(it->second.current_pose_index < lowest_pose_index)
			{
				lowest_pose_index = it->second.current_pose_index;
			}
		}

		return lowest_pose_index;
	}

	fpp_msgs::FPCRobotScaleInfo FPCControllerMaster::getHighestLinScaleValue(int next_target_pose)
	{
		fpp_msgs::FPCRobotScaleInfo highest_robot_scale_info = this->robot_scale_info_list_.begin()->second;
		for (std::map<std::string, fpp_msgs::FPCRobotScaleInfo>::iterator it = this->robot_scale_info_list_.begin();
			 it != this->robot_scale_info_list_.end(); it++)
		{
			if (it->second.next_target_pose_index == next_target_pose &&
				it->second.lin_vel_scale > highest_robot_scale_info.lin_vel_scale)
			{
				highest_robot_scale_info = it->second;
			}
		}

		return highest_robot_scale_info;
	}
	#pragma endregion
}