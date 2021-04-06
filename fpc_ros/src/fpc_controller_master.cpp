#include <fpc_ros/fpc_controller_master.h>

namespace fpc
{
	FPCControllerMaster::FPCControllerMaster(
		std::shared_ptr<fpc_data_classes::FPCParamInfo> fpc_param_info,
		ros::NodeHandle &nh,
		ros::NodeHandle &controller_nh)
		: FPCControllerBase(fpc_param_info, nh, controller_nh)
	{
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

		this->initServices();
	}

	void FPCControllerMaster::run_controller()
	{
		float period_duration = 1.0 / this->fpc_param_info_->controller_params.controller_frequency;

		int current_pose_index = this->locateRobotOnPath(this->current_robot_amcl_pose_);
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
		Eigen::Vector2f euclidean_pose_diff;
		euclidean_pose_diff << std::sqrt(std::pow(pose_diff_tf.getOrigin().getX(), 2) +
										 std::pow(pose_diff_tf.getOrigin().getY(), 2)),
			tf::getYaw(pose_diff_tf.getRotation());

		Eigen::Vector2f theoretical_vel;
		theoretical_vel << euclidean_pose_diff[0] / period_duration,
			euclidean_pose_diff[1] / period_duration;

		Eigen::Vector2f scale_vector;
		scale_vector << 1.0, 1.0;
		
		
	}

	void FPCControllerMaster::initServices()
	{
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
}