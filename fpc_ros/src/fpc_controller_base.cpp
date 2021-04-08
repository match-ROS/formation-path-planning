#include <fpc_ros/fpc_controller_base.h>

namespace fpc
{
	#pragma region Constructors
	FPCControllerBase::FPCControllerBase(
		std::shared_ptr<fpc_data_classes::FPCParamInfo> fpc_param_info,
		ros::NodeHandle &nh,
		ros::NodeHandle &controller_nh)
		: fpc_param_info_(fpc_param_info),
		  nh_(nh),
		  controller_nh_(controller_nh),
		  pose_index_(1),
		  controller_finished_(false)
	{
	}
	#pragma endregion

	#pragma region ControllerInterface
	void FPCControllerBase::initialize(std::string controller_name,
									   tf2_ros::Buffer *tf,
									   costmap_2d::Costmap2DROS *costmap_ros)
	{
		this->controller_name_ = controller_name;
		this->costmap_ros_ = costmap_ros;
		this->costmap_ = this->costmap_ros_->getCostmap();
		this->tf_buffer_ = tf;

		this->global_frame_ = this->costmap_ros_->getGlobalFrameID();

		this->tf_prefix_ = tf::getPrefixParam(this->nh_);
		this->robot_ns_ = this->nh_.getNamespace();

		// Initialize last published cmd vel object
		this->last_published_cmd_vel_.linear.x = 0.0;
		this->last_published_cmd_vel_.linear.y = 0.0;
		this->last_published_cmd_vel_.linear.z = 0.0;
		this->last_published_cmd_vel_.angular.x = 0.0;
		this->last_published_cmd_vel_.angular.y = 0.0;
		this->last_published_cmd_vel_.angular.z = 0.0;

		this->initTopics();
		this->initServices();
		this->initTimers();
	}

	bool FPCControllerBase::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
	{
		this->global_plan_ = plan;
		this->last_target_pose_ = this->global_plan_[0];

		this->controller_timer_.start();

		return true;
	}

	bool FPCControllerBase::isGoalReached(double xy_tolerance, double yaw_tolerance)
	{
		if(this->controller_finished_)
		{
			this->controller_timer_.stop();
			return true;
		}

		geometry_msgs::Pose goal_pose = this->getGoalPose();

		// Check x for tolerance
		if(goal_pose.position.x - this->current_robot_amcl_pose_.position.x > xy_tolerance)
		{
			return false;
		}

		// Check y for tolerance
		if(goal_pose.position.y - this->current_robot_amcl_pose_.position.y > xy_tolerance)
		{
			return false;
		}

		// Check orientation for tolerance
		tf::Quaternion inv_goal_orientation;
		tf::quaternionMsgToTF(goal_pose.orientation, inv_goal_orientation);
		inv_goal_orientation.setW(-inv_goal_orientation.getW()); // Negate w to get inverse quaternion

		tf::Quaternion current_orientation;
		tf::quaternionMsgToTF(this->current_robot_amcl_pose_.orientation, current_orientation);

		tf::Quaternion diff_orientation = current_orientation * inv_goal_orientation;
		float yaw_diff = tf::getYaw(diff_orientation);
		if(yaw_diff > yaw_tolerance)
		{
			return false;
		}

		this->controller_timer_.stop();
		return true;
	}

	bool FPCControllerBase::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
	{
		geometry_msgs::PoseStamped current_robot_pose_stamped;
		current_robot_pose_stamped.header.frame_id = this->global_frame_;
		current_robot_pose_stamped.header.stamp = ros::Time::now();
		current_robot_pose_stamped.pose = this->current_robot_amcl_pose_;

		geometry_msgs::TwistStamped current_velocity;
		current_velocity.header.frame_id = this->current_robot_odom_->header.frame_id;
		current_velocity.header.stamp = ros::Time::now();
		current_velocity.twist = this->current_robot_odom_->twist.twist;

		geometry_msgs::TwistStamped cmd_vel_stamped;
		std::string message;
		bool result;

		result = this->computeVelocityCommands(current_robot_pose_stamped, current_velocity, cmd_vel_stamped, message);

		cmd_vel = cmd_vel_stamped.twist;
	}

	uint32_t FPCControllerBase::computeVelocityCommands(const geometry_msgs::PoseStamped &pose,
														const geometry_msgs::TwistStamped &velocity,
														geometry_msgs::TwistStamped &cmd_vel,
														std::string &message)
	{
		// // ROS_ERROR_STREAM(this->robot_info_->robot_name);
		// // ROS_INFO_STREAM("pose     : " << pose.pose.position.x << " | " << pose.pose.position.y);
		// // ROS_INFO_STREAM("amcl_pose: " << this->current_robot_pose_.position.x << " | " << this->current_robot_pose_.position.y);
		// // ROS_INFO_STREAM("ground_truth: " << this->current_robot_ground_truth_->pose.pose.position.x << " | " << this->current_robot_ground_truth_->pose.pose.position.y);

		// double controller_freq = 2.0; // This must be a parameter later as the move base settings define this
		// double period_time_span = 1.0 / controller_freq;

		// // geometry_msgs::PoseStamped target_pose = this->global_plan_[this->pose_index_];

		// tf::Pose current_pose;
		// tf::poseMsgToTF(pose.pose, current_pose);


		// if(this->pose_index_ >= this->global_plan_.size())
		// {
		// 	ROS_ERROR_STREAM("FPCControllerBase: last pose_index exceeded without reaching goal.");
		// 	cmd_vel.header.frame_id = this->global_frame_;
		// 	cmd_vel.header.stamp = ros::Time::now();
		// 	cmd_vel.twist.linear.x = 0.0;
		// 	cmd_vel.twist.linear.y = 0.0;
		// 	cmd_vel.twist.linear.z = 0.0;
		// 	cmd_vel.twist.angular.x = 0.0;
		// 	cmd_vel.twist.angular.y = 0.0;
		// 	cmd_vel.twist.angular.z = 0.0;

		// 	this->controller_finished_ = true;

		// 	return 107;
		// }

		// tf::Pose target_pose;
		// tf::poseMsgToTF(this->global_plan_[this->pose_index_].pose, target_pose);
		

		// tf::Transform control_diff=current_pose.inverseTimes(target_pose);
		// // tf::Transform control_dif = target_pose.inverseTimes(current_pose);

		// // ROS_INFO_STREAM("current_dir: " << tf::getYaw(pose.pose.orientation) << " target_dir: " << tf::getYaw(this->global_plan_[this->pose_index_].pose.orientation) << " diff: " << tf::getYaw(this->global_plan_[this->pose_index_].pose.orientation) - tf::getYaw(pose.pose.orientation));


		// double x=control_diff.getOrigin().getX();
		// double y=control_diff.getOrigin().getY();   

		// double phi=tf::getYaw(control_diff.getRotation());

		// double v = std::sqrt(std::pow(x, 2) + std::pow(y, 2)) / period_time_span;
		// double omega = phi / period_time_span;
		
		
		// // if(this->robot_info_->robot_name == "robot0")
		// // {
		// // 	ROS_INFO_STREAM("Control: x: " << x << " y: " << y << " phi: " << phi);
		// // 	ROS_INFO_STREAM("v: " << v << " omega: " << omega << " time: " << period_time_span);
		// // }

		// // In theory a phi that is greater or smaller than pi/2 should not occure!
		// // if(phi>=M_PI_2)
		// // {
		// // 	phi-=M_PI;
		// // 	v=-v;    
		// // }
		// // else if( phi<=-M_PI_2)
		// // {
		// // 	phi=phi+M_PI;
		// // 	v=-v;
		// // }

		// double output_v;
		// double output_omega;

		// output_v = this->fpc_param_info_->getLyapunovParams().kx * x + v * cos(phi);
		// // ROS_INFO_STREAM("kx: " << this->robot_info_->lyapunov_params.kx << " x: " << x << " v: " << v << " phi: " << phi << " cos(phi): " << cos(phi));

		// output_omega = omega +
		// 			   this->fpc_param_info_->getLyapunovParams().ky * v * y +
		// 			   this->fpc_param_info_->getLyapunovParams().kphi * sin(phi);
		// // ROS_INFO_STREAM("omega: " << omega << " ky: " << this->robot_info_->lyapunov_params.ky << " v: " << v << " y: " << y << " kphi: " << this->robot_info_->lyapunov_params.kphi << " phi: " << phi << " sin(phi): " << sin(phi));
		// // COPY PASTED FROM MALTE END

		// if(this->fpc_param_info_->getCurrentRobotName() == "robot0")
		// 	ROS_INFO_STREAM("output_v: " << output_v << " output_omega: " << output_omega);

		// cmd_vel.twist.linear.x = output_v;
		// cmd_vel.twist.linear.y = 0.0;
		// cmd_vel.twist.linear.z = 0.0;

		// cmd_vel.twist.angular.x = 0.0;
		// cmd_vel.twist.angular.y = 0.0;
		// cmd_vel.twist.angular.z = output_omega;  // rad/s

		// // Set meta data info
		// this->meta_data_msg_.target_vel = cmd_vel.twist; 
		// this->meta_data_msg_.target_pose = this->convertPose(this->global_plan_[this->pose_index_].pose);
		// this->meta_data_msg_.current_pose = this->convertPose(pose.pose);
		// this->publishMetaData();

		// // if(this->robot_info_->robot_name == "robot0")
		// // 	ROS_INFO_STREAM("x_diff: " << this->global_plan_[this->pose_index_].pose.position.x - this->last_target_pose_.pose.position.x << " | y_diff: " << this->global_plan_[this->pose_index_].pose.position.y - this->last_target_pose_.pose.position.y << " | phi_diff: " << tf::getYaw(this->global_plan_[this->pose_index_].pose.orientation) - tf::getYaw(this->last_target_pose_.pose.orientation));
		// this->last_target_pose_ = this->global_plan_[this->pose_index_];

		// this->pose_index_++;

		cmd_vel.header.frame_id = this->global_frame_;
		cmd_vel.header.stamp = ros::Time::now();
		cmd_vel.twist = this->last_published_cmd_vel_;

		return 0;
	}

	void FPCControllerBase::publishMetaData(geometry_msgs::Pose2D target_pose)
	{
		this->meta_data_msg_.stamp = ros::Time::now();

		this->meta_data_msg_.current_pose = this->convertPose(this->current_robot_amcl_pose_);
		this->meta_data_msg_.current_vel = this->current_robot_odom_->twist.twist;

		this->meta_data_msg_.target_pose = target_pose;

		// Calculate diff current to target
		this->meta_data_msg_.current_to_target_pose_diff = this->calcDiff(
			this->meta_data_msg_.current_pose, this->meta_data_msg_.target_pose);
		this->meta_data_msg_.current_to_target_vel_diff = this->calcDiff(
			this->meta_data_msg_.target_vel, this->meta_data_msg_.current_vel);

		// Insert additional pose info
		this->meta_data_msg_.ground_truth_pose = this->convertPose(this->current_robot_ground_truth_->pose.pose);
		this->meta_data_msg_.odom_pose = this->convertPose(this->current_robot_odom_->pose.pose);

		// Insert additional interesting infos
		this->meta_data_msg_.current_to_ground_truth_pose_diff = this->calcDiff(
			this->meta_data_msg_.current_pose, this->meta_data_msg_.ground_truth_pose);

		// Publish data so it can be visualized in plotjuggler from rosbag
		this->meta_data_publisher_.publish(this->meta_data_msg_);
	}
	#pragma endregion

	#pragma region CallbackMethods
	void FPCControllerBase::getRobotPoseCb(
		const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
	{
		this->current_robot_amcl_pose_ = msg->pose.pose;
	}

	void FPCControllerBase::getRobotOdomCb(const nav_msgs::OdometryConstPtr &msg)
	{
		this->current_robot_odom_ = msg;
	}

	void FPCControllerBase::getRobotGroundTruthCb(const nav_msgs::OdometryConstPtr &msg)
	{
		this->current_robot_ground_truth_ = msg;
	}

	void FPCControllerBase::onControllerTimerCB(const ros::TimerEvent& timer_event_info)
	{
		this->runController();
	}
	#pragma endregion

	#pragma region Controller Methods
	void FPCControllerBase::runController() { }

	float FPCControllerBase::calcLinVelocity(geometry_msgs::Pose2D diff_vector, float scale_factor)
	{
		float output_v = this->fpc_param_info_->getLyapunovParams().kx * diff_vector.x +
						 this->fpc_param_info_->controller_params.max_vel_x * scale_factor * cos(diff_vector.theta);
		ROS_INFO_STREAM("kx: " << this->fpc_param_info_->current_robot_info->lyapunov_params.kx << " x: " << diff_vector.x << " v: " << this->fpc_param_info_->controller_params.max_vel_x << " scale_factor: " << scale_factor << " phi: " << diff_vector.theta << " cos(phi): " << cos(diff_vector.theta) << " v res: " << output_v);
		return output_v;
	}

	float FPCControllerBase::calcRotVelocity(geometry_msgs::Pose2D diff_vector)
	{
		ROS_ERROR_STREAM("diff_vector.y: " << diff_vector.y);
		float target_omega;
		if(diff_vector.theta < 0)
		{
			target_omega = -this->fpc_param_info_->controller_params.max_vel_theta;
		}
		else
		{
			target_omega = this->fpc_param_info_->controller_params.max_vel_theta;
		}

		float output_omega = target_omega +
							 this->fpc_param_info_->getLyapunovParams().ky * this->fpc_param_info_->controller_params.max_vel_x * diff_vector.y +
							 this->fpc_param_info_->getLyapunovParams().kphi * sin(diff_vector.theta);
		ROS_INFO_STREAM("omega: " << this->fpc_param_info_->controller_params.max_vel_theta << " ky: " << this->fpc_param_info_->current_robot_info->lyapunov_params.ky << " v: " << this->fpc_param_info_->controller_params.max_vel_x << " y: " << diff_vector.y << " kphi: " << this->fpc_param_info_->current_robot_info->lyapunov_params.kphi << " phi: " << diff_vector.theta << " sin(phi): " << sin(diff_vector.theta) << " omega res: " << output_omega); 
		return output_omega;
	}

	// int FPCControllerBase::locateRobotOnPath(geometry_msgs::Pose current_pose)
	// {
	// 	int closest_pose_index = 0;
	// 	float closest_distance = this->calcEuclideanDiff(current_pose, this->global_plan_[0].pose);
	// 	int second_closest_pose_index = 0;
	// 	float second_closest_distance = closest_distance; // Will be used later, but need to redo in/out of method					 
	// 	int pose_index = 0;
	// 	for(geometry_msgs::PoseStamped &global_plan_pose: this->global_plan_)
	// 	{
	// 		float distance_to_pose = this->calcEuclideanDiff(current_pose, global_plan_pose.pose);
	// 		if(distance_to_pose < closest_distance)
	// 		{
	// 			second_closest_pose_index = closest_pose_index;
	// 			second_closest_distance = closest_distance;
	// 			closest_pose_index = pose_index;
	// 			closest_distance = distance_to_pose;
	// 		}

	// 		pose_index++;
	// 	}

	// 	return closest_pose_index;
	// }
	#pragma endregion

	#pragma region ProtectedHelperMethods
	void FPCControllerBase::initServices() { }

	void FPCControllerBase::initTopics()
	{
		this->robot_amcl_pose_subscriber_ = this->nh_.subscribe(
			this->fpc_param_info_->getCurrentRobotNamespace() + "/" + this->fpc_param_info_->getCurrentRobotPoseTopic(),
			10,
			&FPCControllerBase::getRobotPoseCb,
			this);
		this->robot_odom_subscriber_ = this->nh_.subscribe(
			this->fpc_param_info_->getCurrentRobotNamespace() + "/" + this->fpc_param_info_->getCurrentRobotOdomTopic(),
			10,
			&FPCControllerBase::getRobotOdomCb,
			this);
		
		this->robot_ground_truth_subscriber_ = this->nh_.subscribe(
			this->fpc_param_info_->getCurrentRobotNamespace() + "/base_pose_ground_truth",
			10,
			&FPCControllerBase::getRobotGroundTruthCb,
			this);

		this->cmd_vel_publisher_ = this->nh_.advertise<geometry_msgs::Twist>(
			this->fpc_param_info_->getCurrentRobotNamespace() + "/" + this->fpc_param_info_->getCurrentRobotCmdVelTopic(),
			1000);

		this->meta_data_publisher_ = this->controller_nh_.advertise<fpp_msgs::LocalPlannerMetaData>(
			"fpc_meta_data", 1000);
	}

	void FPCControllerBase::initTimers() 
	{ 
		this->controller_timer_ = this->controller_nh_.createTimer(
			ros::Duration(1.0/this->fpc_param_info_->controller_params.controller_frequency), 
			&FPCControllerBase::onControllerTimerCB, this, false, false);
	}

	geometry_msgs::Pose2D FPCControllerBase::convertPose(geometry_msgs::Pose pose_to_convert)
	{
		geometry_msgs::Pose2D converted_pose;

		converted_pose.x = pose_to_convert.position.x;
		converted_pose.y = pose_to_convert.position.y;
		converted_pose.theta = tf::getYaw(pose_to_convert.orientation);

		return converted_pose;
	}

	geometry_msgs::Pose2D FPCControllerBase::calcDiff(geometry_msgs::Pose start_pose, geometry_msgs::Pose end_pose)
	{
		geometry_msgs::Pose2D diff_between_poses;
		diff_between_poses.x = end_pose.position.x - start_pose.position.x;
		diff_between_poses.y = end_pose.position.y - start_pose.position.y;
		tf::Quaternion quat_0;
		tf::quaternionMsgToTF(start_pose.orientation, quat_0);
		quat_0.normalize();
		tf::Quaternion quat_1;
		quat_1.normalize();
		tf::quaternionMsgToTF(end_pose.orientation, quat_1);
		tf::Quaternion quat_diff = quat_1 * quat_0.inverse();
		quat_diff.normalize();
		diff_between_poses.theta = tf::getYaw(quat_diff);

		return diff_between_poses;
	}

	geometry_msgs::Pose2D FPCControllerBase::calcDiff(geometry_msgs::Pose2D start_pose, geometry_msgs::Pose2D end_pose)
	{
		geometry_msgs::Pose2D pose_2D_diff;

		pose_2D_diff.x = end_pose.x - start_pose.x;
		pose_2D_diff.y = end_pose.y - start_pose.y;
		pose_2D_diff.theta = end_pose.theta - start_pose.theta;

		return pose_2D_diff;
	}

	geometry_msgs::Twist FPCControllerBase::calcDiff(geometry_msgs::Twist start_vel, geometry_msgs::Twist end_vel)
	{
		geometry_msgs::Twist vel_diff;
	
		vel_diff.linear.x = end_vel.linear.x - start_vel.linear.x;
		vel_diff.linear.y = 0.0;
		vel_diff.linear.z = 0.0;
		vel_diff.angular.x = 0.0;
		vel_diff.angular.y = 0.0;
		vel_diff.angular.z = end_vel.angular.z - start_vel.angular.z;

		return vel_diff;
	}

	float FPCControllerBase::calcEuclideanDiff(geometry_msgs::Pose point1, geometry_msgs::Pose point2)
	{
		return std::sqrt(std::pow(point1.position.x - point2.position.x, 2) +
						 std::pow(point1.position.y - point2.position.y, 2));
	}
	#pragma endregion
}