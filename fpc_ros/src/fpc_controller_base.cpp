#include <fpc_ros/fpc_controller_base.h>

namespace fpc
{
	#pragma region Constructors
	FPCControllerBase::FPCControllerBase(
		std::vector<std::shared_ptr<fpc_data_classes::LocalPlannerRobotInfo>> &robot_info_list,
		std::shared_ptr<fpc_data_classes::LocalPlannerRobotInfo> &robot_info,
		ros::NodeHandle &nh,
		ros::NodeHandle &controller_nh)
		: robot_info_list_(robot_info_list),
		  robot_info_(robot_info),
		  nh_(nh),
		  controller_nh_(controller_nh),
		  master_robot_info_(getMasterRobotInfo(robot_info_list)),
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

		this->initTopics();
		this->initServices();
		this->initTimers();
	}

	bool FPCControllerBase::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
	{
		this->global_plan_ = plan;
		return true;
	}

	bool FPCControllerBase::isGoalReached(double xy_tolerance, double yaw_tolerance)
	{
		if(this->controller_finished_)
		{
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
		// ROS_ERROR_STREAM(this->robot_info_->robot_name);
		// ROS_INFO_STREAM("pose     : " << pose.pose.position.x << " | " << pose.pose.position.y);
		// ROS_INFO_STREAM("amcl_pose: " << this->current_robot_pose_.position.x << " | " << this->current_robot_pose_.position.y);
		// ROS_INFO_STREAM("ground_truth: " << this->current_robot_ground_truth_->pose.pose.position.x << " | " << this->current_robot_ground_truth_->pose.pose.position.y);

		double controller_freq = 2; // This must be a parameter later as the move base settings define this
		double period_time_span = 1.0 / controller_freq;

		// geometry_msgs::PoseStamped target_pose = this->global_plan_[this->pose_index_];

		tf::Pose current_pose;
		tf::poseMsgToTF(pose.pose, current_pose);


		if(this->pose_index_ >= this->global_plan_.size())
		{
			ROS_ERROR_STREAM("FPCControllerBase: last pose_index exceeded without reaching goal.");
			cmd_vel.header.frame_id = this->global_frame_;
			cmd_vel.header.stamp = ros::Time::now();
			cmd_vel.twist.linear.x = 0.0;
			cmd_vel.twist.linear.y = 0.0;
			cmd_vel.twist.linear.z = 0.0;
			cmd_vel.twist.angular.x = 0.0;
			cmd_vel.twist.angular.y = 0.0;
			cmd_vel.twist.angular.z = 0.0;

			this->controller_finished_ = true;

			return 107;
		}

		tf::Pose target_pose;
		tf::poseMsgToTF(this->global_plan_[this->pose_index_].pose, target_pose);
		

		tf::Transform control_diff=current_pose.inverseTimes(target_pose);
		// tf::Transform control_dif = target_pose.inverseTimes(current_pose);

		// ROS_INFO_STREAM("current_dir: " << tf::getYaw(pose.pose.orientation) << " target_dir: " << tf::getYaw(this->global_plan_[this->pose_index_].pose.orientation) << " diff: " << tf::getYaw(this->global_plan_[this->pose_index_].pose.orientation) - tf::getYaw(pose.pose.orientation));


		double x=control_diff.getOrigin().getX();
		double y=control_diff.getOrigin().getY();   

		double phi=tf::getYaw(control_diff.getRotation());

		double v = std::sqrt(std::pow(x, 2) + std::pow(y, 2)) / period_time_span;
		double omega = phi / period_time_span;
		
		
		// if(this->robot_info_->robot_name == "robot0")
		// {
		// 	ROS_INFO_STREAM("Control: x: " << x << " y: " << y << " phi: " << phi);
		// 	ROS_INFO_STREAM("v: " << v << " omega: " << omega << " time: " << period_time_span);
		// }

		// In theory a phi that is greater or smaller than pi/2 should not occure!
		// if(phi>=M_PI_2)
		// {
		// 	phi-=M_PI;
		// 	v=-v;    
		// }
		// else if( phi<=-M_PI_2)
		// {
		// 	phi=phi+M_PI;
		// 	v=-v;
		// }

		double output_v;
		double output_omega;

		output_v = this->robot_info_->lyapunov_params.kx * x + v * cos(phi);
		// ROS_INFO_STREAM("kx: " << kx << " x: " << x << " v: " << v << " phi: " << phi << " cos(phi): " << cos(phi));

		output_omega = omega +
					   this->robot_info_->lyapunov_params.ky * v * y +
					   this->robot_info_->lyapunov_params.kphi * sin(phi);
		// ROS_INFO_STREAM("omega: " << omega << " ky: " << ky << " v: " << v << " y: " << y << " kphi: " << kphi << " phi: " << phi << " sin(phi): " << sin(phi));
		// COPY PASTED FROM MALTE END

		if(this->robot_info_->robot_name == "robot0")
			ROS_INFO_STREAM("output_v: " << output_v << " output_omega: " << output_omega);

		cmd_vel.twist.linear.x = output_v;
		cmd_vel.twist.linear.y = 0.0;
		cmd_vel.twist.linear.z = 0.0;

		cmd_vel.twist.angular.x = 0.0;
		cmd_vel.twist.angular.y = 0.0;
		cmd_vel.twist.angular.z = output_omega;  // rad/s

		// Set meta data info
		this->meta_data_msg_.target_vel = cmd_vel.twist; 
		this->meta_data_msg_.target_pose = this->convertPose(this->global_plan_[this->pose_index_].pose);
		this->meta_data_msg_.current_pose = this->convertPose(pose.pose);
		this->publishMetaData();

		this->pose_index_++;

		return 0;
	}

	void FPCControllerBase::publishMetaData()
	{
		this->meta_data_msg_.stamp = ros::Time::now();

		// Current pose should be set by calculateVelocityCommand method
		this->meta_data_msg_.current_vel = this->current_robot_odom_->twist.twist;

		// Calculate diff current to target
		this->meta_data_msg_.current_to_target_pose_diff = this->calcDiff(
			this->meta_data_msg_.current_pose, this->meta_data_msg_.target_pose);
		this->meta_data_msg_.current_to_target_vel_diff = this->calcDiff(
			this->meta_data_msg_.target_vel, this->meta_data_msg_.current_vel);

		// Insert additional pose info
		this->meta_data_msg_.ground_truth_pose = this->convertPose(this->current_robot_ground_truth_->pose.pose);
		this->meta_data_msg_.odom_pose = this->convertPose(this->current_robot_odom_->pose.pose);
		this->meta_data_msg_.amcl_pose = this->convertPose(this->current_robot_amcl_pose_);

		// Insert additional interesting infos
		this->meta_data_msg_.current_to_amcl_pose_diff = this->calcDiff(
			this->meta_data_msg_.current_pose, this->meta_data_msg_.amcl_pose);
		this->meta_data_msg_.current_to_ground_truth_pose_diff = this->calcDiff(
			this->meta_data_msg_.current_pose, this->meta_data_msg_.ground_truth_pose);
		this->meta_data_msg_.amcl_to_ground_truth_diff = this->calcDiff(
			this->meta_data_msg_.amcl_pose, this->meta_data_msg_.ground_truth_pose);

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
	#pragma endregion

	#pragma region ProtectedHelperMethods
	void FPCControllerBase::initServices() { }

	void FPCControllerBase::initTopics()
	{
		this->robot_amcl_pose_subscriber_ = this->nh_.subscribe(
			this->robot_info_->robot_namespace + "/" + this->robot_info_->robot_pose_topic,
			10,
			&FPCControllerBase::getRobotPoseCb,
			this);
		this->robot_odom_subscriber_ = this->nh_.subscribe(
			this->robot_info_->robot_namespace + "/" + this->robot_info_->robot_odom_topic,
			10,
			&FPCControllerBase::getRobotOdomCb,
			this);
		
		this->robot_ground_truth_subscriber_ = this->nh_.subscribe(
			this->robot_info_->robot_namespace + "/base_pose_ground_truth",
			10,
			&FPCControllerBase::getRobotGroundTruthCb,
			this);

		this->meta_data_publisher_ = this->controller_nh_.advertise<fpp_msgs::LocalPlannerMetaData>(
			"fpc_meta_data", 1000);
	}

	void FPCControllerBase::initTimers() { }

	std::shared_ptr<fpc_data_classes::LocalPlannerRobotInfo> FPCControllerBase::getMasterRobotInfo(
		const std::vector<std::shared_ptr<fpc_data_classes::LocalPlannerRobotInfo>> &robot_info_list)
	{
		for(int robot_info_counter = 0; robot_info_counter < robot_info_list.size(); robot_info_counter++)
		{
			if(robot_info_list[robot_info_counter]->fpc_master)
			{
				return robot_info_list[robot_info_counter];
			}
		}
		ROS_ERROR_STREAM("FPCControllerBase::getMasterRobotInfo: No master robot info found. Please check if flag is set in config.");
		return NULL;
	}

	geometry_msgs::Pose2D FPCControllerBase::convertPose(geometry_msgs::Pose pose_to_convert)
	{
		geometry_msgs::Pose2D converted_pose;

		converted_pose.x = pose_to_convert.position.x;
		converted_pose.y = pose_to_convert.position.y;
		converted_pose.theta = tf::getYaw(pose_to_convert.orientation);

		return converted_pose;
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
	#pragma endregion
}