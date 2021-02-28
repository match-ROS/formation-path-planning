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
		  getMasterRobotInfo(robot_info_list)
	{
	}
	#pragma endregion

	#pragma region ControllerInterface
	void FPCControllerBase::initialize(std::string controller_name,
									   costmap_2d::Costmap2D *costmap,
									   std::string global_frame)
	{
		this->controller_name_ = controller_name;
		this->costmap_ = costmap;
		this->global_frame_ = global_frame;
	}

	bool FPCControllerBase::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
	{
		this->global_plan_ = plan;
		return true;
	}

	bool FPCControllerBase::isGoalReached(double xy_tolerance, double yaw_tolerance)
	{
		// Glaube ich muss hier auch etwas übergreifendes implementieren, falls der Master meldet Ziel ist erreicht, dann müssen die anderen ROboter das auch melden
		// Außer ich will vllt noch versuchen den restlichen Regelfehler auszugleichen, aber da sollte ja keiner sein

		geometry_msgs::Pose goal_pose = this->getGoalPose();

		// Check x for tolerance
		if(goal_pose.position.x - this->current_robot_pose_.position.x > xy_tolerance)
		{
			return false;
		}

		// Check y for tolerance
		if(goal_pose.position.y - this->current_robot_pose_.position.y > xy_tolerance)
		{
			return false;
		}

		// Check orientation for tolerance
		tf::Quaternion inv_goal_orientation;
		tf::quaternionMsgToTF(goal_pose.orientation, inv_goal_orientation);
		inv_goal_orientation.setW(-inv_goal_orientation.getW()); // Negate w to get inverse quaternion

		tf::Quaternion current_orientation;
		tf::quaternionMsgToTF(this->current_robot_pose_.orientation, current_orientation);

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
		current_robot_pose_stamped.pose = this->current_robot_pose_;

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
		ROS_ERROR_STREAM(this->current_robot_info_->robot_name);
		// ROS_INFO_STREAM("pose     : " << pose.pose.position.x << " | " << pose.pose.position.y);
		// ROS_INFO_STREAM("amcl_pose: " << this->current_robot_pose_.position.x << " | " << this->current_robot_pose_.position.y);
		// ROS_INFO_STREAM("ground_truth: " << this->current_robot_ground_truth_->pose.pose.position.x << " | " << this->current_robot_ground_truth_->pose.pose.position.y);

		double controller_freq = 2; // This must be a parameter later as the move base settings define this
		double period_time_span = 1.0 / controller_freq;

		// geometry_msgs::PoseStamped target_pose = this->global_plan_[this->pose_index_];

		tf::Pose current_pose;
		tf::poseMsgToTF(pose.pose, current_pose);
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
		if(this->current_robot_info_->robot_name == "robot0")
		{
			ROS_INFO_STREAM("Control: x: " << x << " y: " << y << " phi: " << phi);
			ROS_INFO_STREAM("v: " << v << " omega: " << omega << " time: " << period_time_span);
		}

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

		double kx = 3.4;
		double ky = 10.0;
		double kphi = 2.5;

		// double kx = 0.0;
		// double ky = 0.0;
		// double kphi = 0.0;

		double output_v;
		double output_omega;

		output_v = kx * x + v * cos(phi);
		// ROS_INFO_STREAM("kx: " << kx << " x: " << x << " v: " << v << " phi: " << phi << " cos(phi): " << cos(phi));

		output_omega = omega + ky * v * y + kphi * sin(phi);
		// ROS_INFO_STREAM("omega: " << omega << " ky: " << ky << " v: " << v << " y: " << y << " kphi: " << kphi << " phi: " << phi << " sin(phi): " << sin(phi));
		// COPY PASTED FROM MALTE END

		if(this->current_robot_info_->robot_name == "robot0")
			ROS_INFO_STREAM("output_v: " << output_v << " output_omega: " << output_omega);

		cmd_vel.twist.linear.x = output_v;
		cmd_vel.twist.linear.y = 0.0;
		cmd_vel.twist.linear.z = 0.0;

		cmd_vel.twist.angular.x = 0.0;
		cmd_vel.twist.angular.y = 0.0;
		cmd_vel.twist.angular.z = output_omega;  // rad/s

		this->pose_index_++;

		return 0;
	}
	#pragma endregion

	#pragma region CallbackMethods
	void FPCControllerBase::getRobotPoseCb(
		const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
	{
		this->current_robot_pose_ = msg->pose.pose;
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
		this->robot_pose_subscriber_ = this->nh_.subscribe(
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
	}

	void FPCControllerBase::initTimers() { }

	std::shared_ptr<fpp_data_classes::RobotInfo> FPCControllerBase::getMasterRobotInfo(
		const std::vector<std::shared_ptr<fpp_data_classes::RobotInfo>> &robot_info_list)
	{
		for(int robot_info_counter = 0; robot_info_counter < robot_info_list.size(); robot_info_counter++)
		{
			if(robot_info_list[robot_info_counter]->fpp_master)
			{
				return robot_info_list[robot_info_counter];
			}
		}
		ROS_ERROR_STREAM("FPPControllerBase::getMasterRobotInfo: No master robot info found. Please check if flag is set in config.");
		return NULL;
	}
	#pragma endregion
}