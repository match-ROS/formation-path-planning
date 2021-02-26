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
			this->tf_buffer_ = tf;
			this->costmap_ros_ = costmap_ros;
			this->costmap_ = this->costmap_ros_->getCostmap();
			this->global_frame_ = this->costmap_ros_->getGlobalFrameID();

			this->nh_ = ros::NodeHandle();
			this->planner_nh_ = ros::NodeHandle("~/" + this->controller_name_);
		
			this->tf_prefix_ = tf::getPrefixParam(this->nh_);
			this->robot_ns_ = this->nh_.getNamespace();

			// Read and store all params that are set by the yaml config
			this->getParams();

			// Initialize the topic subscriber/publisher, services and actions
			this->robot_pose_subscriber_ = this->nh_.subscribe(
				this->current_robot_info_->robot_namespace + "/" + this->current_robot_info_->robot_pose_topic,
				10,
				&FormationPathController::getRobotPoseCb,
				this);
			this->robot_odom_subscriber_ = this->nh_.subscribe(
				this->current_robot_info_->robot_namespace + "/" + this->current_robot_info_->robot_odom_topic,
				10,
				&FormationPathController::getRobotOdomCb,
				this);
			
			this->robot_ground_truth_subscriber_ = this->nh_.subscribe(
				this->current_robot_info_->robot_namespace + "/base_pose_ground_truth",
				10,
				&FormationPathController::getRobotGroundTruthCb,
				this);

			// CREATE MASTER SLAVE OBJECTS HERE

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
		ROS_ERROR_STREAM("setPlan");
		this->global_plan_ = plan; // Save the complete global plan
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

	bool FormationPathController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
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
		return result;
	}

	uint32_t FormationPathController::computeVelocityCommands(const geometry_msgs::PoseStamped &pose,
															  const geometry_msgs::TwistStamped &velocity,
															  geometry_msgs::TwistStamped &cmd_vel,
															  std::string &message)
	{
		ROS_ERROR_STREAM(this->current_robot_info_->robot_name);
		// ROS_INFO_STREAM("pose     : " << pose.pose.position.x << " | " << pose.pose.position.y);
		// ROS_INFO_STREAM("amcl_pose: " << this->current_robot_pose_.position.x << " | " << this->current_robot_pose_.position.y);
		// ROS_INFO_STREAM("ground_truth: " << this->current_robot_ground_truth_->pose.pose.position.x << " | " << this->current_robot_ground_truth_->pose.pose.position.y);

		double controller_freq = 0.2; // This must be a parameter later as the move base settings define this
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

	bool FormationPathController::cancel()
	{
		ROS_ERROR_STREAM("cancel");
	}

	//////////////////////////////////////////////////
	// Getter/Setter
	//////////////////////////////////////////////////
	geometry_msgs::Pose FormationPathController::getStartPose()
	{
		return this->global_plan_.front().pose;
	}

	geometry_msgs::Pose FormationPathController::getGoalPose()
	{
		return this->global_plan_.back().pose;
	}

	////////////////////////////////////////////////
	// Callback method
	////////////////////////////////////////////////
	void FormationPathController::getRobotPoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
	{
		this->current_robot_pose_ = msg->pose.pose;
	}

	void FormationPathController::getRobotOdomCb(const nav_msgs::OdometryConstPtr &msg)
	{
		this->current_robot_odom_ = msg;
	}

	void FormationPathController::getRobotGroundTruthCb(const nav_msgs::OdometryConstPtr &msg)
	{
		this->current_robot_ground_truth_ = msg;
	}

	////////////////////////////////////////////////
	// Private Helper Methods
	////////////////////////////////////////////////
	void FormationPathController::getParams()
	{
		// Get robot name of the current robot
        std::string robot_name_key;
        if(this->nh_.searchParam("robot_name", robot_name_key))
        {
            this->nh_.getParam(robot_name_key, this->robot_name_);
        }
        else
        {
            ROS_ERROR("No RobotName found in the robot namespace. This param \"robot_name\" has to be set.");
        }

		// Get default tolerances for the planner
		this->planner_nh_.param<double>("xy_default_tolerance", this->xy_default_tolerance_, 0.1);
		this->planner_nh_.param<double>("yaw_default_tolerance", this->yaw_default_tolerance_, ((2.0 * M_PI) / (360.0)) * 5.0);

		// Get the topic name where the current pose of the robots will be published
		// This param is used to set the topic name of each robot to the same name. Special "robot_pose_topic" params in the robot params override this.
		std::string general_robot_pose_topic = "DefaultPoseTopicName";
		std::string general_robot_pose_topic_key;
		if(this->nh_.searchParam("robot_pose_topic", general_robot_pose_topic_key))
		{
			this->nh_.getParam(general_robot_pose_topic_key, general_robot_pose_topic);
		}

		// Get the topic name where the odometry of the robots will be published
		// This param is used to set the topic name of each robot to the same name. Special "robot_pose_topic" params in the robot params override this.
		std::string general_robot_odom_topic = "DefaultOdomTopicName";
		std::string general_robot_odom_topic_key;
		if(this->nh_.searchParam("robot_odom_topic", general_robot_odom_topic_key))
		{
			this->nh_.getParam(general_robot_odom_topic_key, general_robot_odom_topic);
		}

		// Get information about the robots that are participating in the formation
		XmlRpc::XmlRpcValue formation_config;
        this->planner_nh_.getParam("formation_config", formation_config);
        if(formation_config.getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
            XmlRpc::XmlRpcValue::ValueStruct::const_iterator robot_iterator;
            for(robot_iterator = formation_config.begin(); robot_iterator != formation_config.end(); robot_iterator++)
            {
				std::shared_ptr<fpc_data_classes::LocalPlannerRobotInfo> robot_info =
					std::make_shared<fpc_data_classes::LocalPlannerRobotInfo>();

                robot_info->robot_name = robot_iterator->first;

                XmlRpc::XmlRpcValue robot_info_xmlrpc = robot_iterator->second;

                if(robot_info_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeStruct)
                {
					// Set master flag to identify which robot is the master
                    if(robot_info_xmlrpc.hasMember("master") && robot_info_xmlrpc["master"].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
                    {
                        robot_info->fpc_master = (bool)robot_info_xmlrpc["master"];
                    }
                    else
                    {
						// master parameter is not available for the robot so robot is not master 
                        robot_info->fpc_master = false;
                    }

					// Set namespace for each robot to be able to access the topics in their namespace
                    if(robot_info_xmlrpc.hasMember("namespace") && robot_info_xmlrpc["namespace"].getType() == XmlRpc::XmlRpcValue::TypeString)
                    {
                        robot_info->robot_namespace = static_cast<std::string>(robot_info_xmlrpc["namespace"]);
                    }

					// Set robot pose topic name for each robot
					if(robot_info_xmlrpc.hasMember("robot_pose_topic") && robot_info_xmlrpc["robot_pose_topic"].getType() == XmlRpc::XmlRpcValue::TypeString)
					{
						robot_info->robot_pose_topic = static_cast<std::string>(robot_info_xmlrpc["robot_pose_topic"]);
					}
					else if(general_robot_pose_topic != "DefaultPoseTopicName")
					{
						robot_info->robot_pose_topic = general_robot_pose_topic;
					}
					else
					{
						ROS_ERROR_STREAM("robot_pose_topic was not set for " << robot_info->robot_name);
					}

					// Set robot_odom topic name for each robot
					if(robot_info_xmlrpc.hasMember("robot_odom_topic") && robot_info_xmlrpc["robot_odom_topic"].getType() == XmlRpc::XmlRpcValue::TypeString)
					{
						robot_info->robot_odom_topic = static_cast<std::string>(robot_info_xmlrpc["robot_odom_topic"]);
					}
					else if(general_robot_odom_topic != "DefaultOdomTopicName")
					{
						robot_info->robot_odom_topic = general_robot_odom_topic;
					}
					else
					{
						ROS_ERROR_STREAM("robot_odom_topic was not set for " << robot_info->robot_name);
					}
                }

                this->robot_info_list_.push_back(robot_info);
                if(robot_info->robot_name == this->robot_name_)
				{
					this->current_robot_info_ = robot_info;
				}
            }
        }		
	}

	Eigen::Vector2f FormationPathController::getPosition(const geometry_msgs::Pose &pose)
	{
		Eigen::Vector2f position;
		position << pose.position.x, pose.position.y;
		return position;
	}

	Eigen::Vector3f FormationPathController::getPose(const geometry_msgs::Pose &pose)
	{
		Eigen::Vector3f eigen_pose;
		float yaw = tf::getYaw(pose.orientation);
		eigen_pose << this->getPosition(pose), yaw;
		return eigen_pose;
	}
}