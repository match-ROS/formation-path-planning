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
			this->getParams();

			// CREATE MASTER SLAVE OBJECTS HERE
			if(this->current_robot_info_->fpc_master)
			{
				this->fpc_controller_ = std::make_shared<fpc::FPCControllerMaster>(this->robot_info_list_,
																				   this->current_robot_info_,
																				   this->nh_,
																				   this->controller_nh_);
			}
			else
			{
				this->fpc_controller_ = std::make_shared<fpc::FPCControllerSlave>(this->robot_info_list_,
																				  this->current_robot_info_,
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
		return this->fpc_controller_->isGoalReached(this->xy_default_tolerance_,
												   this->yaw_default_tolerance_);
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

	////////////
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
		this->controller_nh_.param<double>("xy_default_tolerance", this->xy_default_tolerance_, 0.1);
		this->controller_nh_.param<double>("yaw_default_tolerance", this->yaw_default_tolerance_, ((2.0 * M_PI) / (360.0)) * 5.0);

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
        this->controller_nh_.getParam("formation_config", formation_config);
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

					// Set lyapunov params for each robot individually
					if(robot_info_xmlrpc.hasMember("lyapunov_params") && robot_info_xmlrpc["lyapunov_params"].getType() == XmlRpc::XmlRpcValue::TypeArray)
					{
						float kx = getNumberFromXMLRPC(robot_info_xmlrpc["lyapunov_params"][0],
													   "formation_config/" + robot_info->robot_name + "/lyapunov_params");
						float ky = getNumberFromXMLRPC(robot_info_xmlrpc["lyapunov_params"][1],
													   "formation_config/" + robot_info->robot_name + "/lyapunov_params");
						float kphi = getNumberFromXMLRPC(robot_info_xmlrpc["lyapunov_params"][2],
														 "formation_config/" + robot_info->robot_name + "/lyapunov_params");

						robot_info->lyapunov_params = fpc_data_classes::LyapunovParams(kx, ky, kphi);
					}
					else
					{
						ROS_ERROR_STREAM("lyapunov_params was not set for " << robot_info->robot_name);
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

	double FormationPathController::getNumberFromXMLRPC(XmlRpc::XmlRpcValue value, const std::string full_param_name)
    {
        if (value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
            value.getType() != XmlRpc::XmlRpcValue::TypeDouble)
        {
            std::string& value_string = value;
            ROS_FATAL("Values in the XmlRpcValue specification (param %s) must be numbers. Found value %s.",
                    full_param_name.c_str(), value_string.c_str());
            throw std::runtime_error("Values for the " + full_param_name + " specification must be numbers");
        }
        return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
    }

	// Eigen::Vector2f FormationPathController::getPosition(const geometry_msgs::Pose &pose)
	// {
	// 	Eigen::Vector2f position;
	// 	position << pose.position.x, pose.position.y;
	// 	return position;
	// }

	// Eigen::Vector3f FormationPathController::getPose(const geometry_msgs::Pose &pose)
	// {
	// 	Eigen::Vector3f eigen_pose;
	// 	float yaw = tf::getYaw(pose.orientation);
	// 	eigen_pose << this->getPosition(pose), yaw;
	// 	return eigen_pose;
	// }
}