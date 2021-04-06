#include <fpc_ros/data_classes/fpc_param_manager.h>

namespace fpc_data_classes
{
	FPCParamManager::FPCParamManager(ros::NodeHandle &nh, ros::NodeHandle &controller_nh)
		: nh_(nh), controller_nh_(controller_nh)
	{ 
		this->fpc_param_info_ = std::make_shared<FPCParamInfo>();
	}

	void FPCParamManager::getParams()
	{
		// Get robot name of the current robot
        std::string robot_name_key;
		std::string robot_name;
        if(this->nh_.searchParam("robot_name", robot_name_key))
        {
            this->nh_.getParam(robot_name_key, robot_name);
        }
        else
        {
            ROS_ERROR("No RobotName found in the robot namespace. This param \"robot_name\" has to be set.");
        }

		// Get default tolerances for the planner
		this->controller_nh_.param<double>("xy_default_tolerance", this->fpc_param_info_->xy_default_tolerance, 0.1);
		this->controller_nh_.param<double>("yaw_default_tolerance", this->fpc_param_info_->yaw_default_tolerance, ((2.0 * M_PI) / (360.0)) * 5.0);

		this->controller_nh_.param<bool>("prune_plane", this->fpc_param_info_->prune_plan, false);

		// Get controller params
		this->controller_nh_.param<float>("controller_config/controller_frequency",
										  this->fpc_param_info_->controller_params.controller_frequency, 100.0);
		this->controller_nh_.param<float>("controller_config/max_vel_x",
										  this->fpc_param_info_->controller_params.max_vel_x, 0.3);
		this->controller_nh_.param<float>("controller_config/min_vel_x",
										  this->fpc_param_info_->controller_params.min_vel_x, -0.2);
		this->controller_nh_.param<float>("controller_config/max_vel_theta",
										  this->fpc_param_info_->controller_params.max_vel_theta, 1.0);
		this->controller_nh_.param<float>("controller_config/min_vel_theta",
										  this->fpc_param_info_->controller_params.min_vel_theta, 0.0);

		// Get the topic name where the current pose of the robots will be published
		// This param is used to set the topic name of each robot to the same name. 
		// Special "robot_pose_topic" params in the robot params override this.
		std::string general_robot_pose_topic = "DefaultPoseTopicName";
		std::string general_robot_pose_topic_key;
		if(this->nh_.searchParam("robot_pose_topic", general_robot_pose_topic_key))
		{
			this->nh_.getParam(general_robot_pose_topic_key, general_robot_pose_topic);
		}

		// Get the topic name where the odometry of the robots will be published
		// This param is used to set the topic name of each robot to the same name. 
		// Special "robot_odom_topic" params in the robot params override this.
		std::string general_robot_odom_topic = "DefaultOdomTopicName";
		std::string general_robot_odom_topic_key;
		if(this->nh_.searchParam("robot_odom_topic", general_robot_odom_topic_key))
		{
			this->nh_.getParam(general_robot_odom_topic_key, general_robot_odom_topic);
		}

		// Get the topic name where the cmd_vel of the robots can be published
		// This param is used to set the topic name of each robot to the same name. 
		// Special "robot_cmd_vel_topic" params in the robot params override this.
		std::string general_robot_cmd_vel_topic = "DefaultCmdVelTopicName";
		std::string general_robot_cmd_vel_topic_key;
		if(this->nh_.searchParam("robot_cmd_vel_topic", general_robot_cmd_vel_topic_key))
		{
			this->nh_.getParam(general_robot_cmd_vel_topic_key, general_robot_cmd_vel_topic);
		}

		// Get information about the robots that are participating in the formation
		XmlRpc::XmlRpcValue formation_config;
        this->controller_nh_.getParam("formation_config", formation_config);
        if(formation_config.getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
            XmlRpc::XmlRpcValue::ValueStruct::const_iterator robot_iterator;
            for(robot_iterator = formation_config.begin(); robot_iterator != formation_config.end(); robot_iterator++)
            {
				std::shared_ptr<FPCRobotInfo> robot_info =
					std::make_shared<FPCRobotInfo>();

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

					// Set cmd_vel topic name for each robot
					if(robot_info_xmlrpc.hasMember("robot_cmd_vel_topic") && robot_info_xmlrpc["robot_cmd_vel_topic"].getType() == XmlRpc::XmlRpcValue::TypeString)
					{
						robot_info->robot_cmd_vel_topic = static_cast<std::string>(robot_info_xmlrpc["robot_cmd_vel_topic"]);
					}
					else if(general_robot_cmd_vel_topic != "DefaultCmdVelTopicName")
					{
						robot_info->robot_cmd_vel_topic = general_robot_cmd_vel_topic;
					}
					else
					{
						ROS_ERROR_STREAM("robot_cmd_vel_topic was not set for " << robot_info->robot_name);
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

                this->fpc_param_info_->robot_info_list.push_back(robot_info);
                if(robot_info->robot_name == robot_name)
				{
					this->fpc_param_info_->current_robot_info = robot_info;
				}
				if(robot_info->fpc_master)
				{
					this->fpc_param_info_->master_robot_info = robot_info;
				}
            }
        }		
	}

	std::shared_ptr<FPCParamInfo> FPCParamManager::getLocalPlannerInfo()
	{
		return this->fpc_param_info_;
	}

	double FPCParamManager::getNumberFromXMLRPC(XmlRpc::XmlRpcValue value, const std::string full_param_name)
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
}