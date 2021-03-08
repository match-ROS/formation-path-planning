#include <formation_costmap/formation_costmap_params.h>

namespace formation_costmap
{
	FormationCostmapParamManager::FormationCostmapParamManager(
		ros::NodeHandle &nh,
		ros::NodeHandle &costmap_nh)
				: nh_(nh), costmap_nh_(costmap_nh)
	{ 
		this->formation_costmap_params_ = std::make_shared<FormationCostmapParams>();
	}

	void FormationCostmapParamManager::getParams(std::string formation_costmap_name)
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

		this->formation_costmap_name_ = formation_costmap_name;
		XmlRpc::XmlRpcValue formation_robots;
		this->costmap_nh_.getParam("formation_robots", formation_robots);
		if(formation_robots.getType() == XmlRpc::XmlRpcValue::TypeStruct)
		{
			XmlRpc::XmlRpcValue::ValueStruct::const_iterator robot_iterator;
			for(robot_iterator = formation_robots.begin(); robot_iterator != formation_robots.end(); robot_iterator++)
			{
				std::shared_ptr<FCRobotParams> robot_info = std::make_shared<FCRobotParams>();
				robot_info->robot_name = robot_iterator->first;

				XmlRpc::XmlRpcValue robot_info_xmlrpc = robot_iterator->second;
				if(robot_info_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeStruct)
				{
					if (robot_info_xmlrpc.hasMember("master") &&
						robot_info_xmlrpc["master"].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
					{
						robot_info->master = (bool)robot_info_xmlrpc["master"];
					}
					else
					{
						robot_info->master = false;
					}

					if (robot_info_xmlrpc.hasMember("namespace") &&
						robot_info_xmlrpc["namespace"].getType() == XmlRpc::XmlRpcValue::TypeString)
					{
						robot_info->robot_namespace = static_cast<std::string>(robot_info_xmlrpc["namespace"]);
					}
					
					if (robot_info_xmlrpc.hasMember("robot_outline"))
					{
						XmlRpc::XmlRpcValue robot_outline;
						robot_outline = robot_info_xmlrpc["robot_outline"];
						std::string robot_outline_key = "formation_config/" + robot_iterator->first + "/robot_outline";

						robot_info->robot_outline = this->createRobotOutlineFromXMLRPC(robot_outline, robot_outline_key);
					}

					if (robot_info_xmlrpc.hasMember("robot_pose_topic") &&
						robot_info_xmlrpc["robot_pose_topic"].getType() == XmlRpc::XmlRpcValue::TypeString)
					{
						robot_info->robot_pose_topic = static_cast<std::string>(robot_info_xmlrpc["robot_pose_topic"]);
					}
				}

				this->formation_costmap_params_->formation_robot_params.push_back(robot_info);
				if(robot_info->robot_name == robot_name)
				{
					this->formation_costmap_params_->current_robot_info = robot_info;
				}
			}

			// Set information about the master robot of the formation
			this->formation_costmap_params_->master_robot_info =
				this->getMasterRobotInfo(this->formation_costmap_params_->formation_robot_params);
		}
		else
		{
			ROS_ERROR_STREAM("error");
		}
	}

	void FormationCostmapParamManager::printInfo()
	{
		ROS_INFO_STREAM("Formation Costmap name: " << this->formation_costmap_name_);

		ROS_INFO_STREAM("Current Robot Name: " << this->formation_costmap_params_->current_robot_info->robot_name);
		ROS_INFO_STREAM("Master Robot Name: " << this->formation_costmap_params_->master_robot_info->robot_name);

		ROS_INFO_STREAM("Robot Params:");
		for(std::shared_ptr<FCRobotParams> robot_param : this->formation_costmap_params_->formation_robot_params)
		{
			ROS_INFO_STREAM("Robot Name: " << robot_param->robot_name);
			ROS_INFO_STREAM("Robot Namespace: " << robot_param->robot_namespace);
			ROS_INFO_STREAM("Robot Master: " << robot_param->master);
			ROS_INFO_STREAM("Robot Pose Topic: " << robot_param->robot_pose_topic);
		}
		ROS_INFO_STREAM("_______________________________________________");
	}

	#pragma region Getter/Setter
	std::shared_ptr<FormationCostmapParams> FormationCostmapParamManager::getFormationCostmapParams()
	{
		return this->formation_costmap_params_;
	}
	#pragma endregion

	#pragma ParamReadingHelper
	double FormationCostmapParamManager::getNumberFromXMLRPC(
		XmlRpc::XmlRpcValue value, const std::string full_param_name)
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

	std::vector<Eigen::Vector2f> FormationCostmapParamManager::createRobotOutlineFromXMLRPC(
		XmlRpc::XmlRpcValue footprint_xmlrpc,
		const std::string full_param_name)
	{
		// Make sure we have an array of at least 3 elements.
		if (footprint_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray ||
			footprint_xmlrpc.size() < 3)
		{
			ROS_FATAL("The footprint must be specified as list of lists on the parameter server, %s was specified as %s",
					full_param_name.c_str(), std::string(footprint_xmlrpc).c_str());
			throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least "
									"3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
		}

		std::vector<Eigen::Vector2f> footprint;

		for (int point_counter = 0; point_counter < footprint_xmlrpc.size(); point_counter++)
		{
			// Make sure each element of the list is an array of size 2. (x and y coordinates)
			XmlRpc::XmlRpcValue point_xmlrpc = footprint_xmlrpc[point_counter];
			if (point_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray ||
				point_xmlrpc.size() != 2)
			{
				ROS_FATAL("The footprint (parameter %s) must be specified as list of lists on the parameter server eg: "
							"[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
							full_param_name.c_str());
				throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: "
											"[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
			}
			Eigen::Vector2f point;
			point[0] = getNumberFromXMLRPC(point_xmlrpc[0], full_param_name);
			point[1] = getNumberFromXMLRPC(point_xmlrpc[1], full_param_name);

			footprint.push_back(point);
		}
		return footprint;
	}

	std::shared_ptr<FCRobotParams> FormationCostmapParamManager::getMasterRobotInfo(
		const std::vector<std::shared_ptr<FCRobotParams>> &robot_info_list)
	{
		for(int robot_info_counter = 0; robot_info_counter < robot_info_list.size(); robot_info_counter++)
		{
			if(robot_info_list[robot_info_counter]->master)
			{
				return robot_info_list[robot_info_counter];
			}
		}
		ROS_ERROR_STREAM("FPPParamManager::getMasterRobotInfo: No master robot info found. "
						 "Please check if flag is set in config.");
		return nullptr;
	}
	#pragma endregion
}

