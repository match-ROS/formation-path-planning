#include <fpp_ros/data_classes/fpp_param_manager.h>

namespace fpp_data_classes
{
	FPPParamManager::FPPParamManager(ros::NodeHandle &nh, ros::NodeHandle &planner_nh)
		: nh_(nh), planner_nh_(planner_nh)
	{ }

	void FPPParamManager::getParams()
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

        // Get parameter of planner
        this->planner_nh_.param<float>("default_tolerance", this->default_tolerance_, 0.0);
        
        XmlRpc::XmlRpcValue formation_config;
        this->planner_nh_.getParam("formation_config", formation_config);
        if(formation_config.getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
            XmlRpc::XmlRpcValue::ValueStruct::const_iterator robot_iterator;
            for(robot_iterator = formation_config.begin(); robot_iterator != formation_config.end(); robot_iterator++)
            {
				std::shared_ptr<fpp_data_classes::RobotInfo> robot_info = std::make_shared<fpp_data_classes::RobotInfo>();
                robot_info->robot_name = robot_iterator->first;

                XmlRpc::XmlRpcValue robot_info_xmlrpc = robot_iterator->second;

                if(robot_info_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeStruct)
                {
                    if(robot_info_xmlrpc.hasMember("master") && robot_info_xmlrpc["master"].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
                    {
                        robot_info->fpp_master = (bool)robot_info_xmlrpc["master"];
                    }
                    else
                    {
                        robot_info->fpp_master = false;

                        // Only if the robot is not the master the offset param is existing
                        if(robot_info_xmlrpc.hasMember("offset") && robot_info_xmlrpc["offset"].getType() == XmlRpc::XmlRpcValue::TypeArray)
                        {
							robot_info->offset[0] = getNumberFromXMLRPC(robot_info_xmlrpc["offset"][0],
																		"formation_config/" + robot_info->robot_name + "/offset");
							robot_info->offset[1] = getNumberFromXMLRPC(robot_info_xmlrpc["offset"][1],
																		"formation_config/" + robot_info->robot_name + "/offset");
						}
                    }

                    if(robot_info_xmlrpc.hasMember("namespace") && robot_info_xmlrpc["namespace"].getType() == XmlRpc::XmlRpcValue::TypeString)
                    {
                        robot_info->robot_namespace = static_cast<std::string>(robot_info_xmlrpc["namespace"]);
                    }
                    
                    if(robot_info_xmlrpc.hasMember("robot_outline"))
                    {
                        XmlRpc::XmlRpcValue robot_outline;
                        robot_outline = robot_info_xmlrpc["robot_outline"];
                        std::string robot_outline_key = "formation_config/" + robot_iterator->first + "/robot_outline";

                        robot_info->robot_outline = this->createRobotOutlineFromXMLRPC(robot_outline, robot_outline_key);
                    }

					if(robot_info_xmlrpc.hasMember("robot_pose_topic") && robot_info_xmlrpc["robot_pose_topic"].getType() == XmlRpc::XmlRpcValue::TypeString)
                    {
                        robot_info->robot_pose_topic_name = static_cast<std::string>(robot_info_xmlrpc["robot_pose_topic"]);
                    }
                }

                this->robot_info_list_.push_back(robot_info);
                if(robot_info->robot_name == robot_name)
				{
					this->current_robot_info_ = robot_info;
				}
            }
        }

		// Set information about the master robot of the formation
		this->master_robot_info_ = this->getMasterRobotInfo(this->robot_info_list_);
    }

	#pragma region Getter/Setter
	std::string FPPParamManager::getCurrentRobotName()
	{
		return this->current_robot_info_->robot_name;
	}

	std::string FPPParamManager::getCurrentRobotNamespace()
	{
		return this->current_robot_info_->robot_namespace;
	}

	float FPPParamManager::getDefaultTolerance() 
	{
		return this->default_tolerance_;
	}

	std::vector<std::shared_ptr<fpp_data_classes::RobotInfo>> FPPParamManager::getRobotInfoList()
	{
		return this->robot_info_list_;
	}

	std::shared_ptr<fpp_data_classes::RobotInfo> FPPParamManager::getCurrentRobotInfo()
	{
		return this->current_robot_info_;
	}

	std::shared_ptr<fpp_data_classes::RobotInfo> FPPParamManager::getMasterRobotInfo()
	{
		return this->master_robot_info_;
	}

	std::shared_ptr<fpp_data_classes::RobotInfo> FPPParamManager::getRobotInfoByRobotName(std::string robot_name)
	{
		for(std::shared_ptr<fpp_data_classes::RobotInfo> robot_info: this->robot_info_list_)
		{
			if(robot_info->robot_name == robot_name)
			{
				return robot_info;
			}
		}

		return nullptr; // No info with specified name found, return nullptr
	}
	#pragma endregion

	#pragma region HelperMethods
	bool FPPParamManager::isCurrentRobotMaster()
	{
		return this->current_robot_info_->fpp_master;
	}
	#pragma endregion

	#pragma region ParamReadingHelper
	std::shared_ptr<fpp_data_classes::RobotInfo> FPPParamManager::getMasterRobotInfo(
		const std::vector<std::shared_ptr<fpp_data_classes::RobotInfo>> &robot_info_list)
	{
		for(int robot_info_counter = 0; robot_info_counter < robot_info_list.size(); robot_info_counter++)
		{
			if(robot_info_list[robot_info_counter]->fpp_master)
			{
				return robot_info_list[robot_info_counter];
			}
		}
		ROS_ERROR_STREAM("FPPParamManager::getMasterRobotInfo: No master robot info found. Please check if flag is set in config.");
		return nullptr;
	}

	double FPPParamManager::getNumberFromXMLRPC(XmlRpc::XmlRpcValue value, const std::string full_param_name)
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

    std::vector<Eigen::Vector2f> FPPParamManager::createRobotOutlineFromXMLRPC(XmlRpc::XmlRpcValue footprint_xmlrpc,
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
	#pragma endregion
}