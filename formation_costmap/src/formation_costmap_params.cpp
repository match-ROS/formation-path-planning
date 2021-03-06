#include <formation_costmap/formation_costmap_params.h>

namespace formation_costmap
{
	FormationCostmapParamManager::FormationCostmapParamManager(
		ros::NodeHandle &nh,
		ros::NodeHandle &costmap_nh)
				: nh_(nh), costmap_nh_(costmap_nh)
			{ }

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
			
		}
	}

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
}

