#include <fpp_ros/fpp_ros.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(fpp::FormationPathPlanner, mbf_costmap_core::CostmapPlanner)
PLUGINLIB_EXPORT_CLASS(fpp::FormationPathPlanner, nav_core::BaseGlobalPlanner)

namespace fpp
{
    FormationPathPlanner::FormationPathPlanner() : 
        initialized_(false)
    {
        ROS_ERROR("FORMATION PATH PLANNER DEFAULT CONSTRUCTOR");
    }

    FormationPathPlanner::FormationPathPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros) : 
        initialized_(false)
    {
        ROS_ERROR("FORMATION PATH PLANNER OVERLOADED CONSTRUCTOR");
        this->initialize(name, costmap_ros);
    }

    void FormationPathPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if(!this->initialized_)
        {
            this->costmap_ros_ = costmap_ros_;
            this->initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
        }
    }

    void FormationPathPlanner::initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string global_frame)
    {
        if(!this->initialized_)
        {
            // Safe parameter for planning
            this->path_planner_name_ = name;
            this->costmap_ = costmap;
            this->global_frame_ = global_frame;
            this->array_size_ = this->costmap_->getSizeInCellsX() * this->costmap_->getSizeInCellsY();

            // Initialize node handle that points to namespace of planner
            this->nh_ = ros::NodeHandle();
            this->planner_nh_ = ros::NodeHandle("~/" + this->path_planner_name_);

            // Get the tf prefix
            this->tf_prefix_ = tf::getPrefixParam(this->nh_);
            this->robot_ns_ = this->nh_.getNamespace();

            // Get all params from the config file for the global path planner            
            this->getParams();
            ROS_INFO("Initializing Formation Path Planner in namespace: %s", this->this_robots_robot_info_->robot_name.c_str());

            if(this->this_robots_robot_info_->fpp_master)
            {
                this->fpp_controller_ = std::make_shared<FPPControllerMaster>(this->robot_info_list_,
                                                                              this->this_robots_robot_info_,
                                                                              this->nh_,
                                                                              this->planner_nh_);
            }
            else
            {
                this->fpp_controller_ = std::make_shared<FPPControllerSlave>(this->robot_info_list_,
                                                                             this->this_robots_robot_info_,
                                                                             this->nh_,
                                                                             this->planner_nh_);
            }
            this->fpp_controller_->initialize(name, costmap, global_frame);

            initialized_ = true; // Initialized method was called so planner is now initialized

            ROS_INFO("Formation Path Planner finished intitialization.");
        }
        else
        {
            ROS_WARN("Formation Path Planner has already been initialized");
        }
    }    

    bool FormationPathPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
                const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {
        double cost;
        std::string message;
        return 10 > this->makePlan(start, goal, 0.1, plan, cost, message);
    }  

    uint32_t FormationPathPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                double tolerance, std::vector<geometry_msgs::PoseStamped> &plan, double &cost,
                                std::string &message)
    {
        if(!initialized_) //Planner was not initialized. Abort
        {
            ROS_ERROR("RelaxedAStar planner was not initialized yet. Please initialize the planner before usage.");
            return mbf_msgs::GetPathResult::NOT_INITIALIZED;
        }

        this->start_ = start;
        this->goal_ = goal;

        this->fpp_controller_->execute(start, goal, plan);
        
        return 0;
    }

    bool FormationPathPlanner::cancel()
    {
        ROS_ERROR("Formation Path Planner CANCEL");
        return false; // Returns false if not implemented. Returns true if implemented and cancel has been successful.
    }

    void FormationPathPlanner::getParams()
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

        // Get parameter of planner
        this->planner_nh_.param<float>("default_tolerance", this->default_tolerance_, 0.0);
        
        XmlRpc::XmlRpcValue formation_config;
        this->planner_nh_.getParam("formation_config", formation_config);
        if(formation_config.getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
            XmlRpc::XmlRpcValue::ValueStruct::const_iterator robot_iterator;
            for(robot_iterator = formation_config.begin(); robot_iterator != formation_config.end(); robot_iterator++)
            {
                fpp_data_classes::RobotInfo robot_info;
                robot_info.robot_name = robot_iterator->first;

                XmlRpc::XmlRpcValue robot_info_xmlrpc = robot_iterator->second;

                if(robot_info_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeStruct)
                {
                    if(robot_info_xmlrpc.hasMember("master") && robot_info_xmlrpc["master"].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
                    {
                        robot_info.fpp_master = (bool)robot_info_xmlrpc["master"];
                    }
                    else
                    {
                        robot_info.fpp_master = false;

                        // Only if the robot is not the master the offset param is existing
                        if(robot_info_xmlrpc.hasMember("offset") && robot_info_xmlrpc["offset"].getType() == XmlRpc::XmlRpcValue::TypeArray)
                        {
                            Eigen::Vector2f offset;
                            offset[0] = getNumberFromXMLRPC(robot_info_xmlrpc["offset"][0], "formation_config/" + robot_info.robot_name + "/offset");
                            offset[1] = getNumberFromXMLRPC(robot_info_xmlrpc["offset"][1], "formation_config/" + robot_info.robot_name + "/offset");
                        }
                    }

                    if(robot_info_xmlrpc.hasMember("namespace") && robot_info_xmlrpc["namespace"].getType() == XmlRpc::XmlRpcValue::TypeString)
                    {
                        robot_info.robot_namespace = static_cast<std::string>(robot_info_xmlrpc["namespace"]);
                    }
                    
                    if(robot_info_xmlrpc.hasMember("robot_outline"))
                    {
                        XmlRpc::XmlRpcValue robot_outline;
                        robot_outline = robot_info_xmlrpc["robot_outline"];
                        std::string robot_outline_key = "formation_config/" + robot_iterator->first + "/robot_outline";

                        robot_info.robot_outline = this->createRobotOutlineFromXMLRPC(robot_outline, robot_outline_key);
                    }
                }

                this->robot_info_list_.push_back(robot_info);
                if(this->robot_info_list_.back().robot_name == this->robot_name_)
                {
                    this->this_robots_robot_info_ = &(this->robot_info_list_.back());
                }                
            }
        }
    }

    Eigen::Vector2f FormationPathPlanner::MsgsPoseToEigenVector2f(geometry_msgs::Pose pose_to_convert)
    {
        Eigen::Vector2f converted_position;
        converted_position << pose_to_convert.position.x, pose_to_convert.position.y;
        return converted_position;
    }

    double FormationPathPlanner::getNumberFromXMLRPC(XmlRpc::XmlRpcValue value, const std::string full_param_name)
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

    std::vector<Eigen::Vector2f> FormationPathPlanner::createRobotOutlineFromXMLRPC(XmlRpc::XmlRpcValue footprint_xmlrpc,
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