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
            ROS_INFO("Initializing Formation Path Planner.");
            // Safe parameter for planning
            this->path_planner_name_ = name;
            this->costmap_ = costmap;
            this->global_frame_ = global_frame;
            this->array_size_ = this->costmap_->getSizeInCellsX() * this->costmap_->getSizeInCellsY();
            
            // Get the tf prefix
            ros::NodeHandle nh;
            this->tf_prefix_ = tf::getPrefixParam(nh);
            this->robot_ns_ = nh.getNamespace();

            Eigen::Vector2f lead_vector_world_cs;
            lead_vector_world_cs << 0, 0;
            this->formation_contour_ = geometry_info::FormationContour(lead_vector_world_cs, 0.0);
            
            this->getParams();

            
            // Init services and topics
            this->dyn_rec_inflation_srv_client_ = nh.serviceClient<fpp_msgs::DynReconfigure>("/dyn_reconfig_inflation");
            this->dyn_rec_inflation_srv_client_.waitForExistence();
            fpp_msgs::DynReconfigure dyn_reconfig_msg;
            dyn_reconfig_msg.request.new_inflation_radius = this->formation_outline_circle_.getCircleRadius();
            dyn_reconfig_msg.request.robot_namespace = this->robot_ns_ ;
            ROS_INFO("1");
            ros::Duration(0.1).sleep();
            ROS_INFO("2");
            if(this->robot_name_ == "robot0")
            {
                this->dyn_rec_inflation_srv_client_.call(dyn_reconfig_msg);
            }
            
            ROS_INFO("3");

            this->formation_footprint_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("formation_footprint", 10);
            while(this->formation_footprint_pub_.getNumSubscribers() < 1) 
            {
                ros::Duration(0.01).sleep();
            }
            
            geometry_msgs::PolygonStamped formation_footprint_msg;
            formation_footprint_msg.header.frame_id = "map";
            formation_footprint_msg.header.stamp = ros::Time::now();
            for(Eigen::Vector2f corner: this->formation_contour_.getCornerPointsWorldCS())
            {
                geometry_msgs::Point32 corner_point;
                corner_point.x = corner[0];
                corner_point.y = corner[1];
                corner_point.z = 0.0;
                formation_footprint_msg.polygon.points.push_back(corner_point);
            }
            this->formation_footprint_pub_.publish(formation_footprint_msg);

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

        // Get parameter of planner
        ros::NodeHandle private_nh("~/" + this->path_planner_name_);
        private_nh.param<float>("default_tolerance", this->default_tolerance_, 1333.0);

        
        


        this->start_ = start;
        this->goal_ = goal;

        
        return 0;
    }

    bool FormationPathPlanner::cancel()
    {
        ROS_ERROR("Formation Path Planner CANCEL");
        return false; // Returns false if not implemented. Returns true if implemented and cancel has been successful.
    }

    void FormationPathPlanner::getParams()
    {
        ros::NodeHandle planner_nh("~/" + this->path_planner_name_);
        ros::NodeHandle nh;

        // Get robot name of the current robot
        std::string robot_name_key;
        if(nh.searchParam("robot_name", robot_name_key))
        {
            nh.getParam(robot_name_key, this->robot_name_);
        }
        else
        {
            ROS_ERROR("No RobotName found in the robot namespace. This param \"robot_name\" has to be set.");
        }

        // Get parameter of planner
        planner_nh.param<float>("default_tolerance", this->default_tolerance_, 0.0);

        XmlRpc::XmlRpcValue formation_config;
        planner_nh.getParam("formation_config", formation_config);
        if(formation_config.getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
            XmlRpc::XmlRpcValue::ValueStruct::const_iterator robot_iterator;
            for(robot_iterator = formation_config.begin(); robot_iterator != formation_config.end(); robot_iterator++)
            {
                RobotInfo robot_info;
                robot_info.robot_name = robot_iterator->first;

                XmlRpc::XmlRpcValue robot_info_xmlrpc = robot_iterator->second;

                if(robot_info_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeStruct)
                {
                    if(robot_info_xmlrpc.hasMember("master"))
                    {
                        robot_info.fpp_master = robot_info_xmlrpc["master"];
                    }
                    else
                    {
                        robot_info.fpp_master = false;
                    }
                    
                    if(robot_info_xmlrpc.hasMember("robot_outline"))
                    {
                        XmlRpc::XmlRpcValue robot_outline;
                        robot_outline = robot_info_xmlrpc["robot_outline"];
                        std::string robot_outline_key = "formation_config/" + robot_iterator->first + "/robot_outline";

                        ROS_INFO_STREAM("outline full: " << robot_outline_key);

                        robot_info.robot_outline = this->createRobotOutlineFromXMLRPC(robot_outline, robot_outline_key);
                    }
                }

                this->robot_info_list_.insert(std::pair<std::string, RobotInfo>(robot_info.robot_name, robot_info));
            }
        }


        // // TEST
        // XmlRpc::XmlRpcValue robot_param_list;
        // planner_nh.getParam("formation_config", robot_param_list);

        // if(robot_param_list.getType() == XmlRpc::XmlRpcValue::TypeStruct)
        // {
        //     for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = robot_param_list.begin(); it != robot_param_list.end(); ++it)
        //     {
        //         ROS_INFO_STREAM("Found filter: " << (std::string)(it->first) << " ==> " << robot_param_list[it->first]);
        //         XmlRpc::XmlRpcValue test = robot_param_list[it->first];
        //         for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it2 = test.begin(); it2 != test.end(); ++it2)
        //         {
        //             ROS_INFO_STREAM("Found filter: " << (std::string)(it2->first) << " ==> " << test[it2->first]);
        //             XmlRpc::XmlRpcValue test2 = test[it2->first];
        //             ROS_INFO_STREAM("Type: " << test2.getType());
        //         }
        //     }
        // }
        // // TEST


        // First check for formation_config and robot0 config
        // if(!planner_nh.hasParam("formation_config") || !planner_nh.hasParam("formation_config/robot0"))
        // {
        //     ROS_ERROR("FormationPathPlanner: Error during initialization. formation_config or robot0 config is missing.");
        //     return;
        // }
        // std::string default_robot_name = "robot";
        // int default_last_robot = 100;
        // for(int robot_counter = 0; robot_counter <= default_last_robot; robot_counter++)
        // {
        //     std::string robot_position_namespace = "formation_config/" + default_robot_name + std::to_string(robot_counter) + "/";
        //     if(planner_nh.hasParam(robot_position_namespace))
        //     {
        //         std::string robot_name = default_robot_name + std::to_string(robot_counter);
        //         Eigen::Vector2f robot_position_world_cs;
        //         planner_nh.param<float>(robot_position_namespace + "x", robot_position_world_cs[0], 0.0);
        //         planner_nh.param<float>(robot_position_namespace + "y", robot_position_world_cs[1], 0.0);
        //         float yaw;
        //         planner_nh.param<float>(robot_position_namespace + "yaw", yaw, 0.0);
                
        //         geometry_info::GeometryContour mur205 = geometry_info::GeometryContour(robot_position_world_cs, yaw);
        //         for(Eigen::Vector2f corner_geometry_cs : robot_outline)
        //         {
        //             mur205.addContourCornerGeometryCS(corner_geometry_cs);
        //         }
        //         this->formation_contour_.addRobotToFormation(mur205);
        //         // this->formation_contour_.addContourCornersWorldCS(mur205.getCornerPointsWorldCS());

        //         this->robot_position_list_.insert(std::pair<std::string, geometry_info::GeometryContour>(robot_name, mur205));
        //     }
        // }
    }

    void FormationPathPlanner::calcFormationEnclosingCircle()
    {
        this->formation_outline_circle_ = fpp_helper::MinimalEnclosingCircle();
        std::vector<Eigen::Vector2f> robot_positions;
        this->formation_outline_circle_.calcMinimalEnclosingCircle(this->formation_contour_.getCornerPointsWorldCS());
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