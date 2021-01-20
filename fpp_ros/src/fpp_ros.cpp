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

            // Get all params from the config file for the global path planner            
            this->getParams();
            
            // Das hier in den fpp_master auslagern? Weil das muss nur dort gecalled werden und dann brauche ich hier keine unnötige abfrage
            if(this->robot_name_ == "robot0")
            {
                // Init services
                this->dyn_rec_inflation_srv_client_ = nh.serviceClient<fpp_msgs::DynReconfigure>("/dyn_reconfig_inflation");
                this->dyn_rec_inflation_srv_client_.waitForExistence();

                // Init topics
                this->formation_footprint_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("formation_footprint", 10);
                while(this->formation_footprint_pub_.getNumSubscribers() < 1)
                    ros::Duration(0.01).sleep();

                // Initialize formation planner with default values. Set lead vector when all robots are added and centroid can be calculated
                this->formation_contour_ = geometry_info::FormationContour(Eigen::Matrix<float, 2, 1>::Zero(), 0.0);

                for(fpp_data_classes::RobotInfo robot_info: this->robot_info_list_)
                {
                    std::string amcl_pose_topic = robot_info.robot_namespace + "/amcl_pose";
                    geometry_msgs::PoseWithCovarianceStampedConstPtr robot_pose_ptr;
                    robot_pose_ptr = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>(amcl_pose_topic);

                    Eigen::Vector2f robot_pose;
                    float yaw;
                    if(robot_pose_ptr == nullptr)
                    {
                        ROS_ERROR_STREAM("FormationPathPlanner: No message was received from the topic: " << amcl_pose_topic);
                        robot_pose << 0, 0;
                        yaw = 0;
                    }
                    else
                    {
                        robot_pose << robot_pose_ptr->pose.pose.position.x, robot_pose_ptr->pose.pose.position.y;
                        yaw = tf::getYaw(robot_pose_ptr->pose.pose.orientation);
                    }

                    geometry_info::GeometryContour robot_outline;
                    robot_outline = geometry_info::GeometryContour(robot_pose, yaw);
                    
                    for(Eigen::Vector2f corner: robot_info.robot_outline)
                    {
                        // ROS_INFO_STREAM("Creating robot contour x: " << corner[0] << " y: " << corner[1]);
                        robot_outline.addContourCornerGeometryCS(corner);
                    }
                    robot_outline.createContourEdges();

                    this->robot_outline_list_.insert(std::pair<std::string, geometry_info::GeometryContour>(robot_info.robot_name, robot_outline));
                    this->formation_contour_.addRobotToFormation(robot_outline);
                }

                // Call services
                fpp_msgs::DynReconfigure dyn_reconfig_msg;
                this->formation_contour_.exeGiftWrappingAlg();
                this->formation_outline_circle_.calcMinimalEnclosingCircle(this->formation_contour_.getCornerPointsWorldCS());
                dyn_reconfig_msg.request.new_inflation_radius = this->formation_outline_circle_.getCircleRadius();
                dyn_reconfig_msg.request.robot_namespace = this->robot_ns_ ;
                ros::Duration(0.1).sleep();
                this->dyn_rec_inflation_srv_client_.call(dyn_reconfig_msg);

                geometry_msgs::PolygonStamped formation_footprint_msg;
                formation_footprint_msg.header.frame_id = "map";
                formation_footprint_msg.header.stamp = ros::Time::now();
                for(Eigen::Vector2f corner: this->formation_contour_.getCornerPointsWorldCS())
                {
                    geometry_msgs::Point32 corner_point;
                    corner_point.x = corner[0];
                    corner_point.y = corner[1];
                    ROS_INFO_STREAM("x: " << corner_point.x << " y: " << corner_point.y);
                    corner_point.z = 0.0;
                    formation_footprint_msg.polygon.points.push_back(corner_point);
                }
                this->formation_footprint_pub_.publish(formation_footprint_msg);
            }

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

                        ROS_INFO_STREAM("outline full: " << robot_outline_key);

                        robot_info.robot_outline = this->createRobotOutlineFromXMLRPC(robot_outline, robot_outline_key);
                    }
                }

                this->robot_info_list_.push_back(robot_info);
            }
        }
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