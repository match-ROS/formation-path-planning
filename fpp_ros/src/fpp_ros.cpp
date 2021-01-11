#include <fpp_ros/fpp_ros.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(fpp::FormationPathPlanner, mbf_costmap_core::CostmapPlanner)
PLUGINLIB_EXPORT_CLASS(fpp::FormationPathPlanner, nav_core::BaseGlobalPlanner)

namespace fpp
{
    FormationPathPlanner::FormationPathPlanner()
        : initialized_(false)
    {
        ROS_ERROR("FORMATION PATH PLANNER DEFAULT CONSTRUCTOR");
    }

    FormationPathPlanner::FormationPathPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
        : initialized_(false)
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
            this->costmap_ = costmap;
            this->global_frame_ = global_frame;
            this->array_size_ = this->costmap_->getSizeInCellsX() * this->costmap_->getSizeInCellsY();

            std::string robot_namespace = ros::this_node::getNamespace();
            robot_namespace = robot_namespace.erase(robot_namespace.find("_ns"), 3);
            this->robot_name_ = robot_namespace.erase(robot_namespace.find("/"), 1);

            // Get parameter of planner
            ros::NodeHandle private_nh("~/" + name);
            private_nh.param<float>("default_tolerance", this->default_tolerance_, 0.0);

            //First check for formation_config and robot0 config
            if(!private_nh.hasParam("formation_config") || !private_nh.hasParam("formation_config/robot0"))
            {
                ROS_ERROR("FormationPathPlanner: Error during initialization. formation_config or robot0 config is missing.");
                return;
            }
            std::string default_robot_name = "robot";
            int default_last_robot = 100;
            for(int robot_counter = 0; robot_counter <= default_last_robot; robot_counter++)
            {
                std::string robot_position_namespace = "formation_config/" + default_robot_name + std::to_string(robot_counter) + "/";
                if(private_nh.hasParam(robot_position_namespace))
                {
                    geometry_msgs::Pose robot_position;
                    private_nh.param<double>(robot_position_namespace + "x", robot_position.position.x, 0.0);
                    private_nh.param<double>(robot_position_namespace + "y", robot_position.position.y, 0.0);
                    private_nh.param<double>(robot_position_namespace + "z", robot_position.position.z, 0.0);
                    double yaw;
                    private_nh.param<double>(robot_position_namespace + "yaw", yaw, 0.0);
                    tf::Quaternion robot_orientation;
                    robot_orientation.setEuler(yaw, 0.0, 0.0);
                    tf::quaternionTFToMsg(robot_orientation, robot_position.orientation);

                    this->robot_positions_.insert(std::pair<std::string, geometry_msgs::Pose>(default_robot_name + std::to_string(robot_counter),
                                                                                              robot_position));
                }
            }

            // Get the tf prefix
            ros::NodeHandle nh;
            this->tf_prefix_ = tf::getPrefixParam(nh);

            this->formation_outline_circle_ = fpp_helper::MinimalEnclosingCircle();
            std::vector<Eigen::Vector2d> robot_positions;
            for(std::pair<std::string, geometry_msgs::Pose> robot_position: this->robot_positions_)
            {
                Eigen::Vector2d position;
                position << robot_position.second.position.x, robot_position.second.position.y;
                robot_positions.push_back(position);
            }
            this->formation_outline_circle_.calcMinimalEnclosingCircle(robot_positions);

            // this->temp_ = nh.serviceClient<dynamic_reconfigure::Reconfigure>("/robot1_ns/move_base_flex/global_costmap/inflation/set_parameters");
            // this->temp_.waitForExistence();

            // ROS_INFO("1");
            // dynamic_reconfigure::Reconfigure rec;
            // ROS_INFO("2");
            // dynamic_reconfigure::DoubleParameter param_to_reconfig;
            // dynamic_reconfigure::BoolParameter test_param;
            // ROS_INFO("3");
            // // param_to_reconfig.name = "inflation_radiusss";
            // // param_to_reconfig.value = 5.0;
            // test_param.name="enabled";
            // test_param.value = true;
            // ROS_INFO("5");
            // rec.request.config.bools.push_back(test_param);
            // // rec.request.config.doubles.push_back(param_to_reconfig);
            // // ROS_INFO("size: %i", rec.request.config.doubles.size());
            // ROS_INFO("6");
            // this->temp_.call(rec);
            // //ros::service::call("/robot1_ns/move_base_flex/global_costmap/inflation/set_parameters", reconf_req, reconf_res);
            // ROS_INFO("7");
            // ROS_INFO("%i", rec.response.config.doubles.size());

            // ROS_INFO("1");
            // dynamic_reconfigure::Client<costmap_2d::InflationPluginConfig> client("/robot1_ns/move_base_flex/global_costmap/inflation");
            // ROS_INFO("2");
            // costmap_2d::InflationPluginConfig cfg;
            // ROS_INFO("3");
            // cfg = costmap_2d::InflationPluginConfig::__getMax__();
            // ROS_INFO("4");
            // client.setConfiguration(cfg);
            // ROS_INFO("5");

            ROS_INFO("%s, 0", nh.getNamespace().c_str());
            
            temp_ = nh.serviceClient<dynamic_reconfigure::Reconfigure>("/robot1_ns/move_base_flex/global_costmap/inflation/set_parameters");
            temp_.waitForExistence();

            ROS_INFO("1");
            dynamic_reconfigure::Reconfigure rec;
            ROS_INFO("2");
            dynamic_reconfigure::DoubleParameter param_to_reconfig;
            ROS_INFO("3");
            param_to_reconfig.name = "inflation_radius";
            param_to_reconfig.value = 5.0;
            ROS_INFO("5");
            rec.request.config.doubles.push_back(param_to_reconfig);
            ROS_INFO("size: %i", rec.request.config.doubles.size());
            ROS_INFO("6");
            // temp_.call(rec);
            //ros::service::call("/robot1_ns/move_base_flex/global_costmap/inflation/set_parameters", reconf_req, reconf_res);
            ROS_INFO("7");
            ROS_INFO("size: %i", rec.response.config.doubles.size());
            ROS_INFO("LIST:");
            for(dynamic_reconfigure::DoubleParameter param: rec.response.config.doubles)
            {
                ROS_INFO("name: %s, value: %f", param.name.c_str(), param.value);
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

        ROS_ERROR("RELAXED A STAR MAKEPLAN");

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

    }
}