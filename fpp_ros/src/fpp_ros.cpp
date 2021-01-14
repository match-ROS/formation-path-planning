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
            this->path_planner_name_ = name;
            this->costmap_ = costmap;
            this->global_frame_ = global_frame;
            
            this->array_size_ = this->costmap_->getSizeInCellsX() * this->costmap_->getSizeInCellsY();
            
            std::string robot_namespace = ros::this_node::getNamespace();
            robot_namespace = robot_namespace.erase(robot_namespace.find("_ns"), 3);
            this->robot_name_ = robot_namespace.erase(robot_namespace.find("/"), 1);

            this->getParams();

            // Get the tf prefix
            ros::NodeHandle nh;
            this->tf_prefix_ = tf::getPrefixParam(nh);
            this->robot_ns_ = nh.getNamespace();



            
            


            this->dyn_rec_inflation_srv_client_ = nh.serviceClient<fpp_msgs::DynReconfigure>("/dyn_reconfig_inflation");
            this->dyn_rec_inflation_srv_client_.waitForExistence();
            fpp_msgs::DynReconfigure dyn_reconfig_msg;
            dyn_reconfig_msg.request.new_inflation_radius = this->formation_outline_circle_.getCircleRadius();
            dyn_reconfig_msg.request.robot_namespace = this->robot_ns_ ;
            this->dyn_rec_inflation_srv_client_.call(dyn_reconfig_msg);

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
        // Temp variables
        geometry_msgs::Polygon robot_outline;

        // Get parameter of planner
        ros::NodeHandle private_nh("~/" + this->path_planner_name_);
        private_nh.param<float>("default_tolerance", this->default_tolerance_, 0.0);
        private_nh.param<geometry_msgs::Polygon>("robot_outline", robot_outline, geometry_msgs::Polygon());

        // First check for formation_config and robot0 config
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
                fpp::MuR205FormationInfo robot_info;
                robot_info.robot_name = default_robot_name + std::to_string(robot_counter);
                private_nh.param<double>(robot_position_namespace + "x", robot_info.base_link_pose.position.x, 0.0);
                private_nh.param<double>(robot_position_namespace + "y", robot_info.base_link_pose.position.y, 0.0);
                private_nh.param<double>(robot_position_namespace + "z", robot_info.base_link_pose.position.z, 0.0);
                double yaw;
                private_nh.param<double>(robot_position_namespace + "yaw", yaw, 0.0);
                tf::Quaternion robot_orientation;
                robot_orientation.setEuler(yaw, 0.0, 0.0);
                tf::quaternionTFToMsg(robot_orientation, robot_info.base_link_pose.orientation);
                
                robot_info.robot_outline = robot_outline;

                this->robot_info_list_.insert(std::pair<std::string, fpp::MuR205FormationInfo>(
                    robot_info.robot_name, robot_info));
            }
        }
    }

    void FormationPathPlanner::calcFormationEnclosingCircle()
    {
        this->formation_outline_circle_ = fpp_helper::MinimalEnclosingCircle();
        std::vector<Eigen::Vector2d> robot_positions;
        for(std::pair<std::string, MuR205FormationInfo> robot_info: this->robot_info_list_)
        {
            
            
        }
        this->formation_outline_circle_.calcMinimalEnclosingCircle(robot_positions);
    }

    Eigen::Vector2d FormationPathPlanner::MsgsPoseToEigenVector2d(geometry_msgs::Pose pose_to_convert)
    {
        Eigen::Vector2d converted_position;
        converted_position << pose_to_convert.position.x, pose_to_convert.position.y;
        return converted_position;
    }
}