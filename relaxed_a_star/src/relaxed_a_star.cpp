#include <relaxed_a_star/relaxed_a_star.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(relaxed_a_star::RelaxedAStar, mbf_costmap_core::CostmapPlanner)
PLUGINLIB_EXPORT_CLASS(relaxed_a_star::RelaxedAStar, nav_core::BaseGlobalPlanner)

namespace relaxed_a_star
{

    RelaxedAStar::RelaxedAStar()
        : initialized_(false)
    {
        ROS_ERROR("RELAXED A STAR DEFAULT CONSTRUCTOR");
    }

    RelaxedAStar::RelaxedAStar(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
        : initialized_(false)
    {
        ROS_ERROR("RELAXED A STAR OVERLOADED CONSTRUCTOR");
        this->initialize(name, costmap_ros);
    }

    void RelaxedAStar::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        this->costmap_ros_ = costmap_ros_;
        this->initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
    }

    void RelaxedAStar::initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string global_frame)
    {   
        if(!this->initialized_)
        {
            // Safe parameter for planning
            this->costmap_ = costmap;
            this->global_frame_ = global_frame;

            // Get parameter of planner
            ros::NodeHandle private_nh("~/" + name);
            private_nh.param<float>("default_tolerance", this->default_tolerance_, 0.0);

            // Get the tf prefix
            ros::NodeHandle nh;
            this->tf_prefix_ = tf::getPrefixParam(nh);

            initialized_ = true; // Initialized method was called so planner is now initialized
        }
        else
        {
            ROS_WARN("This planner has already been initialized");
        }
    }    

    bool RelaxedAStar::makePlan(const geometry_msgs::PoseStamped& start, 
                const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {
        double cost;
        std::string message;
        return 10 > this->makePlan(start, goal, this->default_tolerance_, plan, cost, message);
    }  

    uint32_t RelaxedAStar::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                double tolerance, std::vector<geometry_msgs::PoseStamped> &plan, double &cost,
                                std::string &message)
    {
        if(!initialized_) //Planner was not initialized. Abort
        {
            ROS_ERROR("RelaxedAStar planner was not initialized yet. Please initialize the planner before usage.");
            return mbf_msgs::GetPathResult::NOT_INITIALIZED;
        }

        ROS_ERROR("RELAXED A STAR MAKEPLAN");

        plan.clear(); // Clear path in case anything is already in it

        // Check with tf that goal and start frame is the global frame and not any other
        if(tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame_)){
            message = "The goal pose passed to this planner must be in the planner's global frame";
            ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
                        tf::resolve(tf_prefix_, global_frame_).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
            return mbf_msgs::GetPathResult::INVALID_GOAL;
        }

        if(tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, global_frame_)){
            message = "The start pose passed to this planner must be in the planner's global frame";
            ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
                        tf::resolve(tf_prefix_, global_frame_).c_str(), tf::resolve(tf_prefix_, start.header.frame_id).c_str());
            return mbf_msgs::GetPathResult::INVALID_START;
        }

        // Check if the start position is in the map
        double world_x = start.pose.position.x;
        double world_y = start.pose.position.y;
        unsigned int map_x, map_y;
        if(!this->costmap_->worldToMap(world_x, world_y, map_x, map_y))
        {
            message = "The start position of the robot is not inside the costmap";
            ROS_ERROR("The start position of the robot is not inside the costmap. Planning would always fail, are you sure the robot has been properly localized?");
            return mbf_msgs::GetPathResult::INVALID_START;
        }

        // The point where the robot is currently located is definetly clear, so clear the start cell
        tf::Stamped<tf::Pose> start_pose;
        tf::poseStampedMsgToTF(start, start_pose);
        this->costmap_->setCost(map_x, map_y, costmap_2d::FREE_SPACE);

        // TODO
        // Safe the start position for the planner here!

        // Check if the goal position is in the map
        world_x = goal.pose.position.x;
        world_y = goal.pose.position.y;
        if(!this->costmap_->worldToMap(world_x, world_y, map_x, map_y) ||
            tolerance <= 0.0) // If goal is not in the map or the tolerance is too little than path is not possible
        {
            message = "The goal position of the robot is not inside the costmap or the tolerance is too tight";
            ROS_ERROR("The start position of the robot is not inside the costmap or the tolerance is too tight. Planning would always fail.");
            return mbf_msgs::GetPathResult::INVALID_GOAL;
        }

        // TODO
        // Safe the goal position for the planner here!
    }

    bool RelaxedAStar::cancel()
    {
        ROS_ERROR("RELAXED A STAR CANCEL");
        return false; // Returns false if not implemented. Returns true if implemented and cancel has been successful.
    }
}