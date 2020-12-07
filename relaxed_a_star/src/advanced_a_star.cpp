#include <advanced_a_star/advanced_a_star.h>
#include <pluginlib/class_list_macros.h>

namespace advanced_a_star
{
    AdvancedAStar::AdvancedAStar()
        : initialized_(false)
    {
        ROS_ERROR("RELAXED A STAR DEFAULT CONSTRUCTOR");
    }

    AdvancedAStar::AdvancedAStar(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
        : initialized_(false)
    {
        ROS_ERROR("RELAXED A STAR OVERLOADED CONSTRUCTOR");
        this->initialize(name, costmap_ros);
    }

    void AdvancedAStar::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if(this->initialized_)
        {
            this->costmap_ros_ = costmap_ros_;
            this->initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
        }
    }

    void AdvancedAStar::initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string global_frame)
    {
        if(this->initialized_)
        {
            ROS_INFO("Initializing AdvancedAStar planner.");

            //Safe parameter for planning
            this->costmap_ = costmap;
            this->global_frame_ = global_frame;
            this->array_size_ = this->costmap_->getSizeInCellsX() * this->costmap_->getSizeInCellsY();

            // Get parameter of planner
            ros::NodeHandle private_nh("~/" + name);
            private_nh.param<float>("default_tolerance", this->default_tolerance_, 0.0);
            int neighbor_type;
            private_nh.param<int>("neighbor_type", neighbor_type, static_cast<int>(general_types::NeighborType::FourWay));
            this->neighbor_type_ = (general_types::NeighborType)neighbor_type;
            int free_neighbor_mode;
            private_nh.param<int>("free_neighbor_mode", free_neighbor_mode, 0);
            this->free_neighbor_mode_ = (general_types::FreeNeighborMode)free_neighbor_mode;
            private_nh.param<float>("maximal_curvature", this->maximal_curvature_, 20);
            private_nh.param<int>("curvature_calculation_cell_distance", this->curvature_calculation_cell_distance_, 4);

            this->plan_publisher_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
            this->planning_points_orientation_publisher_ = private_nh.advertise<geometry_msgs::PoseArray>("planning_points_orientation", 1);

            this->visu_helper_ = visualization_helper::VisualizationHelper(name);
            this->g_score_marker_array_id_ = this->visu_helper_.addNewMarkerArray();
            visualization_msgs::Marker marker_template_g_score;
            marker_template_g_score.action = visualization_msgs::Marker::ADD;
            marker_template_g_score.color.a = 1.0;
            marker_template_g_score.color.r = 1.0;
            marker_template_g_score.header.frame_id = "map";
            marker_template_g_score.lifetime = ros::Duration(0.3);
            marker_template_g_score.ns = "g_score";
            marker_template_g_score.scale.x = 0.1;
            marker_template_g_score.scale.y = 0.1;
            marker_template_g_score.scale.z = 0.1;
            marker_template_g_score.type = visualization_msgs::Marker::SPHERE;
            this->g_score_marker_template_id_ = this->visu_helper_.addMarkerTemplate(marker_template_g_score);
            
            this->theoretical_path_marker_array_id_ = this->visu_helper_.addNewMarkerArray();
            visualization_msgs::Marker marker_template_theoretical_path;
            marker_template_theoretical_path.action = visualization_msgs::Marker::ADD;
            marker_template_theoretical_path.color.a = 1.0;
            marker_template_theoretical_path.color.b = 1.0;
            marker_template_theoretical_path.header.frame_id = "map";
            marker_template_theoretical_path.lifetime = ros::Duration(0.3);
            marker_template_theoretical_path.ns = "theoretical_path";
            marker_template_theoretical_path.scale.x = 0.1;
            marker_template_theoretical_path.scale.y = 0.1;
            marker_template_theoretical_path.scale.z = 0.1;
            marker_template_theoretical_path.type = visualization_msgs::Marker::SPHERE;
            this->theoretical_path_marker_template_id_ = this->visu_helper_.addMarkerTemplate(marker_template_theoretical_path);

             // Get the tf prefix
            ros::NodeHandle nh;
            this->tf_prefix_ = tf::getPrefixParam(nh);

            initialized_ = true; // Initialized method was called so planner is now initialized

            this->createOccupancyMap();

            ROS_INFO("Advanced AStar planner finished intitialization.");
        }
        else
        {
            ROS_WARN("The AdvancedAStar planner has already been initialized");
        }
    }

    bool AdvancedAStar::makePlan(const geometry_msgs::PoseStamped& start, 
                const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {
        double cost;
        std::string message;
        return 10 > this->makePlan(start, goal, this->default_tolerance_, plan, cost, message);
    }  

    uint32_t AdvancedAStar::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                double tolerance, std::vector<geometry_msgs::PoseStamped> &plan, double &cost,
                                std::string &message)
    {
        if(!this->initialized_)
        {
            ROS_ERROR("RelaxedAStar planner was not initialized yet. Please initialize the planner before usage.");
            return mbf_msgs::GetPathResult::NOT_INITIALIZED;
        }

        ROS_ERROR("RELAXED A STAR MAKEPLAN");

        this->start_ = start;
        this->goal_ = goal;

        plan.clear(); // Clear path in case anything is already in it
        this->createOccupancyMap(); // Costmap can change so the occupancy map must be updated bevor making plan

        // Check with tf that goal and start frame is the global frame and not any other
        if(tf::resolve(this->tf_prefix_, goal.header.frame_id) != tf::resolve(this->tf_prefix_, this->global_frame_)){
            message = "The goal pose passed to this planner must be in the planner's global frame";
            ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
                        tf::resolve(this->tf_prefix_, this->global_frame_).c_str(), 
                        tf::resolve(this->tf_prefix_, goal.header.frame_id).c_str());
            return mbf_msgs::GetPathResult::INVALID_GOAL;
        }
        
        if(tf::resolve(this->tf_prefix_, start.header.frame_id) != tf::resolve(this->tf_prefix_, this->global_frame_)){
            message = "The start pose passed to this planner must be in the planner's global frame";
            ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
                        tf::resolve(this->tf_prefix_, this->global_frame_).c_str(), 
                        tf::resolve(this->tf_prefix_, start.header.frame_id).c_str());
            return mbf_msgs::GetPathResult::INVALID_START;
        }

        // Check if the start position is in the map
        double world_start_pose_x = start.pose.position.x;
        double world_start_pose_y = start.pose.position.y;
        unsigned int map_start_cell_x, map_start_cell_y;
        if(!this->costmap_->worldToMap(world_start_pose_x, world_start_pose_y, map_start_cell_x, map_start_cell_y))
        {
            message = "The start position of the robot is not inside the costmap";
            ROS_ERROR("The start position of the robot is not inside the costmap. Planning would always fail, are you sure the robot has been properly localized?");
            return mbf_msgs::GetPathResult::INVALID_START;
        }

        // The point where the robot is currently located is definetly clear, so clear the start cell
        tf::Stamped<tf::Pose> world_start_pose;
        tf::poseStampedMsgToTF(start, world_start_pose);
        this->costmap_->setCost(map_start_cell_x, map_start_cell_y, costmap_2d::FREE_SPACE);

        // Safe the start position for the planner
        int map_start_cell[2]; // I will save the position as array and not some ros-pose type so it can easily be extracted into a separate class independent of ros
        map_start_cell[0] = map_start_cell_x;
        map_start_cell[1] = map_start_cell_y;
        int array_start_cell = this->getArrayIndexByCostmapCell(map_start_cell);
        
        // Check if the goal position is in the map
        double world_goal_pose_x = goal.pose.position.x;
        double world_goal_pose_y = goal.pose.position.y;
        unsigned int map_goal_cell_x, map_goal_cell_y;

        if (tolerance == 0.0) // For the moment I will use the default tolerance if somebody want 0 tolerance at the goal. Later this should throw an error or warning!
        {
            tolerance = this->default_tolerance_;
        }

        if(!this->costmap_->worldToMap(world_goal_pose_x, world_goal_pose_y, map_goal_cell_x, map_goal_cell_y) ||
            tolerance <= 0.0) // If goal is not in the map or the tolerance is too little than path is not possible
        {
            message = "The goal position of the robot is not inside the costmap or the tolerance is too tight";
            ROS_ERROR("The start position of the robot is not inside the costmap or the tolerance is too tight. Planning would always fail.");
            return mbf_msgs::GetPathResult::INVALID_GOAL;
        }

        // Safe the goal position for the planner
        int map_goal_cell[2];
        map_goal_cell[0] = map_goal_cell_x;
        map_goal_cell[1] = map_goal_cell_y;
        int array_goal_cell = this->getArrayIndexByCostmapCell(map_goal_cell);

        // Beginn of AStar planner
        std::vector<int> closed_cell_list;
        float fTieBreaker = 1 + (1/(this->costmap_->getSizeInCellsX() + this->costmap_->getSizeInCellsY()));

        
    }

    int AdvancedAStar::getArrayIndexByCostmapCell(int *map_cell)
    {
        return map_cell[1] * this->costmap_->getSizeInCellsX() + map_cell[0];
    }

    int AdvancedAStar::getArrayIndexByCostmapCell(int map_cell_x, int map_cell_y)
    {
        int map_cell[2] = {map_cell_x, map_cell_y};
        return this->getArrayIndexByCostmapCell(map_cell);
    }

    void AdvancedAStar::getCostmapPointByArrayIndex(int array_index, int *map_cell)
    {
        map_cell[1] = array_index / this->costmap_->getSizeInCellsX();
        map_cell[0] = array_index % this->costmap_->getSizeInCellsX();
    }

    void AdvancedAStar::getCostmapPointByArrayIndex(int array_index, int &map_cell_x, int &map_cell_y)
    {
        int map_cell[2];
        this->getCostmapPointByArrayIndex(array_index, map_cell);
        map_cell_x = map_cell[0];
        map_cell_y = map_cell[1];
    }

    void AdvancedAStar::createOccupancyMap()
    {
        this->occupancy_map_ = std::shared_ptr<bool[]>(new bool[this->array_size_]);

        // Check every cell of the costmap, if the value is above a vertain threshold
        for(int cell_counter_x = 0; cell_counter_x < this->costmap_->getSizeInCellsX(); cell_counter_x++)
        {
            for(int cell_counter_y = 0; cell_counter_y < this->costmap_->getSizeInCellsY(); cell_counter_y++)
            {
                unsigned int cell_cost = (unsigned int)this->costmap_->getCost(cell_counter_x, cell_counter_y);
                
                if(cell_cost == 0)
                {
                    this->occupancy_map_[this->getArrayIndexByCostmapCell(cell_counter_x, cell_counter_y)] = false;
                }
                else
                {
                    this->occupancy_map_[this->getArrayIndexByCostmapCell(cell_counter_x, cell_counter_y)] = true;
                }
            }
        }
    }
}