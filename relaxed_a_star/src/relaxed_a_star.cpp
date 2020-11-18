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

            for(int counter_x = 0; counter_x < this->costmap_->getSizeInCellsX(); counter_x++)
            {
                for(int counter_y = 0; counter_y < this->costmap_->getSizeInCellsY(); counter_y++)
                {
                    unsigned int cell_cost = static_cast<unsigned int>(this->costmap_->getCost(counter_x, counter_y));
                    int map_point[2] = {counter_x, counter_y};
                    if(cell_cost == 0) // Cell is free
                    {
                        this->occupancy_map_[this->getArrayIndexByCostmapPoint(map_point)] = false; // False because cell is not occupied
                    }
                    else
                    {
                        this->occupancy_map_[this->getArrayIndexByCostmapPoint(map_point)] = true;  // True because cell is occupied
                    }
                }
            }

            // Get parameter of planner
            ros::NodeHandle private_nh("~/" + name);
            private_nh.param<float>("default_tolerance", this->default_tolerance_, 0.0);
            int neighbor_type;
            private_nh.param<int>("neighbor_type", neighbor_type, static_cast<int>(NeighborType::FourWay));
            this->neighbor_type_ = static_cast<NeighborType>(neighbor_type);

            // Get the tf prefix
            ros::NodeHandle nh;
            this->tf_prefix_ = tf::getPrefixParam(nh);

            // Initialize process information
            this->map_size_ = this->costmap_->getSizeInCellsX() * this->costmap_->getSizeInCellsY();

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
        
        // Safe the start position for the planner
        int map_start[2]; // I will save the position as array and not some ros-pose type so it can easily be extracted into a separate class independent of ros
        map_start[0] = map_x;
        map_start[1] = map_y;
        
        // Check if the goal position is in the map
        world_x = goal.pose.position.x;
        world_y = goal.pose.position.y;

        if (tolerance == 0.0) // For the moment I will use the default tolerance if somebody want 0 tolerance at the goal. Later this should throw an error or warning!
        {
            tolerance = this->default_tolerance_;
        }
        
        if(!this->costmap_->worldToMap(world_x, world_y, map_x, map_y) ||
            tolerance <= 0.0) // If goal is not in the map or the tolerance is too little than path is not possible
        {
            message = "The goal position of the robot is not inside the costmap or the tolerance is too tight";
            ROS_ERROR("The start position of the robot is not inside the costmap or the tolerance is too tight. Planning would always fail.");
            return mbf_msgs::GetPathResult::INVALID_GOAL;
        }
        
        // Safe the goal position for the planner
        int map_goal[2];
        map_goal[0] = map_x;
        map_goal[1] = map_y;

        // Begin of Relaxed_A_Star
        float fTieBreaker = 1 + (1/(this->costmap_->getSizeInCellsX() + this->costmap_->getSizeInCellsY()));

        // Initialization 
        std::multiset<cell, std::greater<cell>> open_cell_set;
        open_cell_set.insert({this->getArrayIndexByCostmapPoint(map_start),
                              this->calcHeuristicCost(map_start, map_goal)});
        ROS_INFO("Open_cell_set:");
        for(auto it: open_cell_set)
        {
            ROS_INFO("%i | %f", it.cell_index, it.f_cost);
        }
        
        // 1D-Array
        std::shared_ptr<float[]> g_score(new float[this->map_size_]);
        for(int counter = 0; counter < this->map_size_; counter++)
        {
            g_score[counter] = std::numeric_limits<float>::infinity();
        }
        g_score[this->getArrayIndexByCostmapPoint(map_start)] = 0;

        // Start planning
        while (!open_cell_set.empty() &&
               g_score[map_goal[0], map_goal[1]] == std::numeric_limits<float>::infinity())
        {
            ROS_INFO("Open_Cell_Count: %i", open_cell_set.size());
            std::multiset<cell>::iterator current_cell_it = open_cell_set.begin(); // Get cell with lowest f_score
            open_cell_set.erase(current_cell_it); // Remove cell from open_cell_set so it will not be visited again

            std::vector<int> neighbor_cell_list = this->getFreeNeighborCells(current_cell_it->cell_index);


        }
    }

    bool RelaxedAStar::cancel()
    {
        ROS_ERROR("RELAXED A STAR CANCEL");
        return false; // Returns false if not implemented. Returns true if implemented and cancel has been successful.
    }


    float RelaxedAStar::calcHeuristicCost(int* current_cell, int* map_goal)
    {
        // Calc euclidean distance and return
        return std::sqrt(std::pow(current_cell[0] - map_goal[0], 2) + std::pow(current_cell[1] - map_goal[1], 2));
    }

    int RelaxedAStar::getArrayIndexByCostmapPoint(int *map_point)
    {
        return map_point[1] * this->costmap_->getSizeInCellsX() + map_point[0];
    }

    int RelaxedAStar::getArrayIndexByCostmapPoint(int map_x_coord, int map_y_coord)
    {
        int map_point[2] = {map_x_coord, map_y_coord};
        return this->getArrayIndexByCostmapPoint(map_point);
    }

    void RelaxedAStar::getCostmapPointByArrayIndex(int array_index, int *map_point)
    {
        map_point[1] = array_index / this->costmap_->getSizeInCellsX();
        map_point[0] = array_index % this->costmap_->getSizeInCellsX();
    }

    std::vector<int> RelaxedAStar::getFreeNeighborCells(int current_cell_index)
    {
        int map_point[2];
        std::vector<int> neighbor_cell_list;
        this->getCostmapPointByArrayIndex(current_cell_index, map_point);

        for(int counter_x = -1; counter_x <= 1; counter_x++)
        {
            for(int counter_y = -1; counter_y <= 1; counter_y++)
            {
                if(counter_x == 0 && counter_y == 0)
                    continue;

                if (this->neighbor_type_ == NeighborType::EightWay ||
                    (this->neighbor_type_ == NeighborType::FourWay &&
                     ((counter_x == 0 && counter_y == -1) ||
                      (counter_x == -1 && counter_y == 0) ||
                      (counter_x == 1 && counter_y == 0) ||
                      (counter_x == 0 && counter_y == 1))))
                {

                    int cell_index = this->getArrayIndexByCostmapPoint(map_point[0] + counter_x,
                                                                       map_point[1] + counter_y); 
                    if (this->isCellFree(cell_index))
                    {
                        neighbor_cell_list.push_back(cell_index);
                    }
                }
            }
        }

        return neighbor_cell_list;
    }

    bool RelaxedAStar::isCellFree(int cell_index)
    {
        return this->occupancy_map_[cell_index];
    }
}