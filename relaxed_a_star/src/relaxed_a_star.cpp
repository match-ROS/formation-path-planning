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
            ROS_INFO("Initializing RelaxedAStar planner.");
            // Safe parameter for planning
            this->costmap_ = costmap;
            this->global_frame_ = global_frame;
            this->array_size_ = this->costmap_->getSizeInCellsX() * this->costmap_->getSizeInCellsY();
            this->initializeOccupancyMap();

            // Get parameter of planner
            ros::NodeHandle private_nh("~/" + name);
            private_nh.param<float>("default_tolerance", this->default_tolerance_, 0.0);
            int neighbor_type;
            private_nh.param<int>("neighbor_type", neighbor_type, static_cast<int>(NeighborType::FourWay));
            this->neighbor_type_ = static_cast<NeighborType>(neighbor_type);

            this->plan_publisher_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

            // Get the tf prefix
            ros::NodeHandle nh;
            this->tf_prefix_ = tf::getPrefixParam(nh);

            initialized_ = true; // Initialized method was called so planner is now initialized

            ROS_INFO("Relaxed AStar finished intitialization.");
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
        int map_start[2]; // I will save the position as array and not some ros-pose type so it can easily be extracted into a separate class independent of ros
        map_start[0] = map_start_cell_x;
        map_start[1] = map_start_cell_y;
        int start_cell_index = this->getArrayIndexByCostmapCell(map_start);
        
        // Check if the goal position is in the map
        // double world_start_pose_x = start.pose.position.x;
        // double world_start_pose_y = start.pose.position.y;
        // unsigned int map_start_cell_x, map_start_cell_y;
        //HIER WEITER MACHEN!!!
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
        int goal_cell_index = this->getArrayIndexByCostmapCell(map_goal);

        // Begin of Relaxed_A_Star
        float fTieBreaker = 1 + (1/(this->costmap_->getSizeInCellsX() + this->costmap_->getSizeInCellsY()));

        // Initialization 
        std::multiset<Cell, std::greater<Cell>> open_cell_set;
        open_cell_set.insert({this->getArrayIndexByCostmapCell(map_start),
                              this->calcHCost(map_start, map_goal)});
        
        // 1D-Array
        std::shared_ptr<float[]> g_score(new float[this->array_size_]);
        for(int counter = 0; counter < this->array_size_; counter++)
        {
            g_score[counter] = std::numeric_limits<float>::infinity();
        }
        g_score[this->getArrayIndexByCostmapCell(map_start)] = 0;

        // Start planning
        while (!open_cell_set.empty() &&
               g_score[this->getArrayIndexByCostmapCell(map_goal)] == std::numeric_limits<float>::infinity())
        {
            ROS_INFO("Open_Cell_Count: %i", open_cell_set.size());
            int current_cell_index = open_cell_set.begin()->cell_index; // Get cell with lowest f_score
            open_cell_set.erase(open_cell_set.begin()); // Remove cell from open_cell_set so it will not be visited again

            std::vector<int> neighbor_cell_list = this->getFreeNeighborCells(current_cell_index);
            for(int neighbor_cell_index: neighbor_cell_list)
            {
                if(g_score[neighbor_cell_index] == std::numeric_limits<float>::infinity())
                {
                    g_score[neighbor_cell_index] = this->calcGCost(g_score[current_cell_index], current_cell_index, neighbor_cell_index);
                    open_cell_set.insert({neighbor_cell_index,
                                          this->calcFCost(g_score[current_cell_index],
                                                          current_cell_index,
                                                          goal_cell_index)});
                }
            }
        }
        if (g_score[goal_cell_index] != std::numeric_limits<float>::infinity())
        {
            int current_cell_index = goal_cell_index;
            std::vector<int> goal_to_start_path;
            std::vector<int> start_to_goal_path;

            // Construct path backwards from goal to start to get best path
            goal_to_start_path.push_back(goal_cell_index);
            while(current_cell_index != start_cell_index)
            {
                int next_cell_index = this->getMinGScoreNeighborCell(current_cell_index, g_score);
                goal_to_start_path.push_back(next_cell_index);
                current_cell_index = next_cell_index;
            }

            for(uint path_counter = 0; path_counter < goal_to_start_path.size(); path_counter++)
            {
                start_to_goal_path.insert(start_to_goal_path.begin() + start_to_goal_path.size(),
                                          goal_to_start_path[goal_to_start_path.size() - path_counter]);
            }

            for(uint path_counter = 0; path_counter < start_to_goal_path.size(); path_counter++)
            {
                geometry_msgs::PoseStamped pose = goal;
                int costmap_point[2];
                this->getCostmapPointByArrayIndex(start_to_goal_path[path_counter], costmap_point);
                ROS_INFO("%f, %f", costmap_point[0], costmap_point[1]);
                pose.pose.position.x = costmap_point[0];
                pose.pose.position.y = costmap_point[1];
                plan.insert(plan.begin() + plan.size(), pose);
            }
        }
        ROS_INFO("Planning finished");
        ROS_INFO("Plan contains %i elements", plan.size());
        this->publishPlan(plan);
        return 0;
    }

    bool RelaxedAStar::cancel()
    {
        ROS_ERROR("RELAXED A STAR CANCEL");
        return false; // Returns false if not implemented. Returns true if implemented and cancel has been successful.
    }


    float RelaxedAStar::calcGCost(int current_g_cost, int array_current_cell, int array_target_cell)
    {
        return current_g_cost + this->calcMoveCost(array_current_cell, array_target_cell);
    }

    float RelaxedAStar::calcHCost(int* map_current_cell, int* map_goal_cell)
    {
        // Calc euclidean distance and return
        return std::sqrt(std::pow(map_current_cell[0] - map_goal_cell[0], 2) +
                         std::pow(map_current_cell[1] - map_goal_cell[1], 2));
    }

    float RelaxedAStar::calcHCost(int array_current_cell, int array_goal_cell)
    {
        int map_current_cell[2];
        int map_goal_cell[2];
        this->getCostmapPointByArrayIndex(array_current_cell, map_current_cell);
        this->getCostmapPointByArrayIndex(array_goal_cell, map_goal_cell);
        return this->calcHCost(map_current_cell, map_goal_cell);
    }

    float RelaxedAStar::calcFCost(float current_g_score, int array_current_cell, int array_goal_cell)
    {
        return current_g_score + this->calcHCost(array_current_cell, array_goal_cell);
    }

    int RelaxedAStar::getArrayIndexByCostmapCell(int *map_cell)
    {
        return map_cell[1] * this->costmap_->getSizeInCellsX() + map_cell[0];
    }

    int RelaxedAStar::getArrayIndexByCostmapCell(int map_cell_x, int map_cell_y)
    {
        int map_cell[2] = {map_cell_x, map_cell_y};
        return this->getArrayIndexByCostmapCell(map_cell);
    }

    void RelaxedAStar::getCostmapPointByArrayIndex(int array_index, int *map_cell)
    {
        map_cell[1] = array_index / this->costmap_->getSizeInCellsX();
        map_cell[0] = array_index % this->costmap_->getSizeInCellsX();
    }

    std::vector<int> RelaxedAStar::getFreeNeighborCells(int array_current_cell)
    {
        int map_cell[2];
        std::vector<int> map_neighbor_cell_list;
        this->getCostmapPointByArrayIndex(array_current_cell, map_cell);

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

                    int cell_index = this->getArrayIndexByCostmapCell(map_cell[0] + counter_x,
                                                                       map_cell[1] + counter_y); 
                    if (this->isCellFree(cell_index))
                    {
                        map_neighbor_cell_list.push_back(cell_index);
                    }
                }
            }
        }

        return map_neighbor_cell_list;
    }
    
    int RelaxedAStar::getMinGScoreNeighborCell(int current_cell_index, std::shared_ptr<float[]> g_score)
    {
        int min_g_score_cell_index = current_cell_index;
        std::vector<int> free_neighbor_cell_indexes = this->getFreeNeighborCells(current_cell_index);
        for(int cell_index: free_neighbor_cell_indexes)
        {
            if(g_score.get()[min_g_score_cell_index] > g_score.get()[cell_index])
            {
                min_g_score_cell_index = cell_index;
            }
        }
        return min_g_score_cell_index;
    }

    void RelaxedAStar::initializeOccupancyMap()
    {
        this->occupancy_map_ = std::shared_ptr<bool[]>(new bool[this->array_size_]);
        for(int cell_counter_x = 0; cell_counter_x < this->costmap_->getSizeInCellsX(); cell_counter_x++)
        {
            for(int cell_counter_y = 0; cell_counter_y < this->costmap_->getSizeInCellsY(); cell_counter_y++)
            {
                unsigned int cell_cost = static_cast<unsigned int>(this->costmap_->getCost(cell_counter_x, cell_counter_y));
                int map_cell[2] = {cell_counter_x, cell_counter_y};
                if(cell_cost == 0) // Cell is free
                {
                    this->occupancy_map_[0] = false; // False because cell is not occupied
                }
                else
                {
                    this->occupancy_map_[this->getArrayIndexByCostmapCell(map_cell)] = true;  // True because cell is occupied
                }
            }
        }
    }

    bool RelaxedAStar::isCellFree(int array_cell_index)
    {
        return !this->occupancy_map_[array_cell_index]; // Negate because function is "isFree" but occupancy_map_ defines with true where it is blocked
    }

    float RelaxedAStar::calcMoveCost(int array_current_cell, int array_target_cell)
    {
        int map_current_cell[2];
        int map_target_cell[2];
        this->getCostmapPointByArrayIndex(array_current_cell, map_current_cell);
        this->getCostmapPointByArrayIndex(array_target_cell, map_target_cell);
        return this->calcMoveCost(map_current_cell, map_target_cell);
    }

    float RelaxedAStar::calcMoveCost(int* map_current_cell, int* map_target_cell)
    {
        return this->calcMoveCost(map_current_cell[0],
                                  map_current_cell[1],
                                  map_target_cell[0],
                                  map_target_cell[1]);
    }

    float RelaxedAStar::calcMoveCost(int map_current_cell_x, int map_current_cell_y, int map_target_cell_x, int map_target_cell_y)
    {
        float test1=pow(map_current_cell_x - map_target_cell_x, 2);
        float test2=pow(map_current_cell_y - map_target_cell_y, 2);
        float test3=sqrt(pow(map_current_cell_x - map_target_cell_x, 2) + pow(map_current_cell_y - map_target_cell_y, 2));
        return sqrt(pow(map_current_cell_x - map_target_cell_x, 2) + pow(map_current_cell_y - map_target_cell_y, 2));
    }

    void RelaxedAStar::publishPlan(std::vector<geometry_msgs::PoseStamped> &plan)
    {
        nav_msgs::Path path_to_publish;
        path_to_publish.header.stamp = ros::Time::now();
        path_to_publish.header.frame_id = plan[0].header.frame_id;

        path_to_publish.poses = plan;

        this->plan_publisher_.publish(path_to_publish);
    }
}