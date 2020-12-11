#include <advanced_a_star/advanced_a_star.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(advanced_a_star::AdvancedAStar, mbf_costmap_core::CostmapPlanner)
PLUGINLIB_EXPORT_CLASS(advanced_a_star::AdvancedAStar, nav_core::BaseGlobalPlanner)

namespace advanced_a_star
{
    AdvancedAStar::AdvancedAStar()
        : initialized_(false)
    {
        ROS_ERROR("ADVANCED A STAR DEFAULT CONSTRUCTOR");
    }

    AdvancedAStar::AdvancedAStar(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
        : initialized_(false)
    {
        ROS_ERROR("ADVANCED A STAR OVERLOADED CONSTRUCTOR");
        this->initialize(name, costmap_ros);
    }

    void AdvancedAStar::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        ROS_INFO("1");
        if(!this->initialized_)
        {
            ROS_INFO("2");
            this->costmap_ros_ = costmap_ros_;
            this->initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
        }
    }

    void AdvancedAStar::initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string global_frame)
    {
        ROS_INFO("3");
        if(!this->initialized_)
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
            ROS_INFO("mode: %i", free_neighbor_mode);
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
            marker_template_theoretical_path.lifetime = ros::Duration(0.5);
            marker_template_theoretical_path.ns = "theoretical_path";
            marker_template_theoretical_path.scale.x = 0.1;
            marker_template_theoretical_path.scale.y = 0.1;
            marker_template_theoretical_path.scale.z = 0.1;
            marker_template_theoretical_path.type = visualization_msgs::Marker::SPHERE;
            this->theoretical_path_marker_template_id_ = this->visu_helper_.addMarkerTemplate(marker_template_theoretical_path);

            this->open_cell_marker_array_id_ = this->visu_helper_.addNewMarkerArray();
            visualization_msgs::Marker marker_template_open_cell;
            marker_template_open_cell.action = visualization_msgs::Marker::ADD;
            marker_template_open_cell.color.a = 1.0;
            marker_template_open_cell.color.g = 1.0;
            marker_template_open_cell.header.frame_id = "map";
            marker_template_open_cell.lifetime = ros::Duration(0.5);
            marker_template_open_cell.ns = "open_cell";
            marker_template_open_cell.scale.x = 0.1;
            marker_template_open_cell.scale.y = 0.1;
            marker_template_open_cell.scale.z = 0.1;
            marker_template_open_cell.type = visualization_msgs::Marker::SPHERE;
            this->open_cell_marker_template_id_ = this->visu_helper_.addMarkerTemplate(marker_template_open_cell);

             // Get the tf prefix
            ros::NodeHandle nh;
            this->tf_prefix_ = tf::getPrefixParam(nh);

            this->initialized_ = true; // Initialized method was called so planner is now initialized

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
        float tieBreaker = 1 + (1/(this->costmap_->getSizeInCellsX() + this->costmap_->getSizeInCellsY()));

        // Create g_score array for the whole costmap and initialize with infinity so only visited cells get a value
        std::shared_ptr<float[]> g_score(new float[this->array_size_]);
        for(int counter = 0; counter < this->array_size_; counter++)
        {
            g_score[counter] = std::numeric_limits<float>::infinity();
        }
        g_score[array_start_cell] = 0;

        // Find plan
        // The array_open_cell_list list contains all the open cells that were neighbors but not explored.
        // The elements in this list are linking to the index of the one dimensional costmap representation array.
        std::multiset<general_types::Cell, std::less<general_types::Cell>> array_open_cell_list;
        // Add start cell to open list to initialize it
        array_open_cell_list.insert({array_start_cell, this->calcHCost(array_start_cell, array_goal_cell)});

        while (!array_open_cell_list.empty() &&
               g_score[array_goal_cell] == std::numeric_limits<float>::infinity())
        {
            int array_current_cell = array_open_cell_list.begin()->cell_index; // Get cell with lowest f_score
            array_open_cell_list.erase(array_open_cell_list.begin()); // Remove cell from open_cell_set so it will not be visited again

            std::vector<int> array_neighbor_cell_list = this->getFreeNeighborCells(array_current_cell);

            for(int array_neighbor_cell: array_neighbor_cell_list)
            {
                bool valid_cell = false;
                switch(this->free_neighbor_mode_)
                {
                    case general_types::FreeNeighborMode::CostmapOnly:
                        valid_cell = true;
                        break;
                    case general_types::FreeNeighborMode::CostmapAndMinimalCurveRadius:
                        // Get first vector for angle calculation
                        int array_last_cell = array_neighbor_cell;
                        this->visu_helper_.addMarkerToExistingMarkerArray(this->theoretical_path_marker_array_id_,
                                                                          this->createGeometryPose(array_neighbor_cell),
                                                                          this->theoretical_path_marker_template_id_);
                        for(int counter = 0; counter < this->curvature_calculation_cell_distance_; counter++)
                        {
                            array_last_cell = this->getMinGScoreNeighborCell(array_last_cell, g_score);
                            this->visu_helper_.addMarkerToExistingMarkerArray(this->theoretical_path_marker_array_id_,
                                                                              this->createGeometryPose(array_last_cell),
                                                                              this->theoretical_path_marker_template_id_);
                            if(array_last_cell == array_start_cell)
                            {
                                break;
                            }
                        }
                        int array_first_vector_start_cell = array_last_cell;
                        int map_first_vector_start[2];
                        int map_first_vector_end[2];
                        this->getCostmapPointByArrayIndex(array_first_vector_start_cell, map_first_vector_start);
                        this->getCostmapPointByArrayIndex(array_neighbor_cell, map_first_vector_end);
                        int first_vector[2];
                        first_vector[0] = map_first_vector_end[0] - map_first_vector_start[0];
                        first_vector[1] = map_first_vector_end[1] - map_first_vector_start[1];

                        tf::Quaternion first_quaternion;
                        double yaw = std::atan2(first_vector[1], first_vector[0]);
                        first_quaternion.setRPY(0.0, 0.0, yaw);

                        tf::Quaternion second_quaternion;
                        if(array_last_cell != array_start_cell)
                        {
                            for (int counter = this->curvature_calculation_cell_distance_ - 1;
                            counter < (2 * this->curvature_calculation_cell_distance_) - 1;
                            counter++)
                            {
                                array_last_cell = this->getMinGScoreNeighborCell(array_last_cell, g_score);
                                this->visu_helper_.addMarkerToExistingMarkerArray(this->theoretical_path_marker_array_id_,
                                                                                  this->createGeometryPose(array_last_cell),
                                                                                  this->theoretical_path_marker_template_id_);
                                if(array_last_cell == array_start_cell)
                                {
                                    break;
                                }
                            }

                            int array_second_vector_start_cell = array_last_cell;
                            int map_second_vector_start[2];
                            int map_second_vector_end[2];
                            this->getCostmapPointByArrayIndex(array_second_vector_start_cell, map_second_vector_start);
                            this->getCostmapPointByArrayIndex(array_first_vector_start_cell, map_second_vector_end);
                            int second_vector[2];
                            second_vector[0] = map_second_vector_end[0] - map_second_vector_start[0];
                            second_vector[1] = map_second_vector_end[1] - map_second_vector_start[1]; 

                            yaw = std::atan2(second_vector[1], second_vector[0]);
                            second_quaternion.setRPY(0.0, 0.0, yaw);
                        }
                        else
                        {
                            tf::quaternionMsgToTF(this->start_.pose.orientation, second_quaternion);
                        }
                        
                        tf::Quaternion second_quaternion_inv;
                        second_quaternion_inv = second_quaternion;
                        second_quaternion_inv[3] = -second_quaternion_inv[3]; // Negate to get inverse quaternion for next calculation
                        tf::Quaternion diff_quaternion = first_quaternion * second_quaternion_inv;
                        tf::Matrix3x3 m(diff_quaternion);
                        double roll, pitch;
                        m.getRPY(roll, pitch, yaw);
                        double yaw_angle = ((180/M_PI)*yaw);
                        
                        if(std::abs(yaw_angle) < this->maximal_curvature_)
                        {
                            valid_cell = true;
                        }

                        this->visu_helper_.visualizeMarkerArray(this->theoretical_path_marker_array_id_);
                        this->visu_helper_.clearMarkerArray(this->theoretical_path_marker_array_id_);
                        // DEBUGGING AND VISUALIZING
                        break;
                }

                if(valid_cell)
                {
                    // If cell was not discovered yet or is not in open cell list, add it to open cell list
                    if(g_score[array_neighbor_cell] == std::numeric_limits<float>::infinity())
                    {
                        array_open_cell_list.insert({array_neighbor_cell, this->calcFCost(g_score[array_current_cell],
                                                                                          array_current_cell,
                                                                                          array_neighbor_cell,
                                                                                          array_goal_cell)});
                        // array_open_cell_list.insert({array_neighbor_cell, this->calcGCost(g_score[array_current_cell], array_current_cell, array_neighbor_cell)});
                    }

                    float new_neighbor_g_score = this->calcGCost(g_score[array_current_cell], array_current_cell, array_neighbor_cell);
                    //DEBUGGING
                    // int map_current_cell[2];
                    // int map_neighbor_cell[2];
                    // this->getCostmapPointByArrayIndex(array_current_cell, map_current_cell);
                    // this->getCostmapPointByArrayIndex(array_neighbor_cell, map_neighbor_cell);
                    // ROS_INFO("current_x: %i, current_y: %i, neighbor_x: %i, neighbor_y: %i", map_current_cell[0], map_current_cell[1], map_neighbor_cell[0], map_neighbor_cell[1]);
                    // ROS_INFO("current_cell: %f | old: %f | new: %f | move cost: %f", g_score[array_current_cell], g_score[array_neighbor_cell],  new_neighbor_g_score, this->calcMoveCost(array_current_cell, array_neighbor_cell));
                    //DEBUGGING
                    // Update the g_score if the path to the current cell is shorter than the one from before
                    if (g_score[array_neighbor_cell] == std::numeric_limits<float>::infinity() ||
                        new_neighbor_g_score < g_score[array_neighbor_cell])
                    {
                        g_score[array_neighbor_cell] = new_neighbor_g_score;
                    }
                    valid_cell = false;
                }
                // ros::Duration(0.1).sleep();
            }
            //DEBUGGING
            // for(int y = 509; y <= 513; y++)
            // {
            //     for(int x = 510; x <=525; x++)
            //     {
            //         float g_score_value = g_score[this->getArrayIndexByCostmapCell(x, y)];
            //         if(g_score_value == std::numeric_limits<float>::infinity())
            //         {
            //             ROS_INFO("x: %i | y: %i | g_score: infinity", x, y);
            //         }
            //         else
            //         {
            //             ROS_INFO("x: %i | y: %i | g_score: %f", x, y, g_score_value);
            //         }
            //     }
            // }
            //DEBUGGING
            
            // this->createMarkersForGScoreArray(g_score);
            // this->visu_helper_.visualizeMarkerArray(this->g_score_marker_array_id_);
            // this->visu_helper_.clearMarkerArray(this->g_score_marker_array_id_);
            // ROS_INFO("open list, %i", array_open_cell_list.size());
        }

        // Reached goal, construct path now
        // Goal was not reached! Error handling
        if(g_score[array_goal_cell] == std::numeric_limits<float>::infinity())
        {
            ROS_ERROR("RelaxedAStar: No path found");
            ros::Duration(10.0).sleep();
            return mbf_msgs::GetPathResult::NO_PATH_FOUND;
        }

        int array_current_cell = array_goal_cell;
        std::vector<int> array_goal_to_start_plan;
        std::vector<int> array_start_to_goal_plan;

        // Construct path backwards from goal to start to get best path
        array_goal_to_start_plan.push_back(array_goal_cell);
        while(array_current_cell != array_start_cell)
        {
            int array_next_cell = this->getMinGScoreNeighborCell(array_current_cell, g_score);
            array_goal_to_start_plan.push_back(array_next_cell);
            array_current_cell = array_next_cell;
        }
        
        // <= is necessary as we go backwards through the list. And first elment from the back is found with size()-1 until we match the size, thats the first array.
        for(uint path_counter = 1; path_counter <= array_goal_to_start_plan.size(); path_counter++)
        {
            array_start_to_goal_plan.insert(array_start_to_goal_plan.begin() + array_start_to_goal_plan.size(),
                                            array_goal_to_start_plan[array_goal_to_start_plan.size() - path_counter]);
        }
        
        geometry_msgs::PoseArray planning_points_orientation;
        planning_points_orientation.header.stamp = ros::Time::now();
        planning_points_orientation.header.frame_id = "map";

        geometry_msgs::PoseStamped last_pose;
        // < is necessary because we just copy elements from one vector (0 until size()) to the other
        for(uint path_counter = 0; path_counter < array_start_to_goal_plan.size(); path_counter++)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = this->global_frame_;

            // Fill pose for plan
            int map_current_cell[2];
            this->getCostmapPointByArrayIndex(array_start_to_goal_plan[path_counter], map_current_cell);
            this->costmap_->mapToWorld(map_current_cell[0], map_current_cell[1], pose.pose.position.x, pose.pose.position.y);

            // Calculate orientation for each point of the plan with the current position and the last one
            if(path_counter == 0) // No previous point so orientation of start will be taken
            {
                pose.pose.orientation = start.pose.orientation;
            }
            else // Some other points are before, so orientation can be calculated
            {
                float delta_x = pose.pose.position.x - last_pose.pose.position.x;
                float delta_y = pose.pose.position.y - last_pose.pose.position.y;
                double yaw_angle = std::atan2(delta_y, delta_x);
                pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_angle);
            }

            last_pose = pose; // Safe pose for next iteration
            plan.insert(plan.begin() + plan.size(), pose);
            planning_points_orientation.poses.insert(planning_points_orientation.poses.begin() + planning_points_orientation.poses.size(), pose.pose);
        }

        this->planning_points_orientation_publisher_.publish(planning_points_orientation);
        ROS_INFO("Planning finished %i", plan.size());
        this->publishPlan(plan);
    }

    bool AdvancedAStar::cancel()
    {
        ROS_ERROR("AdvancedAStar cancel called");
    }

    std::vector<int> AdvancedAStar::getFreeNeighborCells(int array_current_cell)
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

                if (this->neighbor_type_ == general_types::NeighborType::EightWay ||
                    (this->neighbor_type_ == general_types::NeighborType::FourWay &&
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

    void AdvancedAStar::publishPlan(std::vector<geometry_msgs::PoseStamped> &plan)
    {
        nav_msgs::Path path_to_publish;
        path_to_publish.header.stamp = ros::Time::now();
        path_to_publish.header.frame_id = plan[0].header.frame_id;

        path_to_publish.poses = plan;

        this->plan_publisher_.publish(path_to_publish);
    }

    int AdvancedAStar::getMinGScoreNeighborCell(int current_cell_index, std::shared_ptr<float[]> g_score)
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

    float AdvancedAStar::calcMoveCost(int array_current_cell, int array_target_cell)
    {
        int map_current_cell[2];
        int map_target_cell[2];
        this->getCostmapPointByArrayIndex(array_current_cell, map_current_cell);
        this->getCostmapPointByArrayIndex(array_target_cell, map_target_cell);
        return this->calcMoveCost(map_current_cell, map_target_cell);
    }

    float AdvancedAStar::calcMoveCost(int* map_current_cell, int* map_target_cell)
    {
        return this->calcMoveCost(map_current_cell[0],
                                  map_current_cell[1],
                                  map_target_cell[0],
                                  map_target_cell[1]);
    }

    float AdvancedAStar::calcMoveCost(int map_current_cell_x, int map_current_cell_y, int map_target_cell_x, int map_target_cell_y)
    {
        return sqrt(pow(map_current_cell_x - map_target_cell_x, 2) + pow(map_current_cell_y - map_target_cell_y, 2));
    }

    float AdvancedAStar::calcGCost(float current_cell_g_cost, int array_current_cell, int array_target_cell)
    {
        return current_cell_g_cost + this->calcMoveCost(array_current_cell, array_target_cell);
    }

    float AdvancedAStar::calcHCost(int* map_selected_cell, int* map_goal_cell)
    {
        // Calc euclidean distance and return
        return std::sqrt(std::pow(map_selected_cell[0] - map_goal_cell[0], 2) +
                         std::pow(map_selected_cell[1] - map_goal_cell[1], 2));
    }

    float AdvancedAStar::calcHCost(int array_selected_cell, int array_goal_cell)
    {
        int map_selected_cell[2];
        int map_goal_cell[2];
        this->getCostmapPointByArrayIndex(array_selected_cell, map_selected_cell);
        this->getCostmapPointByArrayIndex(array_goal_cell, map_goal_cell);
        return this->calcHCost(map_selected_cell, map_goal_cell);
    }

    float AdvancedAStar::calcFCost(float current_cell_g_score, int array_current_cell, int array_target_cell, int array_goal_cell)
    {
        return this->calcGCost(current_cell_g_score, array_current_cell, array_target_cell) +
               this->calcHCost(array_target_cell, array_goal_cell);
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

    bool AdvancedAStar::isCellFree(int array_cell_index)
    {
        return !this->occupancy_map_[array_cell_index]; // Negate because function is "isFree" but occupancy_map_ defines with true where it is blocked
    }

    geometry_msgs::Pose AdvancedAStar::createGeometryPose(int array_cell)
    {
        geometry_msgs::Pose pose_to_return;
        geometry_msgs::Quaternion default_quaternion;
        tf::Quaternion default_tf_quaternion;
        default_tf_quaternion.setRPY(0.0, 0.0, 0.0);
        tf::quaternionTFToMsg(default_tf_quaternion, default_quaternion);
        pose_to_return.orientation = default_quaternion;
        int map_cell[2];
        this->getCostmapPointByArrayIndex(array_cell, map_cell);
        this->costmap_->mapToWorld(map_cell[0], map_cell[1], pose_to_return.position.x, pose_to_return.position.y);
        pose_to_return.position.z = 0.0;

        return pose_to_return;
    }

    void AdvancedAStar::createMarkersForGScoreArray(std::shared_ptr<float[]> g_score)
    {
        for(int counter = 0; counter < this->array_size_; counter++)
        {
            if(g_score[counter] != std::numeric_limits<float>::infinity())
            {
                geometry_msgs::Pose pose;
                geometry_msgs::Quaternion quaternion;
                tf::Quaternion tf_quaternion;
                tf_quaternion.setRPY(0.0, 0.0, 0.0);
                tf::quaternionTFToMsg(tf_quaternion, quaternion);
                pose.orientation = quaternion;
                int map_cell[2];
                this->getCostmapPointByArrayIndex(counter, map_cell);
                this->costmap_->mapToWorld(map_cell[0], map_cell[1], pose.position.x, pose.position.y);
                pose.position.z = 0.0;
                this->visu_helper_.addMarkerToExistingMarkerArray(this->g_score_marker_array_id_,
                                                                  pose,
                                                                  this->g_score_marker_template_id_);
            }
        }
    }

    void AdvancedAStar::createMarkersForOpenCellList(std::multiset<general_types::Cell, std::less<general_types::Cell>> array_open_cell_list)
    {
        for(std::multiset<general_types::Cell, std::less<general_types::Cell>>::const_iterator iterator = array_open_cell_list.begin();
            iterator != array_open_cell_list.end();
            ++iterator)
        {
            geometry_msgs::Pose pose;
            geometry_msgs::Quaternion quaternion;
            tf::Quaternion tf_quaternion;
            tf_quaternion.setRPY(0.0, 0.0, 0.0);
            tf::quaternionTFToMsg(tf_quaternion, quaternion);
            pose.orientation = quaternion;
            int map_cell[2];
            this->getCostmapPointByArrayIndex(iterator->cell_index, map_cell);
            this->costmap_->mapToWorld(map_cell[0], map_cell[1], pose.position.x, pose.position.y);
            pose.position.z = 0.0;
            this->visu_helper_.addMarkerToExistingMarkerArray(this->open_cell_marker_array_id_,
                                                              pose,
                                                              this->open_cell_marker_template_id_);
        }
    }
}