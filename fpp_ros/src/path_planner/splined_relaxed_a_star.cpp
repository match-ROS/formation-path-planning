#include <fpp_ros/path_planner/splined_relaxed_a_star.h>

namespace path_planner
{

    SplinedRelaxedAStar::SplinedRelaxedAStar()
        : initialized_(false)
    {
        ROS_ERROR("RELAXED A STAR DEFAULT CONSTRUCTOR");
    }

    SplinedRelaxedAStar::SplinedRelaxedAStar(std::string name, costmap_2d::Costmap2D *costmap,
                                             std::string global_frame)
        : initialized_(false), costmap_(costmap), global_frame_(global_frame)
    {
        ROS_ERROR("RELAXED A STAR OVERLOADED CONSTRUCTOR");
        this->initialize(name);
    }

    void SplinedRelaxedAStar::initialize(std::string name)
    {
        if(!this->initialized_)
        {
            ROS_INFO("Initializing SplinedRelaxedAStar planner.");
            this->array_size_ = this->costmap_->getSizeInCellsX() * this->costmap_->getSizeInCellsY();

            // Get parameter of planner
            ros::NodeHandle private_nh("~/" + name);

            this->plan_publisher_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
            this->planning_points_orientation_publisher_ = private_nh.advertise<geometry_msgs::PoseArray>("planning_points_orientation", 1);

            this->visu_helper_ = visualization_helper::VisualizationHelper(name);
            this->g_score_marker_identificator_ = "g_score";
            this->theoretical_path_marker_identificator_ = "theoretical_path";

            this->visu_helper_.addNewMarkerArray(this->g_score_marker_identificator_);
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
            this->visu_helper_.addMarkerTemplate(this->g_score_marker_identificator_, marker_template_g_score);
            
            this->visu_helper_.addNewMarkerArray(this->theoretical_path_marker_identificator_);
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
            this->visu_helper_.addMarkerTemplate(this->theoretical_path_marker_identificator_,
                                                 marker_template_theoretical_path);

            // Get the tf prefix
            ros::NodeHandle nh;
            this->tf_prefix_ = tf::getPrefixParam(nh);

            initialized_ = true; // Initialized method was called so planner is now initialized

            ROS_INFO("Relaxed AStar finished intitialization.");

            this->createOccupancyMap();

            //TESTING THIS WORKS FOR SOLVING LIN EQUATIONS
            // Eigen::Matrix<float, 4, 4> A;
            // A << 0,0,0,1,1,1,1,1,0,0,1,0,3,2,1,0;
            // Eigen::Matrix<float, 4, 1> b;
            // b << 5, 0, -1 , 1;
            // std::cout << "Matrix:\n" << A << std::endl;
            // std::cout << "Vector:\n" << b << std::endl;
            // Eigen::Matrix<float, 4, 1> result;
            // result = A.colPivHouseholderQr().solve(b);
            // std::cout << "Result:\n" << result << std::endl;
            // ros::Duration(100).sleep();
            //TESTING END
        }
        else
        {
            ROS_WARN("This planner has already been initialized");
        }
    }    

    bool SplinedRelaxedAStar::makePlan(const geometry_msgs::PoseStamped& start, 
                const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {
        double cost;
        std::string message;
        return 10 > this->makePlan(start, goal, this->default_tolerance_, plan, cost, message);
    }  

    uint32_t SplinedRelaxedAStar::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                double tolerance, std::vector<geometry_msgs::PoseStamped> &plan, double &cost,
                                std::string &message)
    {
        if(!initialized_) //Planner was not initialized. Abort
        {
            ROS_ERROR("SplinedRelaxedAStar planner was not initialized yet. Please initialize the planner before usage.");
            return mbf_msgs::GetPathResult::NOT_INITIALIZED;
        }

        ROS_ERROR("RELAXED A STAR MAKEPLAN");

        this->start_ = start;
        this->goal_ = goal;

        plan.clear(); // Clear path in case anything is already in it
        this->createOccupancyMap(); // Costmap can change so the occupancy map must be updated bevor making plan

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

        // Begin of splined_relaxed_a_star
        float fTieBreaker = 1 + (1/(this->costmap_->getSizeInCellsX() + this->costmap_->getSizeInCellsY()));

        // Initialization 
                
        // Create g_score array for the whole costmap and initialize with infinity so only visited cells get a value
        std::shared_ptr<float[]> g_score(new float[this->array_size_]);
        for(int counter = 0; counter < this->array_size_; counter++)
        {
            g_score[counter] = std::numeric_limits<float>::infinity();
        }
        g_score[array_start_cell] = 0;
		
		// Calculate the start straight line points
		tf::Vector3 start_straight_vector = tf::Vector3(this->start_straight_distance_, 0, 0);
		tf::Quaternion start_quaternion;
		tf::quaternionMsgToTF(start.pose.orientation, start_quaternion);
		start_straight_vector = tf::quatRotate(start_quaternion, start_straight_vector);
		tf::Vector3 world_start = tf::Vector3(world_start_pose_x, world_start_pose_y, 0);
		tf::Vector3 start_straight_end = world_start + start_straight_vector;
		unsigned int start_straight_end_cell_x, start_straight_end_cell_y;
		this->costmap_->worldToMap(start_straight_end.x(), start_straight_end.y(), start_straight_end_cell_x, start_straight_end_cell_y);
		int array_start_straight_end_cell = this->getArrayIndexByCostmapCell(start_straight_end_cell_x, start_straight_end_cell_y);
		
		// Calculate the end straight line points
		tf::Vector3 goal_straight_vector = tf::Vector3(this->end_straight_distance_, 0, 0);
		tf::Quaternion goal_quaternion;
		tf::quaternionMsgToTF(goal.pose.orientation, goal_quaternion);
		goal_straight_vector = tf::quatRotate(goal_quaternion, goal_straight_vector);
		tf::Vector3 world_goal = tf::Vector3(world_goal_pose_x, world_goal_pose_y, 0);
		tf::Vector3 goal_straight_end = world_goal - goal_straight_vector;
		unsigned int goal_straight_end_cell_x, goal_straight_end_cell_y;
		this->costmap_->worldToMap(goal_straight_end.x(), goal_straight_end.y(), goal_straight_end_cell_x, goal_straight_end_cell_y);
		int array_goal_straight_end_cell = this->getArrayIndexByCostmapCell(goal_straight_end_cell_x, goal_straight_end_cell_y);
		
		// Find plan for the start straight line
		this->findPlan(array_start_cell, array_start_straight_end_cell, g_score);

        // Start planning the middle part
        this->findPlan(array_start_straight_end_cell, array_goal_straight_end_cell, g_score);
		
		// Find Plan for the end straight line
		this->findPlan(array_goal_straight_end_cell, array_goal_cell, g_score);

        if (g_score[array_goal_cell] != std::numeric_limits<float>::infinity())
        {
            std::vector<int> array_plan;
            array_plan = this->createPlan(array_start_cell, array_goal_cell, g_score);
            this->createPoseArrayForPlan(array_plan, plan);

            // Select pose every so often
			int control_point_amount = std::ceil(float(plan.size()) / float(this->control_point_distance_));
			int real_control_point_distance = int(plan.size() / control_point_amount);

			std::vector<geometry_msgs::PoseStamped> selected_poses;
			
			// Use smaller or equal because when control_point_amount = 2 (plan.size = 90 && control_point_distance = 50), then there should be start, end and one control point in the middle
			for(int control_point_counter = 0; control_point_counter < control_point_amount; control_point_counter++)
			{
				selected_poses.push_back(plan[control_point_counter * real_control_point_distance]);
			}
            geometry_msgs::PoseStamped last;
            last.header = (plan.end() - 1)->header;
            last.pose = (plan.end() - 1)->pose;
            selected_poses.push_back(last);

			// Create splines
            std::vector<std::shared_ptr<path_planner::QuinticBezierSplines>> spline_list;
            for(int pose_counter = 0; pose_counter < (selected_poses.size()-1); pose_counter++)
            {
                Eigen::Matrix<float, 2, 1> start_pose;
                Eigen::Matrix<float, 2, 1> end_pose;
                start_pose << selected_poses[pose_counter].pose.position.x, selected_poses[pose_counter].pose.position.y;
                end_pose << selected_poses[pose_counter + 1].pose.position.x, selected_poses[pose_counter + 1].pose.position.y;
				std::shared_ptr<path_planner::QuinticBezierSplines> spline =
					std::make_shared<path_planner::QuinticBezierSplines>(&this->visu_helper_, start_pose, end_pose);
				
				if(spline_list.size() > 0)
				{
					spline_list.back()->setNextSpline(spline);
					spline->setPreviousSpline(spline_list.back());
				}
				spline_list.push_back(spline);
            }

			// Set start tangent of first spline
			tf::Quaternion start_quaternion;
            tf::quaternionMsgToTF(start.pose.orientation, start_quaternion);
            spline_list.front()->setStartTangent(start_quaternion);
			// Set end tangent of last spline
			tf::Quaternion end_quaternion;
			tf::quaternionMsgToTF(goal.pose.orientation, end_quaternion);
			spline_list.back()->setEndTangent(end_quaternion);
			
			for(std::shared_ptr<QuinticBezierSplines> &spline: spline_list)
			{
				spline->calcControlPoints();
				spline->addStartEndPointToVisuHelper();
                spline->addTangentsToVisuHelper();
                spline->addControlPointsToVisuHelper();
                spline->addBezierSplineToVisuHelper(this->planning_points_per_spline_);
				// spline->visualizeData();
			}

			// Optimize the curvature of the splines to be under a certain threshold
			ROS_INFO_STREAM("Optimizing the curvature of the splines");
			for(int spline_counter = 0; spline_counter < spline_list.size(); spline_counter++)
			{
				ROS_INFO_STREAM("spline counter: " << spline_counter);
				ROS_INFO_STREAM("valid: " << spline_list[spline_counter]->checkMinCurveRadiusOnSpline(this->planning_points_per_spline_, this->minimal_curve_radius_));

				int timeout_counter = 0;

				int point_of_failure = 0;
				while(!spline_list[spline_counter]->checkMinCurveRadiusOnSpline(this->planning_points_per_spline_, this->minimal_curve_radius_, point_of_failure))
				{
					// This process could be turned into an iterative optimization process, that tries to get to the limit of the minimal curve radius to minimize the distance the robots have to travel
					if(point_of_failure < (this->planning_points_per_spline_ / 2))
					{
						float current_start_magnitude = spline_list[spline_counter]->getStartTangentMagnitude();
						ROS_INFO_STREAM("Current_start_magnitude: " << current_start_magnitude);
						spline_list[spline_counter]->setStartTangentMagnitude(1.5 * current_start_magnitude); // 1.5 ist just a value that I picked for the moment.
					}
					else
					{
						float current_end_magnitude = spline_list[spline_counter]->getEndTangentMagnitude();
						ROS_INFO_STREAM("Current_end_magnitude: " << current_end_magnitude);
						spline_list[spline_counter]->setEndTangentMagnitude(1.5 * current_end_magnitude); // 1.5 ist just a value that I picked for the moment.
					}

					spline_list[spline_counter]->calcControlPoints();

					// Loop was not able to optimize the spline after x iterations. Maybe make parameter for this?
					if(timeout_counter >= 100)
					{
						ROS_ERROR_STREAM("SplinedRelaxedAStar: Timeout while optimizing spline, number: " << spline_counter);
						break;
					}
					timeout_counter++;
				}

				ROS_INFO_STREAM("spline counter: " << spline_counter);
				ROS_INFO_STREAM("valid: " << spline_list[spline_counter]->checkMinCurveRadiusOnSpline(this->planning_points_per_spline_, this->minimal_curve_radius_));
			}

			// Visualization
			for(std::shared_ptr<QuinticBezierSplines> &spline: spline_list)
			{
				spline->addStartEndPointToVisuHelper();
                spline->addTangentsToVisuHelper();
                spline->addControlPointsToVisuHelper();
                spline->addBezierSplineToVisuHelper(this->planning_points_per_spline_);
				// spline->visualizeData();
			}

			// Create plan by splines
            plan.clear();
            std::vector<Eigen::Matrix<float, 2, 1>> points_of_plan;
            for(std::shared_ptr<QuinticBezierSplines> &spline: spline_list)
            {
                std::vector<Eigen::Matrix<float, 2, 1>> points;
                points = spline->calcBezierSpline(this->planning_points_per_spline_);
                points_of_plan.insert(points_of_plan.end(), points.begin(), points.end());
            }
            geometry_msgs::PoseStamped last_pose;
            // < is necessary because we just copy elements from one vector (0 until size()) to the other
            for(uint path_counter = 0; path_counter < points_of_plan.size(); path_counter++)
            {
                geometry_msgs::PoseStamped pose;
                pose.header.stamp = ros::Time::now();
                pose.header.frame_id = this->global_frame_;

                pose.pose.position.x = points_of_plan[path_counter][0];
                pose.pose.position.y = points_of_plan[path_counter][1];

                // Calculate orientation for each point of the plan with the current position and the last one
                if(path_counter == 0) // No previous point so orientation of start will be taken
                {
                    pose.pose.orientation = this->start_.pose.orientation;
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
            }
        }
        ROS_INFO("Planning finished %i", plan.size());
        this->publishPlan(plan);
        return 0;
    }

    bool SplinedRelaxedAStar::cancel()
    {
        ROS_ERROR("RELAXED A STAR CANCEL");
        return false; // Returns false if not implemented. Returns true if implemented and cancel has been successful.
    }

    void SplinedRelaxedAStar::findPlan(int array_start_cell, int array_goal_cell, std::shared_ptr<float[]> g_score)
    {
        // The array_open_cell_list list contains all the open cells that were neighbors but not explored.
        // The elements in this list are linking to the index of the one dimensional costmap representation array.
        std::multiset<fpp_data_classes::Cell, std::less<fpp_data_classes::Cell>> array_open_cell_list;
        array_open_cell_list.insert({array_start_cell, this->calcHCost(array_start_cell, array_goal_cell)});

		// g_score[array_start_cell] = 0.0;
		// g_score[array_goal_cell] = std::numeric_limits<float>::infinity();

		// ROS_INFO_STREAM("Start cell: " << array_start_cell << ", goal cell: " << array_goal_cell);

        while (!array_open_cell_list.empty() &&
               g_score[array_goal_cell] == std::numeric_limits<float>::infinity())
        {
            // ROS_INFO("Open_Cell_Count: %i", array_open_cell_list.size());
            int array_current_cell = array_open_cell_list.begin()->cell_index; // Get cell with lowest f_score
            // ROS_INFO("cell0: %f, cell1: %f, last: %f", array_open_cell_list.begin()->f_cost, std::next(array_open_cell_list.begin())->f_cost, std::prev(array_open_cell_list.end())->f_cost);
            array_open_cell_list.erase(array_open_cell_list.begin()); // Remove cell from open_cell_set so it will not be visited again
            
            std::vector<int> array_neighbor_cell_list = this->getFreeNeighborCells(array_current_cell);
            
            for(int array_neighbor_cell: array_neighbor_cell_list)
            {
                if (g_score[array_neighbor_cell] == std::numeric_limits<float>::infinity())
                {
                    g_score[array_neighbor_cell] = this->calcGCost(g_score[array_current_cell], array_current_cell, array_neighbor_cell);
                    array_open_cell_list.insert({array_neighbor_cell,
                                                 this->calcFCost(g_score[array_current_cell],
                                                                 array_current_cell,
                                                                 array_neighbor_cell,
                                                                 array_goal_cell)});
                }
            }
            // this->createMarkersForGScoreArray(g_score);
            // this->visu_helper_.visualizeMarkerArray(this->g_score_marker_identificator_);
            // this->visu_helper_.clearMarkerArray(this->g_score_marker_identificator_);
            // ros::Duration(0.1).sleep();
        }
    }

    std::vector<int> SplinedRelaxedAStar::createPlan(int array_start_cell, int array_goal_cell, std::shared_ptr<float[]> g_score)
    {
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
        return array_start_to_goal_plan;
    }

    void SplinedRelaxedAStar::createPoseArrayForPlan(std::vector<int> array_plan, std::vector<geometry_msgs::PoseStamped>& plan)
    {
        geometry_msgs::PoseStamped last_pose;
        // < is necessary because we just copy elements from one vector (0 until size()) to the other
        for(uint path_counter = 0; path_counter < array_plan.size(); path_counter++)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = this->global_frame_;

            // Fill pose for plan
            int map_current_cell[2];
            this->getCostmapPointByArrayIndex(array_plan[path_counter], map_current_cell);
            this->costmap_->mapToWorld(map_current_cell[0], map_current_cell[1], pose.pose.position.x, pose.pose.position.y);

            // Calculate orientation for each point of the plan with the current position and the last one
            if(path_counter == 0) // No previous point so orientation of start will be taken
            {
                pose.pose.orientation = this->start_.pose.orientation;
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
        }
    }

    std::vector<int> SplinedRelaxedAStar::getFreeNeighborCells(int array_current_cell)
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

                if (this->neighbor_type_ == path_planner::NeighborType::EightWay ||
                    (this->neighbor_type_ == path_planner::NeighborType::FourWay &&
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

    int SplinedRelaxedAStar::getMinGScoreNeighborCell(int current_cell_index, std::shared_ptr<float[]> g_score)
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

    void SplinedRelaxedAStar::createOccupancyMap()
    {
        this->occupancy_map_ = std::shared_ptr<bool[]>(new bool[this->array_size_]);

        // Check every cell of the costmap and if 
        for(int cell_counter_x = 0; cell_counter_x < this->costmap_->getSizeInCellsX(); cell_counter_x++)
        {
            for(int cell_counter_y = 0; cell_counter_y < this->costmap_->getSizeInCellsY(); cell_counter_y++)
            {
                unsigned int cell_cost = static_cast<unsigned int>(this->costmap_->getCost(cell_counter_x, cell_counter_y));

                if(cell_cost <= this->free_cell_threshhold_) // Cell is free
                {
                    this->occupancy_map_[this->getArrayIndexByCostmapCell(cell_counter_x, cell_counter_y)] = false; // False because cell is not occupied
                }
                else
                {
                    this->occupancy_map_[this->getArrayIndexByCostmapCell(cell_counter_x, cell_counter_y)] = true;  // True because cell is occupied
                }
            }
        }
    }

    void SplinedRelaxedAStar::publishPlan(std::vector<geometry_msgs::PoseStamped> &plan)
    {
        nav_msgs::Path path_to_publish;
        path_to_publish.header.stamp = ros::Time::now();
        path_to_publish.header.frame_id = plan[0].header.frame_id;

        path_to_publish.poses = plan;

        this->plan_publisher_.publish(path_to_publish);
    }

    float SplinedRelaxedAStar::calcMoveCost(int array_current_cell, int array_target_cell)
    {
        int map_current_cell[2];
        int map_target_cell[2];
        this->getCostmapPointByArrayIndex(array_current_cell, map_current_cell);
        this->getCostmapPointByArrayIndex(array_target_cell, map_target_cell);
        return this->calcMoveCost(map_current_cell, map_target_cell);
    }

    float SplinedRelaxedAStar::calcMoveCost(int* map_current_cell, int* map_target_cell)
    {
        return this->calcMoveCost(map_current_cell[0],
                                  map_current_cell[1],
                                  map_target_cell[0],
                                  map_target_cell[1]);
    }

    float SplinedRelaxedAStar::calcMoveCost(int map_current_cell_x, int map_current_cell_y, int map_target_cell_x, int map_target_cell_y)
    {
        return sqrt(pow(map_current_cell_x - map_target_cell_x, 2) + pow(map_current_cell_y - map_target_cell_y, 2));
    }

    float SplinedRelaxedAStar::calcGCost(float current_cell_g_cost, int array_current_cell, int array_target_cell)
    {
        return current_cell_g_cost + this->calcMoveCost(array_current_cell, array_target_cell);
    }

    float SplinedRelaxedAStar::calcHCost(int* map_selected_cell, int* map_goal_cell)
    {
        // Calc euclidean distance and return
        return std::sqrt(std::pow(map_selected_cell[0] - map_goal_cell[0], 2) +
                         std::pow(map_selected_cell[1] - map_goal_cell[1], 2));
    }

    float SplinedRelaxedAStar::calcHCost(int array_selected_cell, int array_goal_cell)
    {
        int map_selected_cell[2];
        int map_goal_cell[2];
        this->getCostmapPointByArrayIndex(array_selected_cell, map_selected_cell);
        this->getCostmapPointByArrayIndex(array_goal_cell, map_goal_cell);
        return this->calcHCost(map_selected_cell, map_goal_cell);
    }

    float SplinedRelaxedAStar::calcFCost(float current_cell_g_score, int array_current_cell, int array_target_cell, int array_goal_cell)
    {
        return this->calcGCost(current_cell_g_score, array_current_cell, array_target_cell) +
               this->calcHCost(array_target_cell, array_goal_cell);
    }

    int SplinedRelaxedAStar::getArrayIndexByCostmapCell(int *map_cell)
    {
        return map_cell[1] * this->costmap_->getSizeInCellsX() + map_cell[0];
    }

    int SplinedRelaxedAStar::getArrayIndexByCostmapCell(int map_cell_x, int map_cell_y)
    {
        int map_cell[2] = {map_cell_x, map_cell_y};
        return this->getArrayIndexByCostmapCell(map_cell);
    }

    void SplinedRelaxedAStar::getCostmapPointByArrayIndex(int array_index, int *map_cell)
    {
        map_cell[1] = array_index / this->costmap_->getSizeInCellsX();
        map_cell[0] = array_index % this->costmap_->getSizeInCellsX();
    }

    void SplinedRelaxedAStar::getCostmapPointByArrayIndex(int array_index, int &map_cell_x, int &map_cell_y)
    {
        int map_cell[2];
        this->getCostmapPointByArrayIndex(array_index, map_cell);
        map_cell_x = map_cell[0];
        map_cell_y = map_cell[1];
    }

    bool SplinedRelaxedAStar::isCellFree(int array_cell_index)
    {
        return !this->occupancy_map_[array_cell_index]; // Negate because function is "isFree" but occupancy_map_ defines with true where it is blocked
    }

    geometry_msgs::Pose SplinedRelaxedAStar::createGeometryPose(int array_cell)
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

    void SplinedRelaxedAStar::createMarkersForGScoreArray(std::shared_ptr<float[]> g_score)
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
                this->visu_helper_.addMarkerToExistingMarkerArray(this->g_score_marker_identificator_,
                                                                  pose,
                                                                  this->g_score_marker_identificator_);
            }
        }
    }

    geometry_msgs::PoseArray SplinedRelaxedAStar::createPoseArrayForOrientationVisu(std::vector<geometry_msgs::PoseStamped>& plan)
    {
        geometry_msgs::PoseArray planning_points_orientation;
        planning_points_orientation.header.stamp = ros::Time::now();
        planning_points_orientation.header.frame_id = "map";

        for(geometry_msgs::PoseStamped pose_stamped: plan)
        {
            planning_points_orientation.poses.insert(planning_points_orientation.poses.begin() + planning_points_orientation.poses.size(),
                                                     pose_stamped.pose);
        }
        return planning_points_orientation;
    }

    void SplinedRelaxedAStar::setDefaultTolerance(float default_tolerance)
    {   
        this->default_tolerance_ = default_tolerance;
    }

    void SplinedRelaxedAStar::setNeighborType(int neighbor_type)
    {
        this->neighbor_type_ = (NeighborType)neighbor_type;
    }

    void SplinedRelaxedAStar::setFreeCellThreshhold(int free_cell_thresshold)
    {
        this->free_cell_threshhold_ = free_cell_thresshold;
    }

	void SplinedRelaxedAStar::setStartStraightDistance(float start_straight_distance)
	{
		this->start_straight_distance_ = start_straight_distance;
	}

	void SplinedRelaxedAStar::setEndStraightDistance(float end_straight_distance)
	{
		this->end_straight_distance_ = end_straight_distance;
	}

	void SplinedRelaxedAStar::setControlPointDistance(int control_point_distance)
	{
		this->control_point_distance_ = control_point_distance;
	}

	void SplinedRelaxedAStar::setMinimalCurveRadius(float minimal_curve_radius)
	{
		this->minimal_curve_radius_ = minimal_curve_radius;
	}

	void SplinedRelaxedAStar::setPlanningPointsPerSpline(int planning_points_per_spline)
	{
		this->planning_points_per_spline_ = planning_points_per_spline;
	}
}