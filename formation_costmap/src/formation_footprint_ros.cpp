#include <formation_costmap/formation_footprint_ros.h>

namespace formation_costmap
{
	FormationFootprintRos::FormationFootprintRos()
		: GeometryContour()
	{ }

    FormationFootprintRos::FormationFootprintRos(Eigen::Vector2f lead_vector_world_cs, float world_to_geometry_cs_rotation)
        : GeometryContour(lead_vector_world_cs, world_to_geometry_cs_rotation)
    {

    }

    void FormationFootprintRos::exeGiftWrappingAlg()
    {
        if(this->corner_points_geometry_cs_.size() < 2)
        {
            std::cout << "FormationFootprintRos::exeGiftWrappingAlg: ERROR, less than two points in contour.\n";
            return; // Error. With only 2 corners there can not be an area between the points
        }
        
        // Start with finding the most left poit of the point cloud
        int index_lowest_point = 0;
        Eigen::Vector2f lowest_point_geometry_cs = this->corner_points_geometry_cs_[0]; // Initialize with first point from list
        for(int corner_counter = 0; corner_counter < this->corner_points_geometry_cs_.size(); corner_counter++)
        {
            if(this->corner_points_geometry_cs_[corner_counter][1] < lowest_point_geometry_cs[1])
            {
                lowest_point_geometry_cs = this->corner_points_geometry_cs_[corner_counter];
                index_lowest_point = corner_counter;
            }
        }

        std::vector<Eigen::Vector2f> corners_to_wrap_geometry_cs = this->corner_points_geometry_cs_;
        std::vector<Eigen::Vector2f> wrapped_contour_corners_geometry_cs;
        wrapped_contour_corners_geometry_cs.push_back(lowest_point_geometry_cs);

        float previous_radian = 2*M_PI; // Init with pi because no point can be below this one. After this point we rotate the line clockwise to find the next point
        // Get next point as long as the start point was not reached again
        while(wrapped_contour_corners_geometry_cs.size() <= 1 || wrapped_contour_corners_geometry_cs.back() != lowest_point_geometry_cs)
        {
            int next_wrapping_point_index;
            Eigen::Vector2f next_wrapping_point_geometry_cs;
            
            next_wrapping_point_index = this->calcNextWrappingPoint(wrapped_contour_corners_geometry_cs.back(),
                                                                    corners_to_wrap_geometry_cs,
                                                                    next_wrapping_point_geometry_cs,
                                                                    previous_radian);
            wrapped_contour_corners_geometry_cs.push_back(next_wrapping_point_geometry_cs);
            corners_to_wrap_geometry_cs.erase(corners_to_wrap_geometry_cs.begin() + next_wrapping_point_index);       
        }

        if(wrapped_contour_corners_geometry_cs.front() == wrapped_contour_corners_geometry_cs.back())
        {
            wrapped_contour_corners_geometry_cs.pop_back();
        }
        this->corner_points_geometry_cs_ = wrapped_contour_corners_geometry_cs;
        this->createContourEdges();
    }
	
    void FormationFootprintRos::addRobotToFormation(std::shared_ptr<RobotFootprintRos> robot_to_add)
    {
		auto it = std::find_if(this->robot_contours_.begin(),
							   this->robot_contours_.end(),
							   [&robot_to_add](std::shared_ptr<RobotFootprintRos> &robot_contour) { return robot_contour->getRobotName() == robot_to_add->getRobotName(); });
		if (it == this->robot_contours_.end())
		{
			this->robot_position_updates_.insert(std::pair<std::string, bool>(robot_to_add->getRobotName(),
																			  false));
			robot_to_add->setRobotPoseChangedEventHandler(
				std::bind(&FormationFootprintRos::robotPositionChanged, this, std::placeholders::_1));
            this->robot_contours_.push_back(robot_to_add);
        }
        else
        {
            std::cout << "FormationFootprintRos: Robot already exists in formation.";
        }
    }

    void FormationFootprintRos::updateRobotPose(std::string robot_name,
                                           Eigen::Vector2f new_robot_pose_world_cs,
                                           float new_rotation_world_to_geometry_cs)
    {
        for(std::shared_ptr<RobotFootprintRos> &robot_contour : this->robot_contours_)
        {
            if(robot_contour->getRobotName() == robot_name)
            {
                robot_contour->moveContour(new_robot_pose_world_cs, new_rotation_world_to_geometry_cs);
            }
        }
    }

    void FormationFootprintRos::updateFormationContour()
    {
		ROS_ERROR_STREAM("FormationFootprintRos::updateFormationContour");
        this->corner_points_geometry_cs_.clear(); // Delete all current corner points

        // Add all corners of current robots and then exe gift wrapping algorithm
        for(std::shared_ptr<RobotFootprintRos> &robot_contour: this->robot_contours_)
        {
			// ROS_INFO_STREAM(robot_contour->getCornerPointsWorldCS()[0]);
			// ROS_INFO_STREAM(robot_contour->getCornerPointsWorldCS()[1]);
			// ROS_INFO_STREAM(robot_contour->getCornerPointsWorldCS()[2]);
			// ROS_INFO_STREAM(robot_contour->getCornerPointsWorldCS()[3]);
			this->addContourCornersWorldCS(robot_contour->getCornerPointsWorldCS());	
        }
        this->exeGiftWrappingAlg();
    }

	void FormationFootprintRos::moveCoordinateSystem(Eigen::Vector2f new_lead_vector_world_cs, float new_cs_rotation)
	{
		Eigen::Matrix<float, 3, 3> new_tf_geometry_to_world_cs = this->createTransformationMatrix(new_lead_vector_world_cs,
                                                                                       new_cs_rotation);
        Eigen::Matrix<float, 3, 3> new_tf_world_to_geometry_cs = new_tf_geometry_to_world_cs.inverse();

		// Robot positions were intialized with world coords.
		// Then coordinate system of formation was places in centre of gravity of the formation contour
		// Robot positions will now be moved into the geometry cs of the formation
		// for(std::shared_ptr<RobotFootprintRos> &robot_contour: this->robot_contours_)
		// {
		// 	Eigen::Vector2f old_robot_lead_vector_world_cs = robot_contour->getLeadVectorWorldCS();
		// 	Eigen::Vector3f old_robot_lead_vector_world_cs_extended;
		// 	old_robot_lead_vector_world_cs_extended << old_robot_lead_vector_world_cs, 1;
		// 	Eigen::Vector3f new_robot_lead_vector_world_cs_extended = new_tf_world_to_geometry_cs * old_robot_lead_vector_world_cs_extended;
		// 	Eigen::Vector2f new_robot_lead_vector_world_cs = new_robot_lead_vector_world_cs_extended.head(2);
		// 	robot_contour->moveCoordinateSystem(new_robot_lead_vector_world_cs, robot_contour->getWorldToGeometryCSRotation());
		// }

        this->lead_vector_world_cs_ = new_lead_vector_world_cs;
        this->world_to_geometry_cs_rotation_ = new_cs_rotation;
        this->tf_geometry_to_world_cs_ = new_tf_geometry_to_world_cs;
        this->tf_world_to_geometry_cs_ = new_tf_world_to_geometry_cs;

        this->corner_points_geometry_cs_.clear();
		this->edge_list_geometry_cs_.clear();

		this->updateFormationContour();
		this->createContourEdges();
	}

	#pragma region Getter/Setter
	Eigen::Vector2f FormationFootprintRos::getRobotPosGeometryCS(std::string robot_name)
	{
		int robot_counter;
		for(robot_counter = 0; robot_counter < this->robot_contours_.size(); robot_counter++)
		{
			if(this->robot_contours_[robot_counter]->getRobotName() == robot_name)
			{
				break;
			}
		}

		return this->robot_contours_[robot_counter]->getLeadVectorWorldCS();
	}

	Eigen::Vector2f FormationFootprintRos::getRobotPosWorldCS(std::string robot_name)
	{
		int robot_counter;
		for(robot_counter = 0; robot_counter < this->robot_contours_.size(); robot_counter++)
		{
			if(this->robot_contours_[robot_counter]->getRobotName() == robot_name)
			{
				break;
			}
		}

		// As the robot contour lives in the formation contour this must be transformed before being in the world cs
		return this->transformGeometryToWorldCS(this->robot_contours_[robot_counter]->getLeadVectorWorldCS());
	}
	
   	std::vector<std::shared_ptr<RobotFootprintRos>> FormationFootprintRos::getRobotContours()
	{
		return this->robot_contours_;
	}

	std::shared_ptr<RobotFootprintRos> FormationFootprintRos::getRobotContour(std::string robot_name)
	{
		for(std::shared_ptr<RobotFootprintRos> &robot_contour: this->robot_contours_)
		{
			if(robot_contour->getRobotName() == robot_name)
			{
				return robot_contour;
			}
		}

		ROS_ERROR_STREAM("FormationFootprintRos::getRobotContour: No robot name matched the robots in the formation.");
	}

	geometry_msgs::PolygonStamped FormationFootprintRos::getFormationFootprint()
	{
		geometry_msgs::PolygonStamped formation_footprint_msg;
        formation_footprint_msg.header.frame_id = "map";
        formation_footprint_msg.header.stamp = ros::Time::now();

        std::vector<Eigen::Vector2f> formation_corner_points = this->getCornerPointsWorldCS();

        for(Eigen::Vector2f corner: formation_corner_points)
        {
            geometry_msgs::Point32 corner_point;
            corner_point.x = corner[0];
            corner_point.y = corner[1];
            corner_point.z = 0.0;
            formation_footprint_msg.polygon.points.push_back(corner_point);
        }

		return formation_footprint_msg;
	}
	#pragma endregion

	#pragma region EventCallbacks
	void FormationFootprintRos::robotPositionChanged(std::string robot_name)
	{
		ROS_INFO_STREAM("Update " << robot_name);
		this->robot_position_updates_[robot_name] = true;

		// if(this->allPositionUpdatesReceived())
		// {
		this->updateFormationContour();

		Eigen::Vector2f new_centroid_world_cs;
		new_centroid_world_cs = this->calcCentroidWorldCS();
		this->moveCoordinateSystem(new_centroid_world_cs, this->world_to_geometry_cs_rotation_);

		// this->resetPositionUpdateTable();
		// }
	}
	#pragma endregion

	#pragma region Private Helper Methods
	int FormationFootprintRos::calcNextWrappingPoint(Eigen::Vector2f current_wrapping_point_geometry_cs,
                                                std::vector<Eigen::Vector2f> corners_to_wrap_geometry_cs,
                                                Eigen::Vector2f &next_wrapping_point_geometry_cs,
                                                float &max_radian)
    {
        int next_wrapping_point_index = 0;
        float next_max_radian;
        float smallest_diff_to_max_radian = 2*M_PI; // This is the max value and should never occure because I look at the diff

        for(int corner_to_wrap_counter = 0; corner_to_wrap_counter < corners_to_wrap_geometry_cs.size(); corner_to_wrap_counter++)
        {
            if(current_wrapping_point_geometry_cs == corners_to_wrap_geometry_cs[corner_to_wrap_counter])
            {
                continue;
            }
            float radian_next_wrapping_point = this->calcTan2(current_wrapping_point_geometry_cs,
                                                              corners_to_wrap_geometry_cs[corner_to_wrap_counter]);
            radian_next_wrapping_point = radian_next_wrapping_point + M_PI; // So I dont get a value area from -pi to pi.
            float next_diff = max_radian - radian_next_wrapping_point;

            if(radian_next_wrapping_point <= max_radian && smallest_diff_to_max_radian > next_diff)
            {
                smallest_diff_to_max_radian = next_diff;
                next_wrapping_point_index = corner_to_wrap_counter;
                next_max_radian = radian_next_wrapping_point;
                next_wrapping_point_geometry_cs = corners_to_wrap_geometry_cs[corner_to_wrap_counter];
            }                                                              
        }
        max_radian = next_max_radian;
        return next_wrapping_point_index;
    }

    float FormationFootprintRos::calcTan2(Eigen::Vector2f start_point_geometry_cs, Eigen::Vector2f end_point_geometry_cs)
    {
        return std::atan2(end_point_geometry_cs[1] - start_point_geometry_cs[1],
                          end_point_geometry_cs[0] - start_point_geometry_cs[0]);
    }

	bool FormationFootprintRos::allPositionUpdatesReceived()
	{
		for(const std::pair<std::string, bool> &robot_pose_update: this->robot_position_updates_)
		{
			if(!robot_pose_update.second)
			{
				return false;
			}
		}

		return true;
	}

	void FormationFootprintRos::resetPositionUpdateTable()
	{
		for(const std::pair<std::string, bool> &robot_pose_update: this->robot_position_updates_)
		{
			this->robot_position_updates_[robot_pose_update.first] = false;
		}
	}
	#pragma endregion
}