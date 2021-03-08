#include <fp_utils/geometry_info/formation_contour.h>

namespace geometry_info
{
	FormationContour::FormationContour()
		: GeometryContour()
	{ }

    FormationContour::FormationContour(Eigen::Vector2f lead_vector_world_cs, float world_to_geometry_cs_rotation)
        : GeometryContour(lead_vector_world_cs, world_to_geometry_cs_rotation)
    { }

    void FormationContour::exeGiftWrappingAlg()
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
	
    bool FormationContour::addRobotToFormation(std::shared_ptr<RobotContour> robot_to_add)
    {
		auto it = std::find_if(this->robot_contours_.begin(),
							   this->robot_contours_.end(),
							   [&robot_to_add](std::shared_ptr<RobotContour> &robot_contour) { return robot_contour->getRobotName() == robot_to_add->getRobotName(); });
		if (it == this->robot_contours_.end())
		{
			robot_to_add->setRobotPoseChangedEventHandler(
				std::bind(&FormationContour::robotPositionChanged, this, std::placeholders::_1));
            this->robot_contours_.push_back(robot_to_add);

			return true;
        }
        else
        {
            std::cout << "FormationFootprintRos: Robot already exists in formation.";

			return false;
        }
    }

    void FormationContour::updateRobotPose(std::string robot_name,
                                           Eigen::Vector2f new_robot_pose_world_cs,
                                           float new_rotation_world_to_geometry_cs)
    {
        for(std::shared_ptr<RobotContour> &robot_contour : this->robot_contours_)
        {
            if(robot_contour->getRobotName() == robot_name)
            {
                robot_contour->moveContour(new_robot_pose_world_cs, new_rotation_world_to_geometry_cs);
            }
        }
    }

    void FormationContour::updateFormationContour()
    {
        this->corner_points_geometry_cs_.clear(); // Delete all current corner points

        // Add all corners of current robots and then exe gift wrapping algorithm
        for(std::shared_ptr<RobotContour> &robot_contour: this->robot_contours_)
        {
			this->addContourCornersWorldCS(robot_contour->getCornerPointsWorldCS());	
        }
        this->exeGiftWrappingAlg();
    }

	void FormationContour::moveCoordinateSystem(Eigen::Vector2f new_lead_vector_world_cs, float new_cs_rotation)
	{
		Eigen::Matrix<float, 3, 3> new_tf_geometry_to_world_cs = this->createTransformationMatrix(new_lead_vector_world_cs,
                                                                                       new_cs_rotation);
        Eigen::Matrix<float, 3, 3> new_tf_world_to_geometry_cs = new_tf_geometry_to_world_cs.inverse();

        this->lead_vector_world_cs_ = new_lead_vector_world_cs;
        this->world_to_geometry_cs_rotation_ = new_cs_rotation;
        this->tf_geometry_to_world_cs_ = new_tf_geometry_to_world_cs;
        this->tf_world_to_geometry_cs_ = new_tf_world_to_geometry_cs;

        this->corner_points_geometry_cs_.clear();
		this->edge_list_geometry_cs_.clear();

		this->updateFormationContour();
		this->createContourEdges();
	}

	float FormationContour::calcMinimalEnclosingCircleRadius()
	{
		geometry_info::MinimalEnclosingCircle minimal_circle = geometry_info::MinimalEnclosingCircle();
		minimal_circle.calcMinimalEnclosingCircle(this->calcCentroidWorldCS(),
												  this->getCornerPointsWorldCS());
		return minimal_circle.getCircleRadius();
	}

	#pragma region Getter/Setter
	Eigen::Vector2f FormationContour::getRobotPosGeometryCS(std::string robot_name)
	{
		int robot_counter;
		for(robot_counter = 0; robot_counter < this->robot_contours_.size(); robot_counter++)
		{
			if(this->robot_contours_[robot_counter]->getRobotName() == robot_name)
			{
				break;
			}
		}

		Eigen::Vector2f robot_lead_vector_world_cs;
		robot_lead_vector_world_cs = this->robot_contours_[robot_counter]->getLeadVectorWorldCS();
		Eigen::Vector2f robot_lead_vector_geometry_cs;
		robot_lead_vector_geometry_cs = this->transformWorldToGeometryCS(robot_lead_vector_world_cs);

		return robot_lead_vector_geometry_cs;
	}

	Eigen::Vector2f FormationContour::getRobotPosWorldCS(std::string robot_name)
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
	
   	std::vector<std::shared_ptr<RobotContour>> FormationContour::getRobotContours()
	{
		return this->robot_contours_;
	}

	std::shared_ptr<RobotContour> FormationContour::getRobotContour(std::string robot_name)
	{
		for(std::shared_ptr<RobotContour> &robot_contour: this->robot_contours_)
		{
			if(robot_contour->getRobotName() == robot_name)
			{
				return robot_contour;
			}
		}

		std::cout << "FormationContour::getRobotContour: No robot name matched the robots in the formation.";
	}
	#pragma endregion

	#pragma region EventCallbacks
	void FormationContour::robotPositionChanged(std::string robot_name)
	{
		this->updateFormationContour();

		Eigen::Vector2f new_centroid_world_cs;
		new_centroid_world_cs = this->calcCentroidWorldCS();
		this->moveCoordinateSystem(new_centroid_world_cs, this->world_to_geometry_cs_rotation_);
	}
	#pragma endregion

	#pragma region Private Helper Methods
	int FormationContour::calcNextWrappingPoint(Eigen::Vector2f current_wrapping_point_geometry_cs,
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

    float FormationContour::calcTan2(Eigen::Vector2f start_point_geometry_cs, Eigen::Vector2f end_point_geometry_cs)
    {
        return std::atan2(end_point_geometry_cs[1] - start_point_geometry_cs[1],
                          end_point_geometry_cs[0] - start_point_geometry_cs[0]);
    }
	#pragma endregion
}