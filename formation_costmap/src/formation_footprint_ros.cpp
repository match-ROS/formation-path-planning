#include <formation_costmap/formation_footprint_ros.h>

namespace formation_costmap
{
	FormationFootprintRos::FormationFootprintRos()
		: FormationContour()
	{ }

	FormationFootprintRos::FormationFootprintRos(Eigen::Vector2f lead_vector_world_cs,
												 float world_to_geometry_cs_rotation)
		: FormationContour(lead_vector_world_cs, world_to_geometry_cs_rotation)
	{
	}

	bool FormationFootprintRos::addRobotToFormation(std::shared_ptr<geometry_info::RobotContour> robot_to_add)
    {
		bool add_robot_result = FormationContour::addRobotToFormation(robot_to_add);

		if(add_robot_result)
		{
			this->robot_position_updates_.insert(std::pair<std::string, bool>(robot_to_add->getRobotName(),
																			  false));
			this->updateFormationContour();
		}

		return add_robot_result;
    }

	#pragma region Getter/Setter
	std::shared_ptr<RobotFootprintRos> FormationFootprintRos::getRobotContour(std::string robot_name)
	{
		for(int robot_counter = 0; robot_counter < this->robot_contours_.size(); robot_counter++)
		{
			if(this->robot_contours_[robot_counter]->getRobotName() == robot_name)
			{
				return std::dynamic_pointer_cast<RobotFootprintRos>(this->robot_contours_[robot_counter]);
			}
		}
		return nullptr;
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
		this->robot_position_updates_[robot_name] = true;

		// if(this->allPositionUpdatesReceived())
		// {
		FormationContour::robotPositionChanged(robot_name);

		// this->resetPositionUpdateTable();
		// }
	}
	#pragma endregion

	#pragma region Private Helper Methods
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