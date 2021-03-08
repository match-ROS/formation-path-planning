#include <fp_utils/geometry_info/robot_contour.h>

namespace geometry_info
{
	RobotContour::RobotContour()
		: RobotContour("")
	{ }
	
	RobotContour::RobotContour(std::string robot_name)
		: GeometryContour(), robot_name_(robot_name)
	{ }

	RobotContour::RobotContour(std::string robot_name,
							   Eigen::Vector2f lead_vector_world_cs,
							   float world_to_geometry_cs_rotation)
		: GeometryContour(lead_vector_world_cs_, world_to_geometry_cs_rotation),
		  robot_name_(robot_name)
	{ }

	void RobotContour::setRobotPoseChangedEventHandler(std::function<void(std::string)> handler)
	{
		this->robot_pose_changed_handler_ = handler;
	}

	void RobotContour::notifyRobotPoseChanged()
	{
		this->robot_pose_changed_handler_(this->robot_name_);
	}

	std::string RobotContour::getRobotName()
	{
		return this->robot_name_;
	}
}