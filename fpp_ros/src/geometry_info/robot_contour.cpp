#include <fpp_ros/geometry_info/robot_contour.h>

namespace geometry_info
{
    RobotContour::RobotContour(Eigen::Vector2f lead_vector_world_cs, float world_to_geometry_cs_rotation, std::string robot_name)
    : GeometryContour(lead_vector_world_cs, world_to_geometry_cs_rotation)
    {
        this->robot_name_ = robot_name;
    }

    std::string RobotContour::getRobotName()
    {
        return this->robot_name_;
    }
}