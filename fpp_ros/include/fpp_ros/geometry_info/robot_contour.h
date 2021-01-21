#pragma once

#include <memory> // Usage of smart pointers
#include <vector>
#include <list>
#include <Eigen/Dense>

#include <fpp_ros/geometry_info/edge_info.h>
#include <fpp_ros/geometry_info/geometry_contour.h>

namespace geometry_info
{
    class RobotContour : public geometry_info::GeometryContour
    {
        public:
            RobotContour() : GeometryContour() { };
            RobotContour(Eigen::Vector2f lead_vector_world_cs, float world_to_geometry_cs_rotation, std::string robot_name);

            std::string getRobotName();

        private:
            std::string robot_name_;
    };
}