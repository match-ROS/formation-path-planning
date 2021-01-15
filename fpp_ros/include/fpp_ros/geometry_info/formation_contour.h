#pragma once

//delete later
#include "ros/ros.h"

#include <memory> // Usage of smart pointers
#include <algorithm>
#include <vector>
#include <list>
#include <Eigen/Dense>

#include <fpp_ros/geometry_info/edge_info.h>
#include <fpp_ros/geometry_info/geometry_contour.h>

namespace geometry_info
{
    class FormationContour : public GeometryContour
    {
        public:
            FormationContour() {};
            FormationContour(Eigen::Vector2f lead_vector_world_cs, float world_to_geometry_cs_rotation);

            void exeGiftWrappingAlg();

            void addRobotToFormation(geometry_info::GeometryContour robot_to_add);

        private:
            std::vector<geometry_info::GeometryContour> robot_contours_;

            int calcNextWrappingPoint(Eigen::Vector2f current_wrapping_point_geometry_cs,
                                      std::vector<Eigen::Vector2f> corners_to_wrap_geometry_cs,
                                      Eigen::Vector2f &next_wrapping_point_geometry_cs,
                                      float &max_radian);

            float calcTan2(Eigen::Vector2f start_point_geometry_cs, Eigen::Vector2f end_point_geometry_cs);
    };
}