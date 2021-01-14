#pragma once

#include <memory> // Usage of smart pointers
#include <vector>
#include <list>
#include <Eigen/Dense>

#include <geometry_info/edge_info.h>
#include <geometry_info/geometry_contour.h>

namespace geometry_info
{
    class FormationContour : GeometryContour
    {
        public:
            FormationContour(Eigen::Vector2d lead_vector_world_cs, float world_to_geometry_cs_rotation);

            void exeGiftWrappingAlg();
        private:
            int calcNextWrappingPoint(Eigen::Vector2d current_wrapping_point_geometry_cs,
                                      std::vector<Eigen::Vector2d> corners_to_wrap_geometry_cs,
                                      Eigen::Vector2d &next_wrapping_point_geometry_cs);

            float calcTan2(Eigen::Vector2d start_point_geometry_cs, Eigen::Vector2d end_point_geometry_cs);
    };
}