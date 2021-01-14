#pragma once

#include <memory> // Usage of smart pointers
#include <vector>
#include <list>
#include <Eigen/Dense>

#include <geometry_info/edge_info.h>

namespace geometry_info
{
    class GeometryContour
    {
        public:
            GeometryContour(Eigen::Vector2d lead_vector_world_cs, float world_to_geometry_cs_rotation);

            void addContourCornerWorldCS(Eigen::Vector2d corner_world_cs);

            void createContourEdges();

            Eigen::Vector2d transformWorldToGeometryCS(Eigen::Vector2d world_cs);
            Eigen::Vector2d transformGeometryToWorldCS(Eigen::Vector2d geometry_cs);
        protected:
            Eigen::Matrix<float, 3, 3> createTransformationMatrix(Eigen::Vector2d lead_vector_world_cs, float rotation);

            Eigen::Vector2d lead_vector_world_cs_;
            float world_to_geometry_cs_rotation_;

            Eigen::Matrix<float, 3, 3> tf_world_to_geometry_cs_;
            Eigen::Matrix<float, 3, 3> tf_geometry_to_world_cs_;

            std::vector<Eigen::Vector2d> corner_points_geometry_cs_;
            std::vector<geometry_info::EdgeInfo> edge_list_geometry_cs_;
    };
}