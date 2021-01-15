#pragma once

//Delete this later
#include "ros/ros.h"

#include <memory> // Usage of smart pointers
#include <vector>
#include <list>
#include <Eigen/Dense>

#include <fpp_ros/geometry_info/edge_info.h>

namespace geometry_info
{
    class GeometryContour
    {
        public:
            GeometryContour() {};
            GeometryContour(Eigen::Vector2f lead_vector_world_cs, float world_to_geometry_cs_rotation);

            void addContourCornerWorldCS(Eigen::Vector2f corner_world_cs);
            void addContourCornerGeometryCS(Eigen::Vector2f corner_geometry_cs);
            void addContourCornersWorldCS(std::vector<Eigen::Vector2f> corner_list_world_cs);

            void createContourEdges();

            Eigen::Vector2f transformWorldToGeometryCS(Eigen::Vector2f world_cs);
            Eigen::Vector2f transformGeometryToWorldCS(Eigen::Vector2f geometry_cs);

            std::vector<Eigen::Vector2f> getCornerPointsWorldCS();

            friend bool operator==(const GeometryContour &lhs, const GeometryContour &rhs);

        protected:
            Eigen::Matrix<float, 3, 3> createTransformationMatrix(Eigen::Vector2f lead_vector_world_cs, float rotation);

            Eigen::Vector2f lead_vector_world_cs_;
            float world_to_geometry_cs_rotation_;

            Eigen::Matrix<float, 3, 3> tf_world_to_geometry_cs_;
            Eigen::Matrix<float, 3, 3> tf_geometry_to_world_cs_;

            std::vector<Eigen::Vector2f> corner_points_geometry_cs_;
            std::vector<geometry_info::EdgeInfo> edge_list_geometry_cs_;
    };
}