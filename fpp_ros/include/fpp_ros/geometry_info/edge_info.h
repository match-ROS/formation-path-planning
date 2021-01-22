#pragma once

#include <memory> // Usage of smart pointers
#include <vector>
#include <list>
#include <Eigen/Dense>

namespace geometry_info
{
    class EdgeInfo
    {
        public:
            EdgeInfo(Eigen::Vector2f start_point, Eigen::Vector2f end_point);

            Eigen::Vector2f getStartPoint();
            Eigen::Vector2f getEndPoint();
            Eigen::Vector2f getEdgeVector();


        private:
            void calculateEdgeVector();

            Eigen::Vector2f start_point_;
            Eigen::Vector2f end_point_;

            Eigen::Vector2f edge_vector_;
    };
}