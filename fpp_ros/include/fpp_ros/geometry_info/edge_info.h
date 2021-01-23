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
            /**
             * @brief Construct for the EdgeInfo object.
             * Please notice that both points are in the same coordinate system.
             * 
             * @param start_point Start point of the line
             * @param end_point End point of the line
             */
            EdgeInfo(Eigen::Vector2f start_point, Eigen::Vector2f end_point);

            Eigen::Vector2f getStartPoint();
            Eigen::Vector2f getEndPoint();
            Eigen::Vector2f getEdgeVector();


        private:
            /**
             * @brief Calculated the vector between the start and end point
             * 
             */
            void calculateEdgeVector();

            //! Start point of the line
            Eigen::Vector2f start_point_;
            //! End point of the line
            Eigen::Vector2f end_point_;
            //! Relativ vector that points from the start to the end point
            Eigen::Vector2f edge_vector_;
    };
}