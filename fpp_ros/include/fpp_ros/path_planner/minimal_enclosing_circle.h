#pragma once

#include <iostream>
#include <string>
#include <memory> // Usage of smart pointers
#include <vector>
#include <list>
#include <set>
#include <map>
#include <algorithm>
#include <Eigen/Dense>

namespace fpp_helper
{
    class MinimalEnclosingCircle
    {
        public:
            MinimalEnclosingCircle();

            void calcMinimalEnclosingCircle(std::vector<Eigen::Vector2d> points_to_enclose);
        private:
            void updateCircle();
            void findNewSmallestCircle();        
            Eigen::Vector2d calcCentreOfVector(Eigen::Vector2d first_point, Eigen::Vector2d second_point);
            Eigen::Vector2d calcCentreOfVector(Eigen::Vector2d start_point, Eigen::Vector2d vector_to_end_point);
            Eigen::Vector2d calcOrthogonalVector(Eigen::Vector2d vector);
            Eigen::Vector2d MinimalEnclosingCircle::calcVectorLineIntersectionPoint(Eigen::Vector2d lead_vector1,
                                                                                    Eigen::Vector2d direction_vector1,
                                                                                    Eigen::Vector2d lead_vector2,
                                                                                    Eigen::Vector2d direction_vector2);
            double calcDistance(Eigen::Vector2d first_point, Eigen::Vector2d second_point);

            std::vector<Eigen::Vector2d> enclosed_points_;
            std::vector<Eigen::Vector2d> circle_defining_points_;

            Eigen::Vector2d circle_centre_;
            double circle_radius_;
    };
}