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

            void calcMinimalEnclosingCircle(std::vector<Eigen::Vector2f> points_to_enclose);

            double getCircleRadius();
            Eigen::Vector2f getCircleCentre();
            std::vector<Eigen::Vector2f> getCircleDefiningPoints();
            std::vector<Eigen::Vector2f> getEnclosedPoints();
        private:
            void updateCircle();
            void findNewSmallestCircle();        
            Eigen::Vector2f calcCentreOfVector(Eigen::Vector2f first_point, Eigen::Vector2f second_point);
            Eigen::Vector2f calcOrthogonalVector(Eigen::Vector2f vector);
            Eigen::Vector2f calcVectorLineIntersectionPoint(Eigen::Vector2f lead_vector1,
                                                            Eigen::Vector2f direction_vector1,
                                                            Eigen::Vector2f lead_vector2,
                                                            Eigen::Vector2f direction_vector2);
            double calcDistance(Eigen::Vector2f first_point, Eigen::Vector2f second_point);
            bool hasCircleAllPointsEnclosed();

            std::vector<Eigen::Vector2f> enclosed_points_;
            std::vector<Eigen::Vector2f> circle_defining_points_;

            Eigen::Vector2f circle_centre_;
            double circle_radius_;
    };
}