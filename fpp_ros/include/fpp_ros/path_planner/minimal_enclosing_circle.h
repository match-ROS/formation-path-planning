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
    struct CircleInfo
    {
        Eigen::Vector2f centre;
        float radius;
    };
    

    class MinimalEnclosingCircle
    {
        public:
            MinimalEnclosingCircle();

            void calcMinimalEnclosingCircle(const std::vector<Eigen::Vector2f> &points_to_enclose);

            double getCircleRadius();
            Eigen::Vector2f getCircleCentre();
            std::vector<Eigen::Vector2f> getCircleDefiningPoints();
            std::vector<Eigen::Vector2f> getEnclosedPoints();
            
        private:
            CircleInfo exeWetzlAlg(std::vector<Eigen::Vector2f> &points_to_enclose,
                                   std::vector<Eigen::Vector2f> circle_defining_points);
            CircleInfo createCircle(std::vector<Eigen::Vector2f> enclosed_points);
            CircleInfo createCircle(Eigen::Vector2f point1, Eigen::Vector2f point2);
            CircleInfo createCircle(Eigen::Vector2f point1, Eigen::Vector2f point2, Eigen::Vector2f point3);

            double calcDistance(Eigen::Vector2f first_point, Eigen::Vector2f second_point);
            Eigen::Vector2f calcCentreOfVector(Eigen::Vector2f first_point, Eigen::Vector2f second_point);

            bool isPointEnclosed(const CircleInfo &circle_info, const Eigen::Vector2f &point);
            bool arePointsEnclosed(const CircleInfo &circle_info, const std::vector<Eigen::Vector2f> &points);


            
            // void updateCircle();
            // void findNewSmallestCircle();        
            
            // Eigen::Vector2f calcOrthogonalVector(Eigen::Vector2f vector);
            // Eigen::Vector2f calcVectorLineIntersectionPoint(Eigen::Vector2f lead_vector1,
            //                                                 Eigen::Vector2f direction_vector1,
            //                                                 Eigen::Vector2f lead_vector2,
            //                                                 Eigen::Vector2f direction_vector2);
            
            // bool hasCircleAllPointsEnclosed();

            std::vector<Eigen::Vector2f> enclosed_points_;
            std::vector<Eigen::Vector2f> circle_defining_points_;

            Eigen::Vector2f circle_centre_;
            double circle_radius_;
    };
}