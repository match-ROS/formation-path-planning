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

namespace geometry_info
{
    struct CircleInfo
    {
        Eigen::Vector2f centre;
        float radius;
    };
    

    class MinimalEnclosingCircle
    {
        public:
            MinimalEnclosingCircle() { };

            /**
             * @brief Method for calculating minimal circle around given points
             * 
             * @param points_to_enclose Points where the minimal circle should be calculated around
             */
            void calcMinimalEnclosingCircle(const std::vector<Eigen::Vector2f> &points_to_enclose);

            /**
             * @brief Method for calculating minimal circle around a defined circle centre.
             * 
             * @param circle_centre Centre point where the circle should be drawn around
             * @param points_to_enclose Points where the minimal circle should be calculated around
             */
            void calcMinimalEnclosingCircle(const Eigen::Vector2f circle_centre, const std::vector<Eigen::Vector2f> &points_to_enclose);

            double getCircleRadius();
            Eigen::Vector2f getCircleCentre();
            std::vector<Eigen::Vector2f> getCircleDefiningPoints();
            std::vector<Eigen::Vector2f> getEnclosedPoints();
            
        private:
            /**
             * @brief Iterative method that uses the Wetzl Algorithm
             * 
             * @param points_to_enclose Points that should be enclosed. Selected points will be deleted from this list
             * @param circle_defining_points List that contains all points that create the smallest circle
             * @param points_to_go Number of points that are left to try to fit in the circle
             * @return CircleInfo Object that contains the minimal circle position and radius
             */
            CircleInfo exeWetzlAlg(std::vector<Eigen::Vector2f> &points_to_enclose,
                                   std::vector<Eigen::Vector2f> circle_defining_points,
                                   int points_to_go);
            /**
             * @brief Create a circle object from n points. Differs between the different number of points to create a circle
             * 
             * @param enclosed_points List with all points that create the circle
             * @return CircleInfo Object that contains the minimal circle position and radius
             */
            CircleInfo createCircle(std::vector<Eigen::Vector2f> enclosed_points);
            /**
             * @brief Create a circle object from two points. Centre in middle and radius is half the distance.
             * 
             * @param point1 First point that creates the circle
             * @param point2 Second point that creates the circle
             * @return CircleInfo Object that contains the minimal circle position and radius
             */
            CircleInfo createCircle(Eigen::Vector2f point1, Eigen::Vector2f point2);
            /**
             * @brief Create a circle object from three points.
             * 
             * @param point1 First point that creates the circle
             * @param point2 Second point that creates the circle
             * @param point3 Third point that creates the circle
             * @return CircleInfo Object that contains the minimal circle position and radius
             */
            CircleInfo createCircle(Eigen::Vector2f point1, Eigen::Vector2f point2, Eigen::Vector2f point3);

            /**
             * @brief Calculate euklidien distance between the points
             * 
             * @param first_point 
             * @param second_point 
             * @return double Distance from first to second point
             */
            double calcDistance(Eigen::Vector2f first_point, Eigen::Vector2f second_point);
            Eigen::Vector2f calcCentreOfVector(Eigen::Vector2f first_point, Eigen::Vector2f second_point);
            /**
             * @brief Calculates an orthogonal vector to the parameter. The length of the vector is not defined.
             * 
             * @param vector The output will be orthogonal to this vector
             * @return Eigen::Vector2f Orthogonal vector
             */
            Eigen::Vector2f calcOrthogonalVector(Eigen::Vector2f vector);
            /**
             * @brief Calculates the intersection point between two vector lines
             * 
             * @param lead_vector1 
             * @param direction_vector1 
             * @param lead_vector2 
             * @param direction_vector2 
             * @return Eigen::Vector2f 
             */
            Eigen::Vector2f calcVectorLineIntersectionPoint(Eigen::Vector2f lead_vector1,
                                                            Eigen::Vector2f direction_vector1,
                                                            Eigen::Vector2f lead_vector2,
                                                            Eigen::Vector2f direction_vector2);

            /**
             * @brief Check if the point is enclosed in the current circle
             * 
             * @param circle_info Info that describes the current circle
             * @param point Point that should be checked
             * @return true Point is in circle
             * @return false Point is not in circle
             */
            bool isPointEnclosed(const CircleInfo &circle_info, const Eigen::Vector2f &point);
            /**
             * @brief Check if all the points are enclosed in the current circle
             * 
             * @param circle_info Info that describes the current circle
             * @param points Points that should be checked
             * @return true All points are in the circle
             * @return false At least one point is not in the circle
             */
            bool arePointsEnclosed(const CircleInfo &circle_info, const std::vector<Eigen::Vector2f> &points);


            std::vector<Eigen::Vector2f> enclosed_points_;
            std::vector<Eigen::Vector2f> circle_defining_points_;

            Eigen::Vector2f circle_centre_;
            double circle_radius_;
    };
}