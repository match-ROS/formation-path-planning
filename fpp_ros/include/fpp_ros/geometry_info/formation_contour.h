#pragma once

//delete later
#include "ros/ros.h"

#include <iostream>
#include <memory> // Usage of smart pointers
#include <algorithm>
#include <vector>
#include <list>
#include <Eigen/Dense>

#include <fpp_ros/geometry_info/edge_info.h>
#include <fpp_ros/geometry_info/robot_contour.h>

namespace geometry_info
{
    class FormationContour : public GeometryContour
    {
        public:
            FormationContour() {};
            FormationContour(Eigen::Vector2f lead_vector_world_cs, float world_to_geometry_cs_rotation);

            void exeGiftWrappingAlg();

            void addRobotToFormation(geometry_info::RobotContour robot_to_add);

            /**
             * @brief Method for updating the position of one individual robot
             * 
             * @param robot_name Name of the robot of which the position should be changed
             * @param new_robot_pose_world_cs New position where the base_link/cs of the robot should be
             * @param new_rotation_world_to_geometry_cs New rotation from world to geometry cs
             */
            void updateRobotPose(std::string robot_name,
                                 Eigen::Vector2f new_robot_pose_world_cs,
                                 float new_rotation_world_to_geometry_cs);

            /**
             * @brief Method for updating the corner points of the formation
             * Notice that the positions of each robot have to be updated first
             */
            void updateFormationContour();


        private:
            std::vector<geometry_info::RobotContour> robot_contours_;

            int calcNextWrappingPoint(Eigen::Vector2f current_wrapping_point_geometry_cs,
                                      std::vector<Eigen::Vector2f> corners_to_wrap_geometry_cs,
                                      Eigen::Vector2f &next_wrapping_point_geometry_cs,
                                      float &max_radian);

            float calcTan2(Eigen::Vector2f start_point_geometry_cs, Eigen::Vector2f end_point_geometry_cs);
    };
}