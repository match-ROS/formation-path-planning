#pragma once

#include "ros/ros.h"

#include <tf/transform_listener.h>

#include <memory> // Usage of smart pointers
#include <vector>
#include <list>
#include <Eigen/Dense>


namespace bezier_splines
{
    class CubicBezierSplines
    {
        public:
            CubicBezierSplines();
            CubicBezierSplines(Eigen::Matrix<float, 2, 1> start_pose, Eigen::Matrix<float, 2, 1> end_pose);

            void setStartTangent(tf::Quaternion robot_orientation);
            void setStartTangent(Eigen::Matrix<float, 2, 1> start_pose_tangent);

            void setEndTangent(tf::Quaternion robot_end_orientation);
            void setEndTangent(tf::Pose next_pose);

            float calcStartToEndLength();

            // CubicBezierSplines(Eigen::Matrix<float, 2, 1> start_pose, float start_first_derivative_value, Eigen::Matrix<float, 2, 1> end_pose);

            // /**
            //  * @brief This method links last 
            //  * 
            //  * @param previous_spline 
            //  */
            // CubicBezierSplines(std::shared_ptr<CubicBezierSplines> previous_spline, Eigen::Matrix<float, 2, 1> end_pose);
            
            // void setNextSpline(std::shared_ptr<CubicBezierSplines> next_spline);

            // void calcFirstSupportPose();
            // void calcSecondSupportPose(Eigen::Matrix<float, 2, 1> end_vector);

            // bool isFirstSpline();
            // bool isLastSpline();

        private:

            Eigen::Matrix<float, 2, 1> start_pose_;
            Eigen::Matrix<float, 2, 1> end_pose_;

            Eigen::Matrix<float, 2, 1> start_pose_tangent_;
            Eigen::Matrix<float, 2, 1> end_pose_tangent_;
    };
}