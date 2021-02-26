#pragma once

#include "ros/ros.h"

#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <memory> // Usage of smart pointers
#include <vector>
#include <list>
#include <Eigen/Dense>

#include <fp_utils/visualization_helper/visualization_helper.h>


namespace bezier_splines
{
	class CubicBezierSplines
    {
        public:
            CubicBezierSplines(visualization_helper::VisualizationHelper *visu_helper);
            CubicBezierSplines(visualization_helper::VisualizationHelper *visu_helper,
                               Eigen::Vector2f start_pose,
                               Eigen::Vector2f end_pose);

			void setPreviousSpline(const std::shared_ptr<CubicBezierSplines> &previous_spline);
			void setNextSpline(const std::shared_ptr<CubicBezierSplines> &next_spline);

            void setStartTangent(tf::Quaternion robot_orientation);
            void setEndTangent(tf::Quaternion robot_end_orientation);
			void setEndTangentByNextPose(Eigen::Vector2f next_pose);

            void calcControlPoints();
            Eigen::Vector2f calcPointOnBezierSpline(float iterator);
            std::vector<Eigen::Vector2f> calcBezierSpline(int resolution);
			Eigen::Vector2f calcSecondDerivativeValue(float iterator);

        private:
    };
}