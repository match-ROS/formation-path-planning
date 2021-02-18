#pragma once

#include "ros/ros.h"

#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <memory> // Usage of smart pointers
#include <vector>
#include <list>
#include <Eigen/Dense>

#include <fpp_ros/visualization_helper/visualization_helper.h>


namespace path_planner
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
            void setStartTangent(Eigen::Vector2f start_pose_tangent);

            void setEndTangent(tf::Quaternion robot_end_orientation);
			void setEndTangent(Eigen::Vector2f end_tangent);
            void setEndTangentByNextPose(Eigen::Vector2f next_pose);

            void calcControlPoints();
            Eigen::Vector2f calcPointOnBezierSpline(float iterator);
            std::vector<Eigen::Vector2f> calcBezierSpline(int resolution);
			Eigen::Vector2f calcSecondDerivativeValue(float iterator);

            void visualizeData();
            void addStartEndPointToVisuHelper();
            void addControlPointsToVisuHelper();
            void addBezierSplineToVisuHelper(int resolution);
            void addTangentsToVisuHelper();

            Eigen::Vector2f getStartPose();
            Eigen::Vector2f getEndPose();
            Eigen::Vector2f getStartTangent();
			Eigen::Vector2f getMultipliedStartTangent();
            Eigen::Vector2f getEndTangent();
			Eigen::Vector2f getMultipliedEndTangent();

			void setStartTangentMagnitude(float start_tangent_magnitude);
			void setEndTangentMagnitude(float end_tangent_magnitude);
			float getStartTangentMagnitude();
			float getEndTangentMagnitude();

        private:
            float calcStartToEndLength();

            

            void initVisuHelper();
            void addTangentToVisuHelper(Eigen::Vector2f start_point, Eigen::Vector2f tangent);
            void addDebugVectorToVisuHelper(Eigen::Vector2f start_point, Eigen::Vector2f vector);
            
			long calcFactorial(long n);
			long calcBinomialCoefficient(long n, long k);

            visualization_helper::VisualizationHelper* visu_helper_;

            std::string start_end_marker_identificator_;
            std::string control_point_marker_identificator_;
            std::string tangent_marker_identificator_;
            std::string bezier_spline_identificator_;

            std::string debug_marker_identificator_;

			std::shared_ptr<CubicBezierSplines> previous_spline_;
			std::shared_ptr<CubicBezierSplines> next_spline_;

            Eigen::Vector2f start_pose_;
            Eigen::Vector2f end_pose_;

            Eigen::Vector2f cp1_;
            Eigen::Vector2f cp2_;

			//! Stores the default value for the start tangent without any factor applied.
            Eigen::Vector2f start_tangent_;
			//! Stores the default value for the end tangent without any factor applied
            Eigen::Vector2f end_tangent_;
			//! Stores the dynamically changable parameter for changing the length of the start tangent
			float start_tangent_magnitude_;
			//! Stores the dynamically changable parameter for changing the length of the end tangent
			float end_tangent_magnitude_;
    };
}