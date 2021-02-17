#pragma once

#include "ros/ros.h"

#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <memory> // Usage of smart pointers
#include <vector>
#include <list>
#include <Eigen/Dense>
#include <math.h>

#include <fpp_ros/visualization_helper/visualization_helper.h>
#include <fpp_ros/path_planner/cubic_bezier_splines.h>

namespace path_planner
{
	class QuinticBezierSplines
    {
		public:
			QuinticBezierSplines(visualization_helper::VisualizationHelper *visu_helper);
			QuinticBezierSplines(visualization_helper::VisualizationHelper *visu_helper,
								 Eigen::Vector2f start_pose,
								 Eigen::Vector2f end_pose);

			void setPreviousSpline(const std::shared_ptr<QuinticBezierSplines> &previous_spline);
			void setNextSpline(const std::shared_ptr<QuinticBezierSplines> &next_spline);

			void setStartTangent(tf::Quaternion robot_orientation);
            void setStartTangent(Eigen::Vector2f start_pose_tangent);

            void setEndTangent(tf::Quaternion robot_end_orientation);
            void setEndTangent(Eigen::Vector2f next_pose);

            void calcControlPoints();
            Eigen::Vector2f calcPointOnBezierSpline(float iterator);
            std::vector<Eigen::Vector2f> calcBezierSpline(int resolution);

			Eigen::Vector2f calcFirstDerivative(float iterator);
			Eigen::Vector2f calcSecondDerivative(float iterator);

			float calcCurvation(float iterator);
			float calcCurveRadius(float iterator);
			/**
			 * @brief 
			 * 
			 * @param iterator 
			 * @param min_curve_radius 
			 * @return true Curve Radius is greater or equal to min curve radius. Curve is fine.
			 * @return false Curve radius is smaller than min curve radius. Curve is not finde.
			 */
			bool checkMinCurveRadiusAtPoint(float iterator, float min_curve_radius);
			/**
			 * @brief 
			 * 
			 * @param resolution 
			 * @param min_curve_radius 
			 * @return true Curve Radius is greater or equal to min curve radius. Curve is fine.
			 * @return false Curve radius is smaller than min curve radius. Curve is not finde.
			 */
			bool checkMinCurveRadiusOnSpline(int resolution, float min_curve_radius);

            void visualizeData();
            void addStartEndPointToVisuHelper();
            void addControlPointsToVisuHelper();
            void addBezierSplineToVisuHelper(int resolution);
            void addTangentsToVisuHelper();

            Eigen::Vector2f getStartPose();
            Eigen::Vector2f getEndPose();
            Eigen::Vector2f getStartTangent();
            Eigen::Vector2f getEndTangent();

			std::shared_ptr<path_planner::CubicBezierSplines> convertToCubicBezierSpline();

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

			std::shared_ptr<QuinticBezierSplines> previous_spline_;
			std::shared_ptr<QuinticBezierSplines> next_spline_;

            Eigen::Vector2f start_pose_;
            Eigen::Vector2f end_pose_;

            Eigen::Vector2f cp1_;
			Eigen::Vector2f cp2_;
			Eigen::Vector2f cp3_;
            Eigen::Vector2f cp4_;

            Eigen::Vector2f start_pose_tangent_;
            Eigen::Vector2f end_pose_tangent_;
    };
}