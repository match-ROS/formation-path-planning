#pragma once

#include "ros/ros.h"

#include <vector>
#include <Eigen/Dense>
#include <string>
#include <iostream>

#include <fp_utils/visualization_helper/visualization_helper.h>

namespace bezier_splines
{
	class BaseBezierSpline
	{
		public:
			#pragma region Constructors
			BaseBezierSpline(int bezier_degree);
			BaseBezierSpline(int bezier_degree,
							 visualization_helper::VisualizationHelper *visu_helper);
			BaseBezierSpline(int bezier_degree,
							 Eigen::Vector2f start_pose,
							 Eigen::Vector2f end_pose);
			BaseBezierSpline(int bezier_degree,
							 visualization_helper::VisualizationHelper *visu_helper,
							 Eigen::Vector2f start_pose,
							 Eigen::Vector2f end_pose);
			BaseBezierSpline(int bezier_degree,
							 Eigen::Vector2f start_pose,
							 Eigen::Vector2f start_tangent,
							 float start_tangent_magnitude,
							 Eigen::Vector2f end_pose,
							 Eigen::Vector2f end_tangent,
							 float end_tangent_magnitude);
			BaseBezierSpline(int bezier_degree,
							 visualization_helper::VisualizationHelper *visu_helper,
							 Eigen::Vector2f start_pose,
							 Eigen::Vector2f start_tangent,
							 float start_tangent_magnitude,
							 Eigen::Vector2f end_pose,
							 Eigen::Vector2f end_tangent,
							 float end_tangent_magnitude);
			#pragma endregion

			#pragma region Getter/Setter
			Eigen::Vector2f getStartPose();
            Eigen::Vector2f getEndPose();

            Eigen::Vector2f getStartTangent();
			void setStartTangent(Eigen::Vector2f start_pose_tangent);
			Eigen::Vector2f getEndTangent();
			void setEndTangent(Eigen::Vector2f end_tangent);

			void setStartTangentMagnitude(float start_tangent_magnitude);
			float getStartTangentMagnitude();
			void setEndTangentMagnitude(float end_tangent_magnitude);
			float getEndTangentMagnitude();

			Eigen::Vector2f getMultipliedStartTangent();
			Eigen::Vector2f getMultipliedEndTangent();
			#pragma endregion

			#pragma region DataManagement
			void setPreviousSpline(const std::shared_ptr<BaseBezierSpline> &previous_spline);
			void setNextSpline(const std::shared_ptr<BaseBezierSpline> &next_spline);
			#pragma endregion

			#pragma region BezierMethods
			void setStartTangentByQuaternion(tf::Quaternion robot_orientation);
            void setEndTangentByQuaternion(tf::Quaternion robot_end_orientation);
			void setEndTangentByNextPose(Eigen::Vector2f next_pose);
			virtual void calcControlPoints() = 0;

			virtual Eigen::Vector2f calcPointOnBezierSpline(float iterator) = 0;
			std::vector<Eigen::Vector2f> calcBezierSpline(int resolution);

			virtual Eigen::Vector2f calcFirstDerivativeValue(float iterator) = 0;
			virtual Eigen::Vector2f calcSecondDerivativeValue(float iterator) = 0;
			float calcCurvation(float iterator);
			float calcCurveRadius(float iterator);

			/**
			 * @brief 
			 * 
			 * @param iterator 
			 * @param min_curve_radius 
			 * @param point_of_failure Iterator index where the curve is to tight
			 * @return true Curve Radius is greater or equal to min curve radius. Curve is fine.
			 * @return false Curve radius is smaller than min curve radius. Curve is not finde.
			 */
			bool checkMinCurveRadiusAtPoint(float iterator, float min_curve_radius);
			/**
			 * @brief 
			 * 
			 * @param resolution 
			 * @param min_curve_radius 
			 * @param point_of_failure Iterator index where the curve is to tight
			 * @return true Curve Radius is greater or equal to min curve radius. Curve is fine.
			 * @return false Curve radius is smaller than min curve radius. Curve is not finde.
			 */
			bool checkMinCurveRadiusOnSpline(int resolution, float min_curve_radius);
			/**
			 * @brief 
			 * 
			 * @param resolution 
			 * @param min_curve_radius 
			 * @param point_of_failure Iterator index where the curve is to tight
			 * @return true Curve Radius is greater or equal to min curve radius. Curve is fine.
			 * @return false Curve radius is smaller than min curve radius. Curve is not finde.
			 */
			bool checkMinCurveRadiusOnSpline(int resolution, float min_curve_radius, int &point_of_failure);
			#pragma endregion

			#pragma region PublicVisuHelper
			void visualizeData();
            void addStartEndPointToVisuHelper();
            void addControlPointsToVisuHelper();
            void addBezierSplineToVisuHelper(int resolution);
            void addTangentsToVisuHelper();

			virtual void printInfo();
			#pragma endregion

		protected:
			std::shared_ptr<BaseBezierSpline> previous_spline_;
			std::shared_ptr<BaseBezierSpline> next_spline_;

			const int BEZIER_DEGREE = 0;

			std::vector<Eigen::Vector2f> control_points_;

			//! Stores the default value for the start tangent without any factor applied.
            Eigen::Vector2f start_tangent_;
			//! Stores the default value for the end tangent without any factor applied
            Eigen::Vector2f end_tangent_;
			//! Stores the dynamically changable parameter for changing the length of the start tangent
			float start_tangent_magnitude_;
			//! Stores the dynamically changable parameter for changing the length of the end tangent
			float end_tangent_magnitude_;

			visualization_helper::VisualizationHelper *visu_helper_;

			std::string start_end_marker_identificator_;
            std::string control_point_marker_identificator_;
            std::string tangent_marker_identificator_;
            std::string bezier_spline_identificator_;

            std::string debug_marker_identificator_;

			#pragma region InitHelper
			virtual void initControlPointList();
			#pragma endregion

			#pragma region MathHelper
			float calcStartToEndLength();
			long calcFactorial(long n);
			long calcBinomialCoefficient(long n, long k);
			#pragma endregion

			#pragma region PrivateVisuHelper
			void initVisuHelper();
			void initVisuHelper(std::string start_end_marker_identificator,
								std::string control_point_marker_identificator,
								std::string tangent_marker_identificator,
								std::string bezier_spline_identificator,
								std::string debug_marker_identificator);
			void addTangentToVisuHelper(Eigen::Vector2f start_point, Eigen::Vector2f tangent);
			void addDebugVectorToVisuHelper(Eigen::Vector2f start_point, Eigen::Vector2f vector);
			bool isVisuHelperNull();
			#pragma endregion
	};
}