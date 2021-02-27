#include <fp_utils/bezier_splines/cubic_bezier_spline.h>

namespace bezier_splines
{
	CubicBezierSplines::CubicBezierSplines()
		: BaseBezierSpline(3) {}
	CubicBezierSplines::CubicBezierSplines(visualization_helper::VisualizationHelper *visu_helper)
		: BaseBezierSpline(3, visu_helper) {}
	CubicBezierSplines::CubicBezierSplines(Eigen::Vector2f start_pose,
										   Eigen::Vector2f end_pose)
		: BaseBezierSpline(3, start_pose, end_pose) {}
	CubicBezierSplines::CubicBezierSplines(visualization_helper::VisualizationHelper *visu_helper,
										   Eigen::Vector2f start_pose,
										   Eigen::Vector2f end_pose)
		: BaseBezierSpline(3, visu_helper, start_pose, end_pose) {}
	CubicBezierSplines::CubicBezierSplines(Eigen::Vector2f start_pose,
										   Eigen::Vector2f start_tangent,
										   float start_tangent_magnitude,
										   Eigen::Vector2f end_pose,
										   Eigen::Vector2f end_tangent,
										   float end_tangent_magnitude)
		: BaseBezierSpline(3,
						   start_pose,
						   start_tangent,
						   start_tangent_magnitude,
						   end_pose,
						   end_tangent,
						   end_tangent_magnitude) {}
	CubicBezierSplines::CubicBezierSplines(visualization_helper::VisualizationHelper *visu_helper,
										   Eigen::Vector2f start_pose,
										   Eigen::Vector2f start_tangent,
										   float start_tangent_magnitude,
										   Eigen::Vector2f end_pose,
										   Eigen::Vector2f end_tangent,
										   float end_tangent_magnitude)
		: BaseBezierSpline(3,
						   visu_helper,
						   start_pose, 
						   start_tangent,
						   start_tangent_magnitude,
						   end_pose, 
						   end_tangent,
						   end_tangent_magnitude) {}

	void CubicBezierSplines::calcControlPoints()
    {
		// This formula comes from S'(0) and S'(1). Has to be (1/5)
		this->control_points_[1] = (1.0 / 5.0) * this->getMultipliedStartTangent() +
								   this->control_points_.front();
		this->control_points_[2] = -(1.0 / 5.0) * this->getMultipliedEndTangent() +
								   this->control_points_.back();
	}

    Eigen::Vector2f CubicBezierSplines::calcPointOnBezierSpline(float iterator)
    {
        Eigen::Matrix<float, 4, 4> bezier_basis_matrix;
        Eigen::Matrix<float, 1, 4> iterator_matrix;

        Eigen::Matrix<float, 6, 1> bernstein_polynom_vector;

		for(int counter = this->BEZIER_DEGREE; counter >= 0; counter--)
		{
			bernstein_polynom_vector[counter] = this->calcBinomialCoefficient(this->BEZIER_DEGREE, counter) *
												std::pow(iterator, counter) *
												std::pow((1 - iterator), this->BEZIER_DEGREE - counter);
		}
		
        Eigen::Vector2f result_vector;
        result_vector << 0, 0;
        for(int counter = 0; counter <= this->BEZIER_DEGREE; counter++)
        {
            result_vector = result_vector + (bernstein_polynom_vector[counter] * this->control_points_[counter]);
        }
        return result_vector;
    }

	Eigen::Vector2f CubicBezierSplines::calcFirstDerivativeValue(float iterator)
	{
		Eigen::Vector2f first_derivative_value;

		first_derivative_value = 3 * std::pow(1 - iterator, 2) * (this->control_points_[1] - this->control_points_.front()) +
								 6 * (1 - iterator) * iterator * (this->control_points_[2] - this->control_points_[1]) +
								 3 * std::pow(iterator, 2) * (this->control_points_.back() - this->control_points_[2]);

		return first_derivative_value;								 
	}

	Eigen::Vector2f CubicBezierSplines::calcSecondDerivativeValue(float iterator)
	{
		Eigen::Vector2f second_derivative_value;

		second_derivative_value = 6 * (1 - iterator) *
									  (this->control_points_.front() - 2 * this->control_points_[1] + this->control_points_[2]) +
								  6 * iterator *
									  (this->control_points_[1] - 2 * this->control_points_[2] + this->control_points_.back());

		// ROS_INFO_STREAM("Iterator: " << iterator << " Second derivative value:" << second_derivative_value[0] << " | " << second_derivative_value[1] << " Start:" << this->start_pose_[0] << " | " << this->start_pose_[1] << " cp1: " << this->cp1_[0] << " | " << this->cp1_[1] << " cp2: " << this->cp2_[0] << " | " << this->cp2_[1] << " end: " << this->end_pose_[0] << " | " << this->end_pose_[1] << " end tangente: " << this->end_tangent_[0] << " | " << this->end_tangent_[1]);
		return second_derivative_value;
	}

	void CubicBezierSplines::printInfo()
	{
		BaseBezierSpline::printInfo();
	}
}