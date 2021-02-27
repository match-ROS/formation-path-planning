#include <fp_utils/bezier_splines/quintic_bezier_spline.h>

namespace bezier_splines
{
	QuinticBezierSplines::QuinticBezierSplines()
		: BaseBezierSpline(5) {}
	QuinticBezierSplines::QuinticBezierSplines(visualization_helper::VisualizationHelper *visu_helper)
		: BaseBezierSpline(5, visu_helper) {}
	QuinticBezierSplines::QuinticBezierSplines(Eigen::Vector2f start_pose,
											   Eigen::Vector2f end_pose)
		: BaseBezierSpline(5, start_pose, end_pose) {}
	QuinticBezierSplines::QuinticBezierSplines(visualization_helper::VisualizationHelper *visu_helper,
											   Eigen::Vector2f start_pose,
											   Eigen::Vector2f end_pose)
		: BaseBezierSpline(5, visu_helper, start_pose, end_pose) {}
	QuinticBezierSplines::QuinticBezierSplines(Eigen::Vector2f start_pose,
											   Eigen::Vector2f start_tangent,
											   float start_tangent_magnitude,
											   Eigen::Vector2f end_pose,
											   Eigen::Vector2f end_tangent,
											   float end_tangent_magnitude)
		: BaseBezierSpline(5,
						   start_pose,
						   start_tangent,
						   start_tangent_magnitude,
						   end_pose,
						   end_tangent,
						   end_tangent_magnitude) {}
	QuinticBezierSplines::QuinticBezierSplines(visualization_helper::VisualizationHelper *visu_helper,
											   Eigen::Vector2f start_pose,
											   Eigen::Vector2f start_tangent,
											   float start_tangent_magnitude,
											   Eigen::Vector2f end_pose,
											   Eigen::Vector2f end_tangent,
											   float end_tangent_magnitude)
		: BaseBezierSpline(5,
						   visu_helper,
						   start_pose,
						   start_tangent,
						   start_tangent_magnitude,
						   end_pose,
						   end_tangent,
						   end_tangent_magnitude) {}

#pragma region BezierMethods
    void QuinticBezierSplines::calcControlPoints()
    {
		// This formula comes from S'(0) and S'(1). Has to be (1/5)
        this->control_points_[1] = (1.0 / 5.0) * this->getMultipliedStartTangent() + this->control_points_.front();
        this->control_points_[4] = -(1.0 / 5.0) * this->getMultipliedEndTangent() + this->control_points_.back();

		std::shared_ptr<bezier_splines::CubicBezierSplines> previous_spline = nullptr;
		std::shared_ptr<bezier_splines::CubicBezierSplines> current_spline = nullptr;
		std::shared_ptr<bezier_splines::CubicBezierSplines> next_spline = nullptr;

		if (this->previous_spline_ != nullptr)
		{
			previous_spline = this->createCubicBezierSpline(this->previous_spline_);
		}
		current_spline = this->createCubicBezierSpline(std::make_shared<bezier_splines::QuinticBezierSplines>(*this));
		if(this->next_spline_ != nullptr)
		{
			next_spline = this->createCubicBezierSpline(this->next_spline_);
		}

		if(this->previous_spline_ != nullptr)
		{
			previous_spline->setStartTangent(this->previous_spline_->getStartTangent());
			previous_spline->setNextSpline(current_spline);
			previous_spline->calcControlPoints(); // Everything is set for the previous spline. Control points can be calculated

			current_spline->setPreviousSpline(previous_spline);
		}
		else
		{
			current_spline->setStartTangent(this->start_tangent_);
		}

		if(this->next_spline_ != nullptr)
		{
			current_spline->setNextSpline(next_spline);
			next_spline->setPreviousSpline(current_spline);
			next_spline->setEndTangent(this->next_spline_->getEndTangent());
			next_spline->calcControlPoints(); // Everything is set for the next spline. Control points can be calculated
		}
		else
		{
			current_spline->setEndTangent(this->end_tangent_);
		}

		current_spline->calcControlPoints(); // Everything is set for the current spline. Control points can be calculated

		Eigen::Vector2f curr_spline_second_derivative_start_val = current_spline->calcSecondDerivativeValue(0.0);
		Eigen::Vector2f average_start_second_derivative_val;
		if(previous_spline != nullptr) // Calc second derivative value through average of both connecting splines
		{
			Eigen::Vector2f prev_spline_second_derivative_val = previous_spline->calcSecondDerivativeValue(1.0);
			
			average_start_second_derivative_val = 0.5 * (prev_spline_second_derivative_val + curr_spline_second_derivative_start_val);
		}
		else // Just use second derivative of the cubic spline
		{
			average_start_second_derivative_val = curr_spline_second_derivative_start_val;
		}
		this->control_points_[2] = (1.0 / 20.0) * average_start_second_derivative_val + 2.0 * this->control_points_[1] - this->control_points_.front();

		Eigen::Vector2f curr_spline_second_derivative_end_val = current_spline->calcSecondDerivativeValue(1.0);
		Eigen::Vector2f average_end_second_derivative_val;
		if(next_spline != nullptr) // Calc second derivative value through average of both connecting splines
		{
			Eigen::Vector2f next_spline_second_derivative_val = next_spline->calcSecondDerivativeValue(0.0);
			
			average_end_second_derivative_val = 0.5 * (next_spline_second_derivative_val + curr_spline_second_derivative_end_val);
			// ROS_INFO_STREAM("average: " << average_end_second_derivative_val[0] << " | " << average_end_second_derivative_val[1] << " next: " << next_spline_second_derivative_val[0] << " | " << next_spline_second_derivative_val[1] << " curr: " << curr_spline_second_derivative_end_val[0] << " | " << curr_spline_second_derivative_end_val[1]);
		}
		else // Just use the second derivative of the cubic spline
		{
			average_end_second_derivative_val = curr_spline_second_derivative_end_val;
		}
		this->control_points_[3] = (1.0 / 20.0) * average_end_second_derivative_val + 2.0 * this->control_points_[4] - this->control_points_.back();
		// ROS_INFO_STREAM("cp3_: " << cp3_[0] << " | " << cp3_[1] << "cp4_: " << cp4_[0] << " | " << cp4_[1] << "end_pose_: " << end_pose_[0] << " | " << end_pose_[1]);
    }

    Eigen::Vector2f QuinticBezierSplines::calcPointOnBezierSpline(float iterator)
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

	Eigen::Vector2f QuinticBezierSplines::calcFirstDerivativeValue(float iterator)
	{
		Eigen::Vector2f first_derivative_value =
			5 * std::pow(1 - iterator, 4) * (this->control_points_[1] - this->control_points_.front()) +
			20 * std::pow(1 - iterator, 3) * iterator * (this->control_points_[2] - this->control_points_[1]) +
			30 * std::pow(1 - iterator, 2) * std::pow(iterator, 2) * (this->control_points_[3] - this->control_points_[2]) +
			20 * (1 - iterator) * std::pow(iterator, 3) * (this->control_points_[4] - this->control_points_[3]) +
			5 * std::pow(iterator, 4) * (this->control_points_.back() - this->control_points_[4]);

		return first_derivative_value;									   
	}

	Eigen::Vector2f QuinticBezierSplines::calcSecondDerivativeValue(float iterator)
	{
		Eigen::Vector2f second_derivative_value =
			20 * std::pow(1 - iterator, 3) * (this->control_points_.front() - 2 * this->control_points_[1] + this->control_points_[2]) +
			60 * std::pow(1 - iterator, 2) * iterator * (this->control_points_[1] - 2 * this->control_points_[2] + this->control_points_[3]) +
			60 * (1 - iterator) * std::pow(iterator, 2) * (this->control_points_[2] - 2 * this->control_points_[3] + this->control_points_[4]) +
			20 * std::pow(iterator, 3) * (this->control_points_[3] - 2 * this->control_points_[4] + this->control_points_.back());

		return second_derivative_value;												  
	}

	#pragma endregion

	std::shared_ptr<bezier_splines::CubicBezierSplines> QuinticBezierSplines::createCubicBezierSpline(std::shared_ptr<bezier_splines::BaseBezierSpline> base_spline)
	{
		std::shared_ptr<bezier_splines::CubicBezierSplines> cubic_spline =
			std::make_shared<bezier_splines::CubicBezierSplines>(base_spline->getStartPose(),
																 base_spline->getStartTangent(),
																 base_spline->getStartTangentMagnitude(),
																 base_spline->getEndPose(),
																 base_spline->getEndTangent(),
																 base_spline->getEndTangentMagnitude());

		return cubic_spline;
	}

	void QuinticBezierSplines::printInfo()
	{
		BaseBezierSpline::printInfo();
	}
}