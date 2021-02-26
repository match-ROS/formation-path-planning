#include <fp_utils/bezier_splines/cubic_bezier_spline.h>

namespace bezier_splines
{
    CubicBezierSplines::CubicBezierSplines(visualization_helper::VisualizationHelper *visu_helper)
    {
        this->visu_helper_ = visu_helper;
        this->initVisuHelper();
    }

    CubicBezierSplines::CubicBezierSplines(visualization_helper::VisualizationHelper *visu_helper,
                                           Eigen::Matrix<float, 2, 1> start_pose,
                                           Eigen::Matrix<float, 2, 1> end_pose)
        : CubicBezierSplines(visu_helper)
    {
        this->start_pose_ = start_pose;
        this->end_pose_ = end_pose;
    }

	void CubicBezierSplines::setPreviousSpline(const std::shared_ptr<CubicBezierSplines> &previous_spline)
	{
		this->previous_spline_ = previous_spline;
		this->setStartTangent(this->previous_spline_->getEndTangent());
	}

	void CubicBezierSplines::setNextSpline(const std::shared_ptr<CubicBezierSplines> &next_spline)
	{
		this->next_spline_ = next_spline;
		this->setEndTangentByNextPose(this->next_spline_->getEndPose());
	}

    void CubicBezierSplines::setStartTangent(tf::Quaternion robot_orientation)
    {
        float start_to_end_length = this->calcStartToEndLength();

        tf::Vector3 direction_vector(1, 0, 0);
        tf::Vector3 rotated_vector = tf::quatRotate(robot_orientation, direction_vector);
        rotated_vector = rotated_vector * start_to_end_length; // see setEndTangent for 0.5 explanation
        this->start_tangent_ << rotated_vector[0], rotated_vector[1];
    }

    

    void CubicBezierSplines::setEndTangent(tf::Quaternion robot_end_orientation)
    {
        float start_to_end_length = this->calcStartToEndLength();

        tf::Vector3 direction_vector(1, 0, 0);
        tf::Vector3 rotated_vector = tf::quatRotate(robot_end_orientation, direction_vector);
        rotated_vector = rotated_vector * start_to_end_length;
        this->end_tangent_ << rotated_vector[0], rotated_vector[1];
    }

	

    void CubicBezierSplines::setEndTangentByNextPose(Eigen::Matrix<float, 2, 1> next_pose)
    {
        Eigen::Matrix<float, 2, 1> diff_vector_start_to_end;
        Eigen::Matrix<float, 2, 1> diff_vector_end_to_next;

        diff_vector_start_to_end = this->end_pose_ - this->start_pose_;
        diff_vector_end_to_next = next_pose - this->end_pose_;

        Eigen::Matrix<float, 2, 1> normalized_diff_vector_start_to_end = diff_vector_start_to_end;
        normalized_diff_vector_start_to_end.normalize();
        Eigen::Matrix<float, 2, 1> normalized_diff_vector_end_to_next = diff_vector_end_to_next;
        normalized_diff_vector_end_to_next.normalize();

        Eigen::Matrix<float, 2, 1> angular_bisector = normalized_diff_vector_end_to_next + normalized_diff_vector_start_to_end;
        angular_bisector.normalize();
        float length_diff_vector_end_to_next = diff_vector_end_to_next.norm(); // Use length of end point and next point to calculate the length of the tangent
        this->end_tangent_ = length_diff_vector_end_to_next * angular_bisector; // This 0.5 value is taken from the paper (p. 32 then links to Linear Geometry with Computer Graphics page 318)
    }

    void CubicBezierSplines::calcControlPoints()
    {
		// Ich glaube den Bruch hier kann ich anpassen um die Controlpoints weiter zu strecken
        this->cp1_ = (1.0 / 5.0) * this->getMultipliedStartTangent() + this->start_pose_;
        this->cp2_ = -(1.0 / 5.0) * this->getMultipliedEndTangent() + this->end_pose_;
    }

    Eigen::Matrix<float, 2, 1> CubicBezierSplines::calcPointOnBezierSpline(float iterator)
    {
        Eigen::Matrix<float, 4, 4> bezier_basis_matrix;
        Eigen::Matrix<float, 1, 4> iterator_matrix;
        
		const int bezier_degree = 3;

        Eigen::Matrix<float, 6, 1> bernstein_polynom_vector;

		for(int counter = bezier_degree; counter >= 0; counter--)
		{
			bernstein_polynom_vector[counter] = this->calcBinomialCoefficient(bezier_degree, counter) *
												std::pow(iterator, counter) *
												std::pow((1 - iterator), bezier_degree - counter);
		}
		
        Eigen::Matrix<Eigen::Matrix<float, 2, 1>, bezier_degree + 1, 1> point_matrix;
        point_matrix[0] = this->start_pose_;
        point_matrix[1] = this->cp1_;
        point_matrix[2] = this->cp2_;
        point_matrix[3] = this->end_pose_;

        Eigen::Matrix<float , 2, 1> result_vector;
        result_vector << 0, 0;
        for(int counter = 0; counter <= bezier_degree; counter++)
        {
            result_vector = result_vector + (bernstein_polynom_vector[counter] * point_matrix[counter]);
        }
        return result_vector;
    }

    std::vector<Eigen::Matrix<float, 2, 1>> CubicBezierSplines::calcBezierSpline(int resolution)
    {
        std::vector<Eigen::Matrix<float, 2, 1>> bezier_spline;
        for(int counter = 0; counter < resolution; counter++)
        {
			float iterator = (counter == 0) ? 0.0 : (float(counter) / float(resolution));
            bezier_spline.push_back(this->calcPointOnBezierSpline(iterator));
        }

		if(this->next_spline_ == nullptr)
		{
			bezier_spline.push_back(this->calcPointOnBezierSpline(1.0));
		}

        return bezier_spline;
    }

	Eigen::Vector2f CubicBezierSplines::calcSecondDerivativeValue(float iterator)
	{
		Eigen::Vector2f second_derivative_value;

		second_derivative_value = (6 * (1 - iterator) * (this->start_pose_ - 2 * this->cp1_ + this->cp2_) +
								  6 * iterator * (this->cp1_ - 2 * this->cp2_ + this->end_pose_));

		// ROS_INFO_STREAM("Iterator: " << iterator << " Second derivative value:" << second_derivative_value[0] << " | " << second_derivative_value[1] << " Start:" << this->start_pose_[0] << " | " << this->start_pose_[1] << " cp1: " << this->cp1_[0] << " | " << this->cp1_[1] << " cp2: " << this->cp2_[0] << " | " << this->cp2_[1] << " end: " << this->end_pose_[0] << " | " << this->end_pose_[1] << " end tangente: " << this->end_tangent_[0] << " | " << this->end_tangent_[1]);
		return second_derivative_value;
	}
}