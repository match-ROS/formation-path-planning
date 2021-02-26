#include <fp_utils/bezier_splines/quintic_bezier_spline.h>

namespace bezier_splines
{
    QuinticBezierSplines::QuinticBezierSplines(visualization_helper::VisualizationHelper *visu_helper)
		: previous_spline_(nullptr), next_spline_(nullptr), start_tangent_magnitude_(0.25), end_tangent_magnitude_(0.25)
    {
        this->visu_helper_ = visu_helper;
        this->initVisuHelper();
    }

    QuinticBezierSplines::QuinticBezierSplines(visualization_helper::VisualizationHelper *visu_helper,
                                           Eigen::Vector2f start_pose,
                                           Eigen::Vector2f end_pose)
        : QuinticBezierSplines(visu_helper)
    {
        this->start_pose_ = start_pose;
        this->end_pose_ = end_pose;
    }

	void QuinticBezierSplines::setPreviousSpline(const std::shared_ptr<QuinticBezierSplines> &previous_spline)
	{
		this->previous_spline_ = previous_spline;
		this->setStartTangent(this->previous_spline_->getEndTangent());
		this->setStartTangentMagnitude(this->previous_spline_->getEndTangentMagnitude());
	}

	void QuinticBezierSplines::setNextSpline(const std::shared_ptr<QuinticBezierSplines> &next_spline)
	{
		this->next_spline_ = next_spline;
		this->setEndTangentByNextPose(this->next_spline_->getEndPose());
	}

    void QuinticBezierSplines::setStartTangent(tf::Quaternion robot_orientation)
    {
		float start_to_end_length = this->calcStartToEndLength();

        tf::Vector3 direction_vector(1, 0, 0);
        tf::Vector3 rotated_vector = tf::quatRotate(robot_orientation, direction_vector);
		// In the paper a factor of 0.5 was used to shorten the tangent. This will no longer be applied directly.
		// The factor will now be applied through the Poperty "getMultipliedStartTangent"
		// (p. 32 then links to Linear Geometry with Computer Graphics page 318)
        rotated_vector = rotated_vector * start_to_end_length;
        this->start_tangent_ << rotated_vector[0], rotated_vector[1];

		ROS_INFO_STREAM("start_pose_tangent:" << this->start_tangent_[0] << " | " << this->start_tangent_[1]);
    }

    void QuinticBezierSplines::setStartTangent(Eigen::Vector2f start_tangent)
    {
        this->start_tangent_ = start_tangent;
    }

	// void QuinticBezierSplines::setMultipliedStartTangent(Eigen::Vector2f start_tangent, float start_tangent_magnitude)
	// {
	// 	this->start_tangent_ = start_tangent;
	// 	this->start_tangent_magnitude_ = start_tangent_magnitude;
	// }

    void QuinticBezierSplines::setEndTangent(tf::Quaternion robot_end_orientation)
    {
        float start_to_end_length = this->calcStartToEndLength();

        tf::Vector3 direction_vector(1, 0, 0);
        tf::Vector3 rotated_vector = tf::quatRotate(robot_end_orientation, direction_vector);
        rotated_vector = rotated_vector * start_to_end_length;
        this->end_tangent_ << rotated_vector[0], rotated_vector[1];

		ROS_INFO_STREAM("end_pose_tangent:" << this->end_tangent_[0] << " | " << this->end_tangent_[1]);
    }

	void QuinticBezierSplines::setEndTangent(Eigen::Vector2f end_tangent)
	{
		this->end_tangent_ = end_tangent;
	}

    void QuinticBezierSplines::setEndTangentByNextPose(Eigen::Vector2f next_pose)
    {
        Eigen::Vector2f diff_vector_start_to_end;
        Eigen::Vector2f diff_vector_end_to_next;

        diff_vector_start_to_end = this->end_pose_ - this->start_pose_;
        diff_vector_end_to_next = next_pose - this->end_pose_;

        Eigen::Vector2f normalized_diff_vector_start_to_end = diff_vector_start_to_end;
        normalized_diff_vector_start_to_end.normalize();
        Eigen::Vector2f normalized_diff_vector_end_to_next = diff_vector_end_to_next;
        normalized_diff_vector_end_to_next.normalize();

        Eigen::Vector2f angular_bisector = normalized_diff_vector_end_to_next + normalized_diff_vector_start_to_end;
        angular_bisector.normalize();
        float length_diff_vector_end_to_next = diff_vector_end_to_next.norm(); // Use length of end point and next point to calculate the length of the tangent
		// In the paper a factor of 0.5 was used to shorten the tangent. This will no longer be applied directly.
		// The factor will now be applied through the Poperty "getMultipliedEndTangent"
		// (p. 32 then links to Linear Geometry with Computer Graphics page 318)
        this->end_tangent_ = length_diff_vector_end_to_next * angular_bisector; 
    }

	// void QuinticBezierSplines::setMultipliedEndTangent(Eigen::Vector2f end_tangent, float end_tangent_magnitude)
	// {
	// 	this->end_tangent_ = end_tangent;
	// 	this->end_tangent_magnitude_ = end_tangent_magnitude;
	// }

    void QuinticBezierSplines::visualizeData()
    {
        this->visu_helper_->visualizeMarkerArray(this->start_end_marker_identificator_);
        this->visu_helper_->visualizeMarkerArray(this->control_point_marker_identificator_);
        this->visu_helper_->visualizeMarkerArray(this->tangent_marker_identificator_);
        this->visu_helper_->visualizeMarkerArray(this->bezier_spline_identificator_);
        // this->visu_helper_->visualizeMarkerArray(this->debug_marker_identificator_);

        ros::Duration(0.1).sleep(); // Wait for markers to be shown, maybe this helps to visualize them every time
    }

    Eigen::Vector2f QuinticBezierSplines::getStartPose()
    {
        return this->start_pose_;
    }

    Eigen::Vector2f QuinticBezierSplines::getEndPose()
    {
        return this->end_pose_;
    }

    Eigen::Vector2f QuinticBezierSplines::getStartTangent()
    {
        return this->start_tangent_;
    }

	Eigen::Vector2f QuinticBezierSplines::getMultipliedStartTangent()
	{
		return this->start_tangent_magnitude_ * this->start_tangent_;
	}

    Eigen::Vector2f QuinticBezierSplines::getEndTangent()
    {
        return this->end_tangent_;
    }

	Eigen::Vector2f QuinticBezierSplines::getMultipliedEndTangent()
	{
		return this->end_tangent_magnitude_ * this->end_tangent_;
	}

	void QuinticBezierSplines::setStartTangentMagnitude(float start_tangent_magnitude)
	{
		this->start_tangent_magnitude_ = start_tangent_magnitude;
	}

	void QuinticBezierSplines::setEndTangentMagnitude(float end_tangent_magnitude)
	{
		this->end_tangent_magnitude_ = end_tangent_magnitude;
	}

	float QuinticBezierSplines::getStartTangentMagnitude()
	{
		return this->start_tangent_magnitude_;
	}

	float QuinticBezierSplines::getEndTangentMagnitude()
	{
		return this->end_tangent_magnitude_;
	}

    float QuinticBezierSplines::calcStartToEndLength()
    {
        return std::sqrt(std::pow((this->end_pose_[0] - this->start_pose_[0]), 2) + std::pow((this->end_pose_[1] - this->start_pose_[1]), 2));
    }

    void QuinticBezierSplines::calcControlPoints()
    {
		// This formula comes from S'(0) and S'(1). Has to be (1/5)
        this->cp1_ = (1.0 / 5.0) * this->getMultipliedStartTangent() + this->start_pose_;
        this->cp4_ = -(1.0 / 5.0) * this->getMultipliedEndTangent() + this->end_pose_;

		std::shared_ptr<path_planner::CubicBezierSplines> previous_spline = nullptr;
		std::shared_ptr<path_planner::CubicBezierSplines> current_spline = nullptr;
		std::shared_ptr<path_planner::CubicBezierSplines> next_spline = nullptr;

		if(this->previous_spline_ != nullptr)
		{
			previous_spline = this->previous_spline_->convertToCubicBezierSpline();
		}
		current_spline = this->convertToCubicBezierSpline();
		if(this->next_spline_ != nullptr)
		{
			next_spline = this->next_spline_->convertToCubicBezierSpline();
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
		this->cp2_ = (1.0 / 20.0) * average_start_second_derivative_val + 2.0 * this->cp1_ - this->start_pose_;

		ROS_INFO_STREAM("Current Spline");
		Eigen::Vector2f curr_spline_second_derivative_end_val = current_spline->calcSecondDerivativeValue(1.0);
		Eigen::Vector2f average_end_second_derivative_val;
		if(next_spline != nullptr) // Calc second derivative value through average of both connecting splines
		{
			ROS_INFO_STREAM("Next Spline");
			Eigen::Vector2f next_spline_second_derivative_val = next_spline->calcSecondDerivativeValue(0.0);
			
			average_end_second_derivative_val = 0.5 * (next_spline_second_derivative_val + curr_spline_second_derivative_end_val);
			ROS_INFO_STREAM("average: " << average_end_second_derivative_val[0] << " | " << average_end_second_derivative_val[1] << " next: " << next_spline_second_derivative_val[0] << " | " << next_spline_second_derivative_val[1] << " curr: " << curr_spline_second_derivative_end_val[0] << " | " << curr_spline_second_derivative_end_val[1]);
		}
		else // Just use the second derivative of the cubic spline
		{
			average_end_second_derivative_val = curr_spline_second_derivative_end_val;
		}
		this->cp3_ = (1.0 / 20.0) * average_end_second_derivative_val + 2.0 * this->cp4_ - this->end_pose_;
		ROS_INFO_STREAM("cp3_: " << cp3_[0] << " | " << cp3_[1] << "cp4_: " << cp4_[0] << " | " << cp4_[1] << "end_pose_: " << end_pose_[0] << " | " << end_pose_[1]);
    }

    Eigen::Vector2f QuinticBezierSplines::calcPointOnBezierSpline(float iterator)
    {
        Eigen::Matrix<float, 4, 4> bezier_basis_matrix;
        Eigen::Matrix<float, 1, 4> iterator_matrix;

		const int bezier_degree = 5;

		Eigen::Matrix<float, 6, 1> bernstein_polynom_vector;
		
		for(int counter = bezier_degree; counter >= 0; counter--)
		{
			bernstein_polynom_vector[counter] = this->calcBinomialCoefficient(bezier_degree, counter) *
												std::pow(iterator, counter) *
												std::pow((1 - iterator), bezier_degree - counter);
		}

        Eigen::Matrix<Eigen::Vector2f, 6, 1> point_matrix;
        point_matrix[0] = this->start_pose_;
        point_matrix[1] = this->cp1_;
		point_matrix[2] = this->cp2_;
		point_matrix[3] = this->cp3_;
        point_matrix[4] = this->cp4_;
        point_matrix[5] = this->end_pose_;

        Eigen::Vector2f result_vector;
        result_vector << 0, 0;
        for(int counter = 0; counter < point_matrix.size(); counter++)
        {
			result_vector = result_vector + (bernstein_polynom_vector[counter] * point_matrix[counter]);
        }
        return result_vector;
    }

	std::shared_ptr<path_planner::CubicBezierSplines> QuinticBezierSplines::convertToCubicBezierSpline()
	{
		std::shared_ptr<path_planner::CubicBezierSplines> cubic_spline =
			std::make_shared<path_planner::CubicBezierSplines>(this->visu_helper_, this->start_pose_, this->end_pose_);

		// Set the magnitudes as these are settings that affect the splines deeply
		cubic_spline->setStartTangentMagnitude(this->start_tangent_magnitude_);
		cubic_spline->setEndTangentMagnitude(this->end_tangent_magnitude_);

		return cubic_spline;
	}

    std::vector<Eigen::Vector2f> QuinticBezierSplines::calcBezierSpline(int resolution)
    {
		int int_res = int(resolution);
        std::vector<Eigen::Vector2f> bezier_spline;
        for(int counter = 0; counter < int_res; counter++)
        {
			float iterator = (counter == 0) ? 0.0 : (float(counter) / float(resolution));
            bezier_spline.push_back(this->calcPointOnBezierSpline(iterator));
        }

		if(this->next_spline_ == nullptr) // Add point at 1.0 because there is no spline after this that can add the point thorugh its 0.0 value
		{
			bezier_spline.push_back(this->calcPointOnBezierSpline(1.0));
		}

        return bezier_spline;
    }

	Eigen::Vector2f QuinticBezierSplines::calcFirstDerivative(float iterator)
	{
		Eigen::Vector2f first_derivative_value = 5 * std::pow(1 - iterator, 4) * (this->cp1_ - this->start_pose_) +
												 20 * std::pow(1 - iterator, 3) * iterator * (this->cp2_ - this->cp1_) +
												 30 * std::pow(1 - iterator, 2) * std::pow(iterator, 2) * (this->cp3_ - this->cp2_) +
												 20 * (1 - iterator) * std::pow(iterator, 3) * (this->cp4_ - this->cp3_) +
												 5 * std::pow(iterator, 4) * (this->end_pose_ - this->cp4_);

		return first_derivative_value;									   
	}

	Eigen::Vector2f QuinticBezierSplines::calcSecondDerivative(float iterator)
	{
		Eigen::Vector2f second_derivative_value = 20 * std::pow(1 - iterator, 3) * (this->start_pose_ - 2 * this->cp1_ + this->cp2_) +
												  60 * std::pow(1 - iterator, 2) * iterator * (this->cp1_ - 2 * this->cp2_ + this->cp3_) +
												  60 * (1 - iterator) * std::pow(iterator, 2) * (this->cp2_ - 2 * this->cp3_ + this->cp4_) +
												  20 * std::pow(iterator, 3) * (this->cp3_ - 2 * this->cp4_ + this->end_pose_);

		return second_derivative_value;												  
	}

	float QuinticBezierSplines::calcCurvation(float iterator)
	{
		Eigen::Vector2f first_derivative_value = this->calcFirstDerivative(iterator);
		Eigen::Vector2f second_derivative_value = this->calcSecondDerivative(iterator);
		// ROS_INFO_STREAM("first derivative: " << first_derivative_value[0] << "|" << first_derivative_value[1] << " second derivative: " << second_derivative_value[0] << "|" << second_derivative_value[1]);
		// Calculate curvation. See Calculus Early Transcendentals p. 856  for formula
		float curvation_value = std::abs(first_derivative_value[0] * second_derivative_value[1] - first_derivative_value[1] * second_derivative_value[0]) /
								std::pow(std::abs(first_derivative_value.norm()), 3);
		// ROS_INFO_STREAM("curvature: " << curvation_value);								

		return curvation_value;
	}

	float QuinticBezierSplines::calcCurveRadius(float iterator)
	{
		float radius_value = 1 / this->calcCurvation(iterator);
		// ROS_INFO_STREAM("radius: " << radius_value);
		return radius_value;
	}

	bool QuinticBezierSplines::checkMinCurveRadiusAtPoint(float iterator, float min_curve_radius)
	{
		return this->calcCurveRadius(iterator) > min_curve_radius;
	}

	bool QuinticBezierSplines::checkMinCurveRadiusOnSpline(int resolution, float min_curve_radius)
	{
		int point_of_failure_dummy = 0;
		return this->checkMinCurveRadiusOnSpline(resolution, min_curve_radius, point_of_failure_dummy);
	}

	bool QuinticBezierSplines::checkMinCurveRadiusOnSpline(int resolution, float min_curve_radius, int &point_of_failure)
	{
		for(point_of_failure = 0; point_of_failure <= resolution; point_of_failure++)
		{
			// ROS_INFO_STREAM("iterator: " << counter << " | " << float(counter) / float(resolution));
			// Eigen::Vector2f point = this->calcPointOnBezierSpline(float(counter) / float(resolution));
			// ROS_INFO_STREAM("x: " << point[0] << " y: " << point[1]);

			if(!this->checkMinCurveRadiusAtPoint((float(point_of_failure) / float(resolution)), min_curve_radius))
			{
				return false;
			}
		}
		return true;
	}

	void QuinticBezierSplines::printInfo()
	{
		ROS_INFO_STREAM("Start_point: " << this->start_pose_[0] << " | " << this->start_pose_[1]);
		ROS_INFO_STREAM("cp1: " << this->cp1_[0] << " | " << this->cp1_[1]);
		ROS_INFO_STREAM("cp2: " << this->cp2_[0] << " | " << this->cp2_[1]);
		ROS_INFO_STREAM("cp3: " << this->cp3_[0] << " | " << this->cp3_[1]);
		ROS_INFO_STREAM("cp4: " << this->cp4_[0] << " | " << this->cp4_[1]);
		ROS_INFO_STREAM("End_point: " << this->end_pose_[0] << " | " << this->end_pose_[1]);
	}

    void QuinticBezierSplines::initVisuHelper()
    {
        this->start_end_marker_identificator_ = "start_end";
        this->control_point_marker_identificator_ = "control_points";
        this->tangent_marker_identificator_ = "tangents";
        this->bezier_spline_identificator_ = "bezier_spline";

        this->debug_marker_identificator_ = "bezier_debug";

        if(!this->visu_helper_->isMarkerArrayExisting(this->start_end_marker_identificator_))
        {
            this->visu_helper_->addNewMarkerArray(this->start_end_marker_identificator_);
        }
        if(!this->visu_helper_->isMarkerTemplateExisting(this->start_end_marker_identificator_))
        {
            visualization_msgs::Marker marker_template_start_end;
            marker_template_start_end.action = visualization_msgs::Marker::ADD;
            marker_template_start_end.color.a = 1.0;
            marker_template_start_end.color.g = 1.0;
            marker_template_start_end.header.frame_id = "map";
            marker_template_start_end.ns = "start_end";
            marker_template_start_end.scale.x = 0.1;
            marker_template_start_end.scale.y = 0.1;
            marker_template_start_end.scale.z = 0.1;
            marker_template_start_end.type = visualization_msgs::Marker::SPHERE;
            this->visu_helper_->addMarkerTemplate(this->start_end_marker_identificator_, marker_template_start_end);
        }

        if(!this->visu_helper_->isMarkerArrayExisting(this->control_point_marker_identificator_))
        {
            this->visu_helper_->addNewMarkerArray(this->control_point_marker_identificator_);
        }
        if(!this->visu_helper_->isMarkerTemplateExisting(this->control_point_marker_identificator_))
        {
            visualization_msgs::Marker marker_template_control_point;
            marker_template_control_point.action = visualization_msgs::Marker::ADD;
            marker_template_control_point.color.a = 1.0;
            marker_template_control_point.color.b = 1.0;
            marker_template_control_point.header.frame_id = "map";
            marker_template_control_point.ns = "control_points";
            marker_template_control_point.scale.x = 0.1;
            marker_template_control_point.scale.y = 0.1;
            marker_template_control_point.scale.z = 0.1;
            marker_template_control_point.type = visualization_msgs::Marker::SPHERE;
            this->visu_helper_->addMarkerTemplate(this->control_point_marker_identificator_, marker_template_control_point);
        }

        if(!this->visu_helper_->isMarkerArrayExisting(this->tangent_marker_identificator_))
        {
            this->visu_helper_->addNewMarkerArray(this->tangent_marker_identificator_);
        }
        if(!this->visu_helper_->isMarkerTemplateExisting(this->tangent_marker_identificator_))
        {
            visualization_msgs::Marker marker_template_tangents;
            marker_template_tangents.action = visualization_msgs::Marker::ADD;
            marker_template_tangents.color.a = 1.0;
            marker_template_tangents.color.b = 1.0;
            marker_template_tangents.color.g = 1.0;
            marker_template_tangents.header.frame_id = "map";
            marker_template_tangents.ns = "tangents";
            marker_template_tangents.scale.x = 0.03;
            marker_template_tangents.type = visualization_msgs::Marker::LINE_LIST;
            this->visu_helper_->addMarkerTemplate(this->tangent_marker_identificator_, marker_template_tangents);
        }

        if(!this->visu_helper_->isMarkerArrayExisting(this->bezier_spline_identificator_))
        {
            this->visu_helper_->addNewMarkerArray(this->bezier_spline_identificator_);
        }
        if(!this->visu_helper_->isMarkerTemplateExisting(this->bezier_spline_identificator_))
        {
            visualization_msgs::Marker bezier_spline_template;
            bezier_spline_template.action = visualization_msgs::Marker::ADD;
            bezier_spline_template.color.a = 1.0;
            bezier_spline_template.color.r = 1.0;
            bezier_spline_template.header.frame_id = "map";
            bezier_spline_template.ns = "bezier_spline";
            bezier_spline_template.scale.x = 0.03;
            bezier_spline_template.type = visualization_msgs::Marker::LINE_STRIP;
            this->visu_helper_->addMarkerTemplate(this->bezier_spline_identificator_, bezier_spline_template);
        }

        if(!this->visu_helper_->isMarkerArrayExisting(this->debug_marker_identificator_))
        {
            this->visu_helper_->addNewMarkerArray(this->debug_marker_identificator_);
        }
        if(!this->visu_helper_->isMarkerTemplateExisting(this->debug_marker_identificator_))
        {
            visualization_msgs::Marker marker_template_debug;
            marker_template_debug.action = visualization_msgs::Marker::ADD;
            marker_template_debug.color.a = 1.0;
            marker_template_debug.color.r = 1.0;
            marker_template_debug.color.b = 1.0;
            marker_template_debug.header.frame_id = "map";
            marker_template_debug.ns = "bezier_debug";
            marker_template_debug.scale.x = 0.05;
            marker_template_debug.type = visualization_msgs::Marker::LINE_LIST;
            this->visu_helper_->addMarkerTemplate(this->debug_marker_identificator_, marker_template_debug);
        }
    }

    void QuinticBezierSplines::addStartEndPointToVisuHelper()
    {
        this->visu_helper_->addMarkerToExistingMarkerArray(this->start_end_marker_identificator_,
                                                           this->visu_helper_->createGeometryPose(this->start_pose_[0], this->start_pose_[1]),
                                                           this->start_end_marker_identificator_);
        this->visu_helper_->addMarkerToExistingMarkerArray(this->start_end_marker_identificator_,
                                                           this->visu_helper_->createGeometryPose(this->end_pose_[0], this->end_pose_[1]),
                                                           this->start_end_marker_identificator_);                                                        
    }

    void QuinticBezierSplines::addControlPointsToVisuHelper()
    {
        this->visu_helper_->addMarkerToExistingMarkerArray(this->control_point_marker_identificator_,
                                                           this->visu_helper_->createGeometryPose(this->cp1_[0], this->cp1_[1]),
                                                           this->control_point_marker_identificator_);
		this->visu_helper_->addMarkerToExistingMarkerArray(this->control_point_marker_identificator_,
                                                           this->visu_helper_->createGeometryPose(this->cp2_[0], this->cp2_[1]),
                                                           this->control_point_marker_identificator_);
		this->visu_helper_->addMarkerToExistingMarkerArray(this->control_point_marker_identificator_,
                                                           this->visu_helper_->createGeometryPose(this->cp3_[0], this->cp3_[1]),
                                                           this->control_point_marker_identificator_);
        this->visu_helper_->addMarkerToExistingMarkerArray(this->control_point_marker_identificator_,
                                                           this->visu_helper_->createGeometryPose(this->cp4_[0], this->cp4_[1]),
                                                           this->control_point_marker_identificator_);     
    }

    void QuinticBezierSplines::addTangentsToVisuHelper()
    {
        this->addTangentToVisuHelper(this->start_pose_, this->start_tangent_);
        this->addTangentToVisuHelper(this->end_pose_, this->end_tangent_);
    }

    void QuinticBezierSplines::addTangentToVisuHelper(Eigen::Vector2f start_point, Eigen::Vector2f tangent)
    {
        std::vector<geometry_msgs::Point> line;
        Eigen::Vector2f end_point;
        end_point = start_point + tangent;

        line.push_back(this->visu_helper_->createGeometryPoint(start_point[0], start_point[1]));
        line.push_back(this->visu_helper_->createGeometryPoint(end_point[0], end_point[1]));

        this->visu_helper_->addMarkerLineToMarkerArray(this->tangent_marker_identificator_,
                                                       line,
                                                       this->tangent_marker_identificator_);
    }

    void QuinticBezierSplines::addBezierSplineToVisuHelper(int resolution)
    {
        std::vector<Eigen::Vector2f> bezier_spline;
        std::vector<geometry_msgs::Point> line;

        bezier_spline = this->calcBezierSpline(resolution);

        for(Eigen::Vector2f point_on_spline: bezier_spline)
        {
            line.push_back(this->visu_helper_->createGeometryPoint(point_on_spline[0], point_on_spline[1]));
        }

        this->visu_helper_->addMarkerLineToMarkerArray(this->bezier_spline_identificator_,
                                                       line, 
                                                       this->bezier_spline_identificator_);
    }

    void QuinticBezierSplines::addDebugVectorToVisuHelper(Eigen::Vector2f start_point, Eigen::Vector2f vector)
    {
        std::vector<geometry_msgs::Point> line;
        Eigen::Vector2f end_point;
        end_point = start_point + vector;

        line.push_back(this->visu_helper_->createGeometryPoint(start_point[0], start_point[1]));
        line.push_back(this->visu_helper_->createGeometryPoint(end_point[0], end_point[1]));
        this->visu_helper_->addMarkerLineToMarkerArray(this->debug_marker_identificator_,
                                                       line,
                                                       this->debug_marker_identificator_);
    }

	long QuinticBezierSplines::calcFactorial(long n)
	{
		if(n == 0)
		{
			return 1;
		}

		long result = 1;
		for(long counter = 1; counter <= n; counter++)
		{
			result = result * counter;
		}
		return result;
	}

	long QuinticBezierSplines::calcBinomialCoefficient(long n, long k)
	{
		return (this->calcFactorial(n)) / (this->calcFactorial(k) * this->calcFactorial(n-k));
	}

}