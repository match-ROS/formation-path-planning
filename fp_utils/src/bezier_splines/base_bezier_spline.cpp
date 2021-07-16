#include <fp_utils/bezier_splines/base_bezier_spline.h>

namespace bezier_splines
{
	#pragma region Constructors

	BaseBezierSpline::BaseBezierSpline(int bezier_degree)
		: visu_helper_(nullptr),
		  BEZIER_DEGREE(bezier_degree),
		  previous_spline_(nullptr),
		  next_spline_(nullptr),
		  start_tangent_magnitude_(0.25),
		  end_tangent_magnitude_(0.25)
	{
		this->initControlPointList();
	}

	BaseBezierSpline::BaseBezierSpline(int bezier_degree,
									   visualization_helper::VisualizationHelper *visu_helper)
		: BaseBezierSpline(bezier_degree)
	{
		this->visu_helper_ = visu_helper;
		this->initVisuHelper();
	}

	BaseBezierSpline::BaseBezierSpline(int bezier_degree,
									   Eigen::Vector2f start_pose,
									   Eigen::Vector2f end_pose)
		: BaseBezierSpline(bezier_degree)
	{
		this->control_points_.front() = start_pose;
		this->control_points_.back() = end_pose;
	}

	BaseBezierSpline::BaseBezierSpline(int bezier_degree,
									   visualization_helper::VisualizationHelper *visu_helper,
									   Eigen::Vector2f start_pose,
									   Eigen::Vector2f end_pose)
		: BaseBezierSpline(bezier_degree, start_pose, end_pose)
	{
		this->visu_helper_ = visu_helper;
		this->initVisuHelper();
	}

	BaseBezierSpline::BaseBezierSpline(int bezier_degree,
									   Eigen::Vector2f start_pose,
									   Eigen::Vector2f start_tangent,
									   float start_tangent_magnitude,
									   Eigen::Vector2f end_pose,
									   Eigen::Vector2f end_tangent,
									   float end_tangent_magnitude)
		: BaseBezierSpline(bezier_degree, start_pose, end_pose)
	{
		this->start_tangent_ = start_tangent;
		this->start_tangent_magnitude_ = start_tangent_magnitude;
		this->end_tangent_ = end_tangent;
		this->end_tangent_magnitude_ = end_tangent_magnitude;
	}

	BaseBezierSpline::BaseBezierSpline(int bezier_degree,
									   visualization_helper::VisualizationHelper *visu_helper,
									   Eigen::Vector2f start_pose,
									   Eigen::Vector2f start_tangent,
									   float start_tangent_magnitude,
									   Eigen::Vector2f end_pose,
									   Eigen::Vector2f end_tangent,
									   float end_tangent_magnitude)
		: BaseBezierSpline(bezier_degree,
						   start_pose,
						   start_tangent,
						   start_tangent_magnitude,
						   end_pose,
						   end_tangent,
						   end_tangent_magnitude)
	{
		this->visu_helper_ = visu_helper;
		this->initVisuHelper();
	}
	#pragma endregion


	#pragma region Getter/Setter
	Eigen::Vector2f BaseBezierSpline::getStartPose()
    {
        return this->control_points_.front();
    }

    Eigen::Vector2f BaseBezierSpline::getEndPose()
    {
        return this->control_points_.back();
    }

	Eigen::Vector2f BaseBezierSpline::getStartTangent()
    {
        return this->start_tangent_;
    }

	void BaseBezierSpline::setStartTangent(Eigen::Vector2f start_pose_tangent)
    {
        this->start_tangent_ = start_pose_tangent;
    }

    Eigen::Vector2f BaseBezierSpline::getEndTangent()
    {
        return this->end_tangent_;
    }

	void BaseBezierSpline::setEndTangent(Eigen::Vector2f end_tangent)
	{
		this->end_tangent_ = end_tangent;
	}

	void BaseBezierSpline::setStartTangentMagnitude(float start_tangent_magnitude)
	{
		this->start_tangent_magnitude_ = start_tangent_magnitude;
	}
	
	float BaseBezierSpline::getStartTangentMagnitude()
	{
		return this->start_tangent_magnitude_;
	}

	void BaseBezierSpline::setEndTangentMagnitude(float end_tangent_magnitude)
	{
		this->end_tangent_magnitude_ = end_tangent_magnitude;
	}

	float BaseBezierSpline::getEndTangentMagnitude()
	{
		return this->end_tangent_magnitude_;
	}

	Eigen::Vector2f BaseBezierSpline::getMultipliedStartTangent()
	{
		return this->start_tangent_magnitude_ * this->start_tangent_;
	}

	Eigen::Vector2f BaseBezierSpline::getMultipliedEndTangent()
	{
		return this->end_tangent_magnitude_ * this->end_tangent_;
	}
	#pragma endregion


	#pragma region DataManagement
	void BaseBezierSpline::setPreviousSpline(const std::shared_ptr<BaseBezierSpline> &previous_spline)
	{
		this->previous_spline_ = previous_spline;
		this->setStartTangent(this->previous_spline_->getEndTangent());
		this->setStartTangentMagnitude(this->previous_spline_->getEndTangentMagnitude());
	}

	void BaseBezierSpline::setNextSpline(const std::shared_ptr<BaseBezierSpline> &next_spline)
	{
		this->next_spline_ = next_spline;
		this->setEndTangentByNextPose(this->next_spline_->getEndPose());
	}
	#pragma endregion


	#pragma region BezierMethods
	void BaseBezierSpline::setStartTangentByQuaternion(tf::Quaternion robot_orientation)
    {
        float start_to_end_length = this->calcStartToEndLength();

        tf::Vector3 direction_vector(1, 0, 0);
        tf::Vector3 rotated_vector = tf::quatRotate(robot_orientation, direction_vector);
        rotated_vector = rotated_vector * start_to_end_length;
        this->start_tangent_ << rotated_vector[0], rotated_vector[1];
    }

    void BaseBezierSpline::setEndTangentByQuaternion(tf::Quaternion robot_end_orientation)
    {
        float start_to_end_length = this->calcStartToEndLength();

        tf::Vector3 direction_vector(1, 0, 0);
        tf::Vector3 rotated_vector = tf::quatRotate(robot_end_orientation, direction_vector);
        rotated_vector = rotated_vector * start_to_end_length;
        this->end_tangent_ << rotated_vector[0], rotated_vector[1];
    }

    void BaseBezierSpline::setEndTangentByNextPose(Eigen::Vector2f next_pose)
    {
        Eigen::Vector2f diff_vector_start_to_end;
        Eigen::Vector2f diff_vector_end_to_next;

        diff_vector_start_to_end = this->control_points_.back() - this->control_points_.front();
        diff_vector_end_to_next = next_pose - this->control_points_.back();

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

	std::vector<Eigen::Vector2f> BaseBezierSpline::calcBezierSpline(int resolution)
    {
        std::vector<Eigen::Vector2f> bezier_spline;
        for(int counter = 0; counter < resolution; counter++)
        {
			float iterator = (counter == 0) ? 0.0 : (float(counter) / float(resolution)); // inline if necessary?
            bezier_spline.push_back(this->calcPointOnBezierSpline(iterator));
        }

		if(this->next_spline_ == nullptr) // Add point at 1.0 because there is no spline after this that can add the point thorugh its 0.0 value
		{
			bezier_spline.push_back(this->calcPointOnBezierSpline(1.0));
		}

        return bezier_spline;
    }

	float BaseBezierSpline::calcSplineLength(float lower_bound, float upper_bound, float max_step_size)
	{
		int number_of_steps = (upper_bound - lower_bound) / max_step_size;
		float real_step_size = (upper_bound - lower_bound) / float(number_of_steps);

		float approx_spline_length = 0.0;

		for(float step_counter = 0; step_counter < number_of_steps; step_counter++)
		{
			// Eigen::Vector2f lower_point = this->calcPointOnBezierSpline(lower_bound + float(step_counter) * real_step_size);
			// Eigen::Vector2f upper_point = this->calcPointOnBezierSpline(lower_bound + float(step_counter + 1) * real_step_size);

			// Eigen::Vector2f diff = upper_point - lower_point;
			// approx_spline_length = approx_spline_length + diff.norm();

			Eigen::Vector2f gradient_value = this->calcFirstDerivativeValue(lower_bound + float(step_counter) * real_step_size);
			approx_spline_length = approx_spline_length + (gradient_value * real_step_size).norm();
		}

		return approx_spline_length;
	}

	bool BaseBezierSpline::calcIteratorBySplineLength(float &iterator,
													  float target_spline_length,
													  float max_diff_from_target,
													  float max_step_size,
													  float &spline_length_remainder)
	{
		return this->calcIteratorBySplineLength(iterator, target_spline_length, max_diff_from_target, 0.0, max_step_size, spline_length_remainder);
	}

	bool BaseBezierSpline::calcIteratorBySplineLength(float &iterator,
													  float target_spline_length,
													  float max_diff_from_target,
													  float first_step_size,
													  float max_step_size,
													  float &spline_length_remainder)
	{
		float start_iterator = iterator;

		// The first step can help to speed up the process as the iterator not always has to start at 0.0
		if(first_step_size == 0.0)
		{
			iterator = iterator + max_step_size;	
		}
		else
		{
			iterator = iterator + first_step_size;
		}		

		float approx_spline_length = this->calcSplineLength(start_iterator, iterator, max_step_size);

		// Factor that will decrease the backtracking by half each time a step is performed
		float max_step_size_factor = 0.5;
		// The backtracking steps will be reduced only if the algorithm increased the iterator at least once
		bool iterator_increased_once = false;

		do
		{
			if(approx_spline_length < target_spline_length)
			{
				iterator = iterator + max_step_size;
				// Reset the factor for the next iteration where approx_spline_length is bigger than target_spline_length
				max_step_size_factor = 0.5;
				iterator_increased_once = true;
			}
			else if(approx_spline_length > target_spline_length)
			{
				iterator = iterator - max_step_size_factor * max_step_size;
				// Lower max_step_size_factor for next iteration to get even closer to the target_spline_length
				if(iterator_increased_once)
				{
					max_step_size_factor = max_step_size_factor * 0.5;
				}
			}
			
			approx_spline_length = this->calcSplineLength(start_iterator, iterator, max_step_size);

			if(iterator > 1.0)
			{
				if((target_spline_length - approx_spline_length) < 0.0)
				{
					// Iterator is too high and current spline can not offer the target_spline_length.
					// Calculate until max value of 1.0 and return missing spline length
					approx_spline_length = this->calcSplineLength(start_iterator, 1.0, max_step_size);
					spline_length_remainder = target_spline_length - approx_spline_length;
					return false;	
				}				
			}
		} while(std::abs(approx_spline_length - target_spline_length) > max_diff_from_target || iterator > 1.0);

		// target_spline_length was reached so spline is not finished
		spline_length_remainder = 0.0;
		return true;
	}

	float BaseBezierSpline::calcCurvation(float iterator)
	{
		Eigen::Vector2f first_derivative_value = this->calcFirstDerivativeValue(iterator);
		Eigen::Vector2f second_derivative_value = this->calcSecondDerivativeValue(iterator);
		// ROS_INFO_STREAM("first derivative: " << first_derivative_value[0] << "|" << first_derivative_value[1] << " second derivative: " << second_derivative_value[0] << "|" << second_derivative_value[1]);
		// Calculate curvation. See Calculus Early Transcendentals p. 856  for formula
		float curvation_value = std::abs(first_derivative_value[0] * second_derivative_value[1] -
										 first_derivative_value[1] * second_derivative_value[0]) /
								std::pow(std::abs(first_derivative_value.norm()), 3);
		// ROS_INFO_STREAM("curvature: " << curvation_value);								

		return curvation_value;
	}

	float BaseBezierSpline::calcCurveRadius(float iterator)
	{
		float radius_value = 1 / this->calcCurvation(iterator);
		// ROS_INFO_STREAM("radius: " << radius_value);
		return radius_value;
	}

	bool BaseBezierSpline::checkMinCurveRadiusAtPoint(float iterator, float min_curve_radius)
	{
		return this->calcCurveRadius(iterator) > min_curve_radius;
	}

	bool BaseBezierSpline::checkMinCurveRadiusOnSpline(int resolution, float min_curve_radius)
	{
		int point_of_failure_dummy = 0;
		return this->checkMinCurveRadiusOnSpline(resolution, min_curve_radius, point_of_failure_dummy);
	}

	bool BaseBezierSpline::checkMinCurveRadiusOnSpline(int resolution, float min_curve_radius, int &point_of_failure)
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
	#pragma endregion


	#pragma region PublicVisuHelper
	void BaseBezierSpline::visualizeData()
    {
		if(this->isVisuHelperNull())
		{
			return;
		}

		this->visu_helper_->visualizeMarkerArray(this->start_end_marker_identificator_);
		this->visu_helper_->visualizeMarkerArray(this->control_point_marker_identificator_);
		this->visu_helper_->visualizeMarkerArray(this->tangent_marker_identificator_);
		this->visu_helper_->visualizeMarkerArray(this->bezier_spline_identificator_);
		// this->visu_helper_->visualizeMarkerArray(this->debug_marker_identificator_);

		ros::Duration(0.1).sleep(); // Wait for markers to be shown, maybe this helps to visualize them every time
    }

	void BaseBezierSpline::addStartEndPointToVisuHelper()
    {
		if(this->isVisuHelperNull())
		{
			return;
		}

        this->visu_helper_->addMarkerToExistingMarkerArray(this->start_end_marker_identificator_,
                                                           this->visu_helper_->createGeometryPose(this->control_points_.front()[0], this->control_points_.front()[1]),
                                                           this->start_end_marker_identificator_);
        this->visu_helper_->addMarkerToExistingMarkerArray(this->start_end_marker_identificator_,
                                                           this->visu_helper_->createGeometryPose(this->control_points_.back()[0], this->control_points_.back()[1]),
                                                           this->start_end_marker_identificator_);                                                        
    }

    void BaseBezierSpline::addControlPointsToVisuHelper()
    {
		if(this->isVisuHelperNull())
		{
			return;
		}

		for(Eigen::Vector2f control_point: this->control_points_)
		{
			this->visu_helper_->addMarkerToExistingMarkerArray(this->control_point_marker_identificator_,
															   this->visu_helper_->createGeometryPose(control_point[0], control_point[1]),
															   this->control_point_marker_identificator_);
			this->visu_helper_->addMarkerToExistingMarkerArray(this->control_point_marker_identificator_,
															   this->visu_helper_->createGeometryPose(control_point[0], control_point[1]),
															   this->control_point_marker_identificator_);
		}
    }

	void BaseBezierSpline::addBezierSplineToVisuHelper(int resolution)
    {
		if(this->isVisuHelperNull())
		{
			return;
		}

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

    void BaseBezierSpline::addTangentsToVisuHelper()
    {
		if(this->isVisuHelperNull())
		{
			return;
		}

        this->addTangentToVisuHelper(this->control_points_.front(), this->getMultipliedStartTangent());
        this->addTangentToVisuHelper(this->control_points_.back(), this->getMultipliedEndTangent());
    }

	void BaseBezierSpline::printInfo()
	{
		ROS_INFO_STREAM("Start_point: " << this->control_points_.front()[0] << " | " << this->control_points_.front()[1]);

		// From one to size-1 to not print this line for the start and end point
		for(int control_point_counter = 1; control_point_counter < (this->control_points_.size() - 1); control_point_counter++)
		{
			ROS_INFO_STREAM("ControlPoint" << control_point_counter << ": " << this->control_points_[control_point_counter][0] << " | " << this->control_points_[control_point_counter][1]);
		}

		ROS_INFO_STREAM("End_point: " << this->control_points_.back()[0] << " | " << this->control_points_.back()[1]);
	}
	#pragma endregion

	void BaseBezierSpline::initControlPointList()
	{
		for(int control_point_counter = 0; control_point_counter <= this->BEZIER_DEGREE; control_point_counter++)
		{
			this->control_points_.push_back(Eigen::Vector2f::Zero());
		}
	}

	#pragma region MathHelper
	float BaseBezierSpline::calcStartToEndLength()
	{
		return std::sqrt(std::pow((this->control_points_.back()[0] - this->control_points_.front()[0]), 2) +
						 std::pow((this->control_points_.back()[1] - this->control_points_.front()[1]), 2));
	}

	long BaseBezierSpline::calcFactorial(long n)
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

	long BaseBezierSpline::calcBinomialCoefficient(long n, long k)
	{
		return (this->calcFactorial(n)) / (this->calcFactorial(k) * this->calcFactorial(n-k));
	}
	#pragma endregion



	#pragma region PrivateVisuHelper
	void BaseBezierSpline::initVisuHelper()
	{
		if(this->isVisuHelperNull())
		{
			return;
		}

		this->initVisuHelper("start_end",
							 "control_points",
							 "tangents",
							 "bezier_spline",
							 "bezier_debug");
	}

	void BaseBezierSpline::initVisuHelper(std::string start_end_marker_identificator,
										  std::string control_point_marker_identificator,
										  std::string tangent_marker_identificator,
										  std::string bezier_spline_identificator,
										  std::string debug_marker_identificator)
	{
		if(this->isVisuHelperNull())
		{
			return;
		}

		this->start_end_marker_identificator_ = start_end_marker_identificator;
		this->control_point_marker_identificator_ = control_point_marker_identificator;
		this->tangent_marker_identificator_ = tangent_marker_identificator;
		this->bezier_spline_identificator_ = bezier_spline_identificator;
		this->debug_marker_identificator_ = debug_marker_identificator;

		if (!this->visu_helper_->isMarkerArrayExisting(this->start_end_marker_identificator_))
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
            marker_template_start_end.ns = this->start_end_marker_identificator_;
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
            marker_template_control_point.ns = this->control_point_marker_identificator_;
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
            marker_template_tangents.ns = this->tangent_marker_identificator_;
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
            bezier_spline_template.ns = this->bezier_spline_identificator_;
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
            marker_template_debug.ns = this->debug_marker_identificator_;
            marker_template_debug.scale.x = 0.05;
            marker_template_debug.type = visualization_msgs::Marker::LINE_LIST;
            this->visu_helper_->addMarkerTemplate(this->debug_marker_identificator_, marker_template_debug);
        }
	}

	void BaseBezierSpline::addTangentToVisuHelper(Eigen::Vector2f start_point,
												  Eigen::Vector2f tangent)
	{
		if(this->isVisuHelperNull())
		{
			return;
		}

        std::vector<geometry_msgs::Point> line;
        Eigen::Vector2f end_point;
        end_point = start_point + tangent;

        line.push_back(this->visu_helper_->createGeometryPoint(start_point[0], start_point[1]));
        line.push_back(this->visu_helper_->createGeometryPoint(end_point[0], end_point[1]));

        this->visu_helper_->addMarkerLineToMarkerArray(this->tangent_marker_identificator_,
                                                       line,
                                                       this->tangent_marker_identificator_);
    }

	void BaseBezierSpline::addDebugVectorToVisuHelper(Eigen::Vector2f start_point,
													  Eigen::Vector2f vector)
	{
		if(this->isVisuHelperNull())
		{
			return;
		}

        std::vector<geometry_msgs::Point> line;
        Eigen::Vector2f end_point;
        end_point = start_point + vector;

        line.push_back(this->visu_helper_->createGeometryPoint(start_point[0], start_point[1]));
        line.push_back(this->visu_helper_->createGeometryPoint(end_point[0], end_point[1]));
        this->visu_helper_->addMarkerLineToMarkerArray(this->debug_marker_identificator_,
                                                       line,
                                                       this->debug_marker_identificator_);
    }

	bool BaseBezierSpline::isVisuHelperNull()
	{
		if(this->visu_helper_ == nullptr)
		{
			return true;
		}
		return false;
	}
	#pragma endregion
}