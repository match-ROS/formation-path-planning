#include <fp_utils/bezier_splines/base_bezier_spline.h>

namespace bezier_splines
{
	#pragma region Constructors

	BaseBezierSpline::BaseBezierSpline()
		: visu_helper_(nullptr)
	{

	}

	BaseBezierSpline::BaseBezierSpline(visualization_helper::VisualizationHelper *visu_helper)
		: BaseBezierSpline()
	{
		this->visu_helper_ = visu_helper_;
	}

	BaseBezierSpline::BaseBezierSpline(Eigen::Vector2f start_pose,
									   Eigen::Vector2f end_pose)
		: BaseBezierSpline()
	{
		this->start_pose_ = start_pose;
		this->end_pose_ = end_pose;
	}

	BaseBezierSpline::BaseBezierSpline(visualization_helper::VisualizationHelper *visu_helper,
									   Eigen::Vector2f start_pose,
									   Eigen::Vector2f end_pose)
		: BaseBezierSpline(start_pose, end_pose)
	{
		this->visu_helper_ = visu_helper_;
	}

	BaseBezierSpline::BaseBezierSpline(Eigen::Vector2f start_pose,
									   Eigen::Vector2f start_tangent,
									   float start_tangent_magnitude,
									   Eigen::Vector2f end_pose,
									   Eigen::Vector2f end_tangent,
									   float end_tangent_magnitude)
		: BaseBezierSpline(start_pose, end_pose)
	{
		this->start_tangent_ = start_tangent;
		this->start_tangent_magnitude_ = start_tangent_magnitude;
		this->end_tangent_ = end_tangent;
		this->end_tangent_magnitude_ = end_tangent_magnitude;
	}

	BaseBezierSpline::BaseBezierSpline(visualization_helper::VisualizationHelper *visu_helper,
									   Eigen::Vector2f start_pose,
									   Eigen::Vector2f start_tangent,
									   float start_tangent_magnitude,
									   Eigen::Vector2f end_pose,
									   Eigen::Vector2f end_tangent,
									   float end_tangent_magnitude)
		: BaseBezierSpline(start_pose,
						   start_tangent,
						   start_tangent_magnitude,
						   end_pose,
						   end_tangent,
						   end_tangent_magnitude)
	{
		this->visu_helper_ = visu_helper;
	}
	#pragma endregion


	#pragma region Getter/Setter
	Eigen::Vector2f BaseBezierSpline::getStartPose()
    {
        return this->start_pose_;
    }

    Eigen::Vector2f BaseBezierSpline::getEndPose()
    {
        return this->end_pose_;
    }

	Eigen::Vector2f BaseBezierSpline::getStartTangent()
    {
        return this->start_tangent_;
    }

	void BaseBezierSpline::setStartTangent(Eigen::Matrix<float, 2, 1> start_pose_tangent)
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

	float BaseBezierSpline::getStartTangentMagnitude()
	{
		return this->start_tangent_magnitude_;
	}

	void BaseBezierSpline::setStartTangentMagnitude(float start_tangent_magnitude)
	{
		this->start_tangent_magnitude_ = start_tangent_magnitude;
	}

	float BaseBezierSpline::getEndTangentMagnitude()
	{
		return this->end_tangent_magnitude_;
	}

	void BaseBezierSpline::setEndTangentMagnitude(float end_tangent_magnitude)
	{
		this->end_tangent_magnitude_ = end_tangent_magnitude;
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


	#pragma region PublicVisuHelper
	void BaseBezierSpline::visualizeData()
    {
        this->visu_helper_->visualizeMarkerArray(this->start_end_marker_identificator_);
        this->visu_helper_->visualizeMarkerArray(this->control_point_marker_identificator_);
        this->visu_helper_->visualizeMarkerArray(this->tangent_marker_identificator_);
        this->visu_helper_->visualizeMarkerArray(this->bezier_spline_identificator_);
        // this->visu_helper_->visualizeMarkerArray(this->debug_marker_identificator_);

        ros::Duration(0.1).sleep(); // Wait for markers to be shown, maybe this helps to visualize them every time
    }

	void BaseBezierSpline::addStartEndPointToVisuHelper()
    {
        this->visu_helper_->addMarkerToExistingMarkerArray(this->start_end_marker_identificator_,
                                                           this->visu_helper_->createGeometryPose(this->start_pose_[0], this->start_pose_[1]),
                                                           this->start_end_marker_identificator_);
        this->visu_helper_->addMarkerToExistingMarkerArray(this->start_end_marker_identificator_,
                                                           this->visu_helper_->createGeometryPose(this->end_pose_[0], this->end_pose_[1]),
                                                           this->start_end_marker_identificator_);                                                        
    }

    void BaseBezierSpline::addControlPointsToVisuHelper()
    {
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
        std::vector<Eigen::Matrix<float, 2, 1>> bezier_spline;
        std::vector<geometry_msgs::Point> line;

        bezier_spline = this->calcBezierSpline(resolution);

        for(Eigen::Matrix<float, 2, 1> point_on_spline: bezier_spline)
        {
            line.push_back(this->visu_helper_->createGeometryPoint(point_on_spline[0], point_on_spline[1]));
        }

        this->visu_helper_->addMarkerLineToMarkerArray(this->bezier_spline_identificator_,
                                                       line, 
                                                       this->bezier_spline_identificator_);
    }

    void BaseBezierSpline::addTangentsToVisuHelper()
    {
        this->addTangentToVisuHelper(this->start_pose_, this->getMultipliedStartTangent());
        this->addTangentToVisuHelper(this->end_pose_, this->getMultipliedEndTangent());
    }
	#pragma endregion



	#pragma region MathHelper
	float BaseBezierSpline::calcStartToEndLength()
	{
		return std::sqrt(std::pow((this->end_pose_[0] - this->start_pose_[0]), 2) +
						 std::pow((this->end_pose_[1] - this->start_pose_[1]), 2));
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

	void BaseBezierSpline::addTangentToVisuHelper(Eigen::Vector2f start_point,
												  Eigen::Vector2f tangent)
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

	void BaseBezierSpline::addDebugVectorToVisuHelper(Eigen::Vector2f start_point,
													  Eigen::Vector2f vector)
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
	#pragma endregion
}