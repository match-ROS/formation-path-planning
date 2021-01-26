#include <fpp_ros/path_planner/cubic_bezier_splines.h>

namespace path_planner
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

    void CubicBezierSplines::setStartTangent(tf::Quaternion robot_orientation)
    {
        float start_to_end_length = this->calcStartToEndLength();

        tf::Vector3 direction_vector(1, 0, 0);
        tf::Vector3 rotated_vector = tf::quatRotate(robot_orientation, direction_vector);
        rotated_vector = rotated_vector * start_to_end_length;
        this->start_pose_tangent_ << rotated_vector[0], rotated_vector[1];
    }

    void CubicBezierSplines::setStartTangent(Eigen::Matrix<float, 2, 1> start_pose_tangent)
    {
        this->start_pose_tangent_ = start_pose_tangent;
    }

    void CubicBezierSplines::setEndTangent(tf::Quaternion robot_end_orientation)
    {
        float start_to_end_length = this->calcStartToEndLength();

        tf::Vector3 direction_vector(1, 0, 0);
        tf::Vector3 rotated_vector = tf::quatRotate(robot_end_orientation, direction_vector);
        rotated_vector = rotated_vector * start_to_end_length;
        this->end_pose_tangent_ << rotated_vector[0], rotated_vector[1];
    }

    void CubicBezierSplines::setEndTangent(Eigen::Matrix<float, 2, 1> next_pose)
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
        float length_diff_vector_end_to_next = angular_bisector.norm();
        this->end_pose_tangent_ = 2.0 * length_diff_vector_end_to_next * angular_bisector; // This 0.5 value is taken from the paper
    }

    void CubicBezierSplines::visualizeData()
    {
        this->visu_helper_->visualizeMarkerArray(this->start_end_marker_identificator_);
        this->visu_helper_->visualizeMarkerArray(this->control_point_marker_identificator_);
        this->visu_helper_->visualizeMarkerArray(this->tangent_marker_identificator_);
        this->visu_helper_->visualizeMarkerArray(this->bezier_spline_identificator_);
        // this->visu_helper_->visualizeMarkerArray(this->debug_marker_identificator_);

        ros::Duration(0.2).sleep(); // Wait for markers to be shown, maybe this helps to visualize them every time
    }

    Eigen::Matrix<float, 2, 1> CubicBezierSplines::getStartPose()
    {
        return this->start_pose_;
    }

    Eigen::Matrix<float, 2, 1> CubicBezierSplines::getEndPose()
    {
        return this->end_pose_;
    }

    Eigen::Matrix<float, 2, 1> CubicBezierSplines::getStartTangent()
    {
        return this->start_pose_tangent_;
    }

    Eigen::Matrix<float, 2, 1> CubicBezierSplines::getEndTangent()
    {
        return this->end_pose_tangent_;
    }




    float CubicBezierSplines::calcStartToEndLength()
    {
        return std::sqrt(std::pow((this->end_pose_[0] - this->start_pose_[0]), 2) + std::pow((this->end_pose_[1] - this->start_pose_[1]), 2));
    }

    void CubicBezierSplines::calcControlPoints()
    {
        this->cp1_ = (1.0 / 3.0) * this->start_pose_tangent_ + this->start_pose_;
        this->cp2_ = -(1.0 / 3.0) * this->end_pose_tangent_ + this->end_pose_;
    }

    Eigen::Matrix<float, 2, 1> CubicBezierSplines::calcPointOnBezierSpline(float iterator)
    {
        Eigen::Matrix<float, 4, 4> bezier_basis_matrix;
        Eigen::Matrix<float, 1, 4> iterator_matrix;
        
        bezier_basis_matrix << -1, 3, -3, 1, 3, -6, 3, 0, -3, 3, 0, 0, 1, 0, 0, 0;
        iterator_matrix << std::pow(iterator, 3), std::pow(iterator, 2), iterator, 1;

        Eigen::Matrix<Eigen::Matrix<float, 2, 1>, 4, 1> point_matrix;
        point_matrix[0] = this->start_pose_;
        point_matrix[1] = this->cp1_;
        point_matrix[2] = this->cp2_;
        point_matrix[3] = this->end_pose_;

        Eigen::Matrix<float, 4, 1> matrix_multi = iterator_matrix * bezier_basis_matrix;
        Eigen::Matrix<float , 2, 1> result_vector;
        result_vector << 0, 0;
        for(int counter = 0; counter <= 3; counter++)
        {
            result_vector = result_vector + (matrix_multi[counter] * point_matrix[counter]);
        }
        return result_vector;
    }

    std::vector<Eigen::Matrix<float, 2, 1>> CubicBezierSplines::calcBezierSpline(float resolution)
    {
        std::vector<Eigen::Matrix<float, 2, 1>> bezier_spline;
        for(float counter = 0; counter <= 1.0; counter = counter + resolution)
        {
            bezier_spline.push_back(this->calcPointOnBezierSpline(counter));
        }

        return bezier_spline;
    }

    void CubicBezierSplines::initVisuHelper()
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

    void CubicBezierSplines::addStartEndPointToVisuHelper()
    {
        this->visu_helper_->addMarkerToExistingMarkerArray(this->start_end_marker_identificator_,
                                                           this->visu_helper_->createGeometryPose(this->start_pose_[0], this->start_pose_[1]),
                                                           this->start_end_marker_identificator_);
        this->visu_helper_->addMarkerToExistingMarkerArray(this->start_end_marker_identificator_,
                                                           this->visu_helper_->createGeometryPose(this->end_pose_[0], this->end_pose_[1]),
                                                           this->start_end_marker_identificator_);                                                        
    }

    void CubicBezierSplines::addControlPointsToVisuHelper()
    {
        this->visu_helper_->addMarkerToExistingMarkerArray(this->control_point_marker_identificator_,
                                                           this->visu_helper_->createGeometryPose(this->cp1_[0], this->cp1_[1]),
                                                           this->control_point_marker_identificator_);
        this->visu_helper_->addMarkerToExistingMarkerArray(this->control_point_marker_identificator_,
                                                           this->visu_helper_->createGeometryPose(this->cp2_[0], this->cp2_[1]),
                                                           this->control_point_marker_identificator_);     
    }

    void CubicBezierSplines::addTangentsToVisuHelper()
    {
        this->addTangentToVisuHelper(this->start_pose_, this->start_pose_tangent_);
        this->addTangentToVisuHelper(this->end_pose_, this->end_pose_tangent_);
    }

    void CubicBezierSplines::addTangentToVisuHelper(Eigen::Matrix<float, 2, 1> start_point, Eigen::Matrix<float, 2, 1> tangent)
    {
        std::vector<geometry_msgs::Point> line;
        Eigen::Matrix<float, 2, 1> end_point;
        end_point = start_point + tangent;

        line.push_back(this->visu_helper_->createGeometryPoint(start_point[0], start_point[1]));
        line.push_back(this->visu_helper_->createGeometryPoint(end_point[0], end_point[1]));

        this->visu_helper_->addMarkerLineToMarkerArray(this->tangent_marker_identificator_,
                                                       line,
                                                       this->tangent_marker_identificator_);
    }

    void CubicBezierSplines::addBezierSplineToVisuHelper()
    {
        std::vector<Eigen::Matrix<float, 2, 1>> bezier_spline;
        std::vector<geometry_msgs::Point> line;

        bezier_spline = this->calcBezierSpline(0.1);

        for(Eigen::Matrix<float, 2, 1> point_on_spline: bezier_spline)
        {
            line.push_back(this->visu_helper_->createGeometryPoint(point_on_spline[0], point_on_spline[1]));
        }

        this->visu_helper_->addMarkerLineToMarkerArray(this->bezier_spline_identificator_,
                                                       line, 
                                                       this->bezier_spline_identificator_);
    }

    void CubicBezierSplines::addDebugVectorToVisuHelper(Eigen::Matrix<float, 2, 1> start_point, Eigen::Matrix<float, 2, 1> vector)
    {
        std::vector<geometry_msgs::Point> line;
        Eigen::Matrix<float, 2, 1> end_point;
        end_point = start_point + vector;

        line.push_back(this->visu_helper_->createGeometryPoint(start_point[0], start_point[1]));
        line.push_back(this->visu_helper_->createGeometryPoint(end_point[0], end_point[1]));
        this->visu_helper_->addMarkerLineToMarkerArray(this->debug_marker_identificator_,
                                                       line,
                                                       this->debug_marker_identificator_);
    }

    // CubicBezierSplines::CubicBezierSplines(Eigen::Matrix<float, 2, 1> start_pose,
    //                                        float start_first_derivative_value,
    //                                        Eigen::Matrix<float, 2, 1> end_pose) : CubicBezierSplines()
    // {
    //     this->start_pose_ = start_pose;
    //     this->start_first_derivative_value_ = start_first_derivative_value;
    //     this->calcFirstSupportPose();
    //     this->end_pose_ = end_pose;
    // }

    // CubicBezierSplines::CubicBezierSplines(std::shared_ptr<CubicBezierSplines> previous_spline, Eigen::Matrix<float, 2, 1> end_pose) :
    //     CubicBezierSplines()
    // {
    //     this->end_pose_ = end_pose;
    //     this->previous_spline_ = previous_spline;
    //     this->previous_spline_->setNextSpline(std::shared_ptr<CubicBezierSplines>(this));
    // }

    // void CubicBezierSplines::calcFirstSupportPose()
    // {
    //     Eigen::Matrix<float, 2, 1> first_derivative_vector;
    //     first_derivative_vector << 1*std::cos(std::atan(this->start_first_derivative_value_)), 1*std::sin(std::atan(this->start_first_derivative_value_)); // This vector is now calculated with a vector of the length 1
    //     this->first_support_pose_ = (1/3) * first_derivative_vector + this->start_pose_;
    // }

    // void CubicBezierSplines::calcSecondSupportPose(Eigen::Matrix<float, 2, 1> end_vector)
    // {
    //     Eigen::Matrix<float, 2, 1> first_derivative_vector;
        

    // }
}