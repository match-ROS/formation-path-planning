#include <splines/cubic_bezier_splines.h>

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

        tf::Quaternion first_quaternion;
        tf::Quaternion second_quaternion;
        first_quaternion.setRPY(0, 0, atan2(diff_vector_start_to_end[0], diff_vector_start_to_end[1]));
        second_quaternion.setRPY(0, 0, atan2(diff_vector_end_to_next[0], diff_vector_end_to_next[1]));

        tf::Quaternion second_quaternion_inv = second_quaternion;
        second_quaternion_inv[3] = -second_quaternion_inv[3];
        tf::Quaternion diff_quaternion = first_quaternion * second_quaternion_inv;

        // Currently for debugging
        tf::Matrix3x3 m(diff_quaternion);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        double yaw_angle = ((180/M_PI)*yaw);
        ROS_INFO("Yaw_angle: %f", yaw_angle);
        // end
    }

    void CubicBezierSplines::visualizeData()
    {
        this->visu_helper_->visualizeMarkerArray(this->start_end_marker_identificator_);
        // this->visu_helper_->visualizeMarkerArray(this->control_point_marker_identificator_);
        // this->visu_helper_->visualizeMarkerArray(this->tangent_marker_identificator_);
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

    void CubicBezierSplines::initVisuHelper()
    {
        this->start_end_marker_identificator_ = "start_end";
        this->control_point_marker_identificator_ = "control_points";
        this->tangent_marker_identificator_ = "tangents";

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
            marker_template_control_point.color.g = 1.0;
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
            marker_template_tangents.scale.x = 1.0;
            marker_template_tangents.type = visualization_msgs::Marker::LINE_LIST;
            this->visu_helper_->addMarkerTemplate(this->tangent_marker_identificator_, marker_template_tangents);
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
        // this->visu_helper_->addMarkerToExistingMarkerArray(this->control_point_marker_array_id_,
        // this->visu_helper_->createGeometryPose(this->control))
    }

    void CubicBezierSplines::addTangentsToVisuHelper()
    {
        // ROS_INFO("start_pose: %f | %f", this->start_pose_[0], this->start_pose_[1]);
        // ROS_INFO("start_pose_tangent: %f | %f", this->start_pose_tangent_[0], this->start_pose_tangent_[1]);
        // ROS_INFO("end_pose: %f | %f", this->end_pose_[0], this->end_pose_[1]);
        // ROS_INFO("end_pose_tangent: %f | %f", this->end_pose_tangent_[0], this->end_pose_tangent_[1]);
        this->addTangentToVisuHelper(this->start_pose_, this->start_pose_tangent_);
        this->addTangentToVisuHelper(this->end_pose_, this->end_pose_tangent_);
    }

    void CubicBezierSplines::addTangentToVisuHelper(Eigen::Matrix<float, 2, 1> start_point, Eigen::Matrix<float, 2, 1> tangent)
    {
        this->visu_helper_->addMarkerToExistingMarkerArray(this->tangent_marker_identificator_,
                                                           this->visu_helper_->createGeometryPose(start_point[0], start_point[1]),
                                                           this->tangent_marker_identificator_);
        Eigen::Matrix<float, 2, 1> end_point;
        end_point = start_point + tangent;
        this->visu_helper_->addMarkerToExistingMarkerArray(this->tangent_marker_identificator_,
                                                           this->visu_helper_->createGeometryPose(end_point[0], end_point[1]),
                                                           this->tangent_marker_identificator_);
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