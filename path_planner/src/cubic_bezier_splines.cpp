#include <splines/cubic_bezier_splines.h>

namespace bezier_splines
{
    CubicBezierSplines::CubicBezierSplines()
    {
        
    }

    CubicBezierSplines::CubicBezierSplines(Eigen::Matrix<float, 2, 1> start_pose, Eigen::Matrix<float, 2, 1> end_pose)
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

    void CubicBezierSplines::setEndTangent(tf::Pose next_pose)
    {
        Eigen::Matrix<float, 2, 1> next_point;
        next_point << next_pose.getOrigin().getX(), next_pose.getOrigin().getX();

        Eigen::Matrix<float, 2, 1> diff_vector_start_to_end;
        Eigen::Matrix<float, 2, 1> diff_vector_end_to_next;

        diff_vector_start_to_end = this->end_pose_ - this->start_pose_;
        diff_vector_end_to_next = next_point - this->end_pose_;

        tf::Quaternion first_quaternion;
        tf::Quaternion second_quaternion;
        first_quaternion.setRPY(0, 0, atan2(diff_vector_start_to_end[0], diff_vector_start_to_end[1]));
        second_quaternion.setRPY(0, 0, atan2(diff_vector_end_to_next[0], diff_vector_end_to_next[1]));

        tf::Quaternion second_quaternion_inv = second_quaternion;
        second_quaternion_inv[3] = -second_quaternion_inv[3];
        tf::Quaternion diff_quaternion = first_quaternion * second_quaternion_inv;

    }

    float CubicBezierSplines::calcStartToEndLength()
    {
        return std::sqrt(std::pow((this->end_pose_[0] - this->start_pose_[0]), 2) + std::pow((this->end_pose_[1] - this->start_pose_[1]), 2));
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