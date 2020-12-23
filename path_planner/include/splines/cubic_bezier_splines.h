#pragma once

#include "ros/ros.h"

#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <memory> // Usage of smart pointers
#include <vector>
#include <list>
#include <Eigen/Dense>

#include <debug_helper/visualization_helper.h>


namespace bezier_splines
{
    class CubicBezierSplines
    {
        public:
            CubicBezierSplines(visualization_helper::VisualizationHelper *visu_helper);
            CubicBezierSplines(visualization_helper::VisualizationHelper *visu_helper,
                               Eigen::Matrix<float, 2, 1> start_pose,
                               Eigen::Matrix<float, 2, 1> end_pose);

            void setStartTangent(tf::Quaternion robot_orientation);
            void setStartTangent(Eigen::Matrix<float, 2, 1> start_pose_tangent);

            void setEndTangent(tf::Quaternion robot_end_orientation);
            void setEndTangent(Eigen::Matrix<float, 2, 1> next_pose);

            void calcControlPoints();
            void calcBezierSpline();

            void visualizeData();
            void addStartEndPointToVisuHelper();
            void addControlPointsToVisuHelper();
            void addTangentsToVisuHelper();

            Eigen::Matrix<float, 2, 1> getStartPose();
            Eigen::Matrix<float, 2, 1> getEndPose();
            Eigen::Matrix<float, 2, 1> getStartTangent();
            Eigen::Matrix<float, 2, 1> getEndTangent();

            // CubicBezierSplines(Eigen::Matrix<float, 2, 1> start_pose, float start_first_derivative_value, Eigen::Matrix<float, 2, 1> end_pose);

            // /**
            //  * @brief This method links last 
            //  * 
            //  * @param previous_spline 
            //  */
            // CubicBezierSplines(std::shared_ptr<CubicBezierSplines> previous_spline, Eigen::Matrix<float, 2, 1> end_pose);
            
            // void setNextSpline(std::shared_ptr<CubicBezierSplines> next_spline);

            // void calcFirstSupportPose();
            // void calcSecondSupportPose(Eigen::Matrix<float, 2, 1> end_vector);

            // bool isFirstSpline();
            // bool isLastSpline();

        private:
            float calcStartToEndLength();

            

            void initVisuHelper();
            void addTangentToVisuHelper(Eigen::Matrix<float, 2, 1> start_point, Eigen::Matrix<float, 2, 1> tangent);
            void addDebugVectorToVisuHelper(Eigen::Matrix<float, 2, 1> start_point, Eigen::Matrix<float, 2, 1> vector);
            
            visualization_helper::VisualizationHelper* visu_helper_;

            std::string start_end_marker_identificator_;
            std::string control_point_marker_identificator_;
            std::string tangent_marker_identificator_;

            std::string debug_marker_identificator_;

            Eigen::Matrix<float, 2, 1> start_pose_;
            Eigen::Matrix<float, 2, 1> end_pose_;

            Eigen::Matrix<float, 2, 1> cp1_;
            Eigen::Matrix<float, 2, 1> cp2_;

            Eigen::Matrix<float, 2, 1> start_pose_tangent_;
            Eigen::Matrix<float, 2, 1> end_pose_tangent_;
    };
}