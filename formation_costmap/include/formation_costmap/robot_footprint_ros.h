#pragma once

#include "ros/ros.h"
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <fp_utils/geometry_info/geometry_contour.h>

#include <memory> // Usage of smart pointers
#include <vector>
#include <list>
#include <Eigen/Dense>
#include <functional>

namespace formation_costmap
{
    class RobotFootprintRos : public geometry_info::GeometryContour
    {
        public:
			RobotFootprintRos(ros::NodeHandle &nh,
							std::string robot_name,
							std::string robot_namespace,
							std::string robot_pose_topic_name);

			void setRobotPoseChangedEventHandler(std::function<void(std::string)> robot_pose_changed_handler);

			std::string getRobotName();

        private:
			ros::NodeHandle &nh_;

            //! Robot name for identification
            std::string robot_name_;
			//! Namespace the robot is located in
			std::string robot_namespace_;
			//! Name of the pose topic where the robot position can be received
			std::string robot_pose_topic_name_;

			//! function pointer to the method of the formation to notify the formation about change in the robot pose
			std::function<void(std::string)> robot_pose_changed_handler_;

			//! Subscriber that gets the robot position
			ros::Subscriber robot_pose_sub_;

			void initTopics();

			void getRobotPoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
    };
}