#pragma once

#include "ros/ros.h"
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PolygonStamped.h>

#include <fp_utils/geometry_info/robot_contour.h>

#include <functional>

namespace formation_costmap
{
    class RobotFootprintRos : public geometry_info::RobotContour
    {
        public:
			RobotFootprintRos(ros::NodeHandle &nh,
							  std::string robot_name,
							  std::string robot_namespace,
							  std::string robot_pose_topic_name);

			#pragma region Getter/Setter
			geometry_msgs::PolygonStamped getRobotFootprint();
			#pragma endregion

        private:
			ros::NodeHandle &nh_;

            //! Namespace the robot is located in
			std::string robot_namespace_;
			//! Name of the pose topic where the robot position can be received
			std::string robot_pose_topic_name_;

			//! Subscriber that gets the robot position
			ros::Subscriber robot_pose_sub_;

			void initTopics();

			void getRobotPoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
    };
}