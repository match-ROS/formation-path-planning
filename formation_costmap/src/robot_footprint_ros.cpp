#include <formation_costmap/robot_footprint_ros.h>

namespace formation_costmap
{
	RobotFootprintRos::RobotFootprintRos(ros::NodeHandle &nh,
										 std::string robot_name,
										 std::string robot_namespace,
										 std::string robot_pose_topic_name)
		: RobotContour(robot_name), nh_(nh)
	{
		this->robot_namespace_ = robot_namespace;
		this->robot_pose_topic_name_ = robot_pose_topic_name;

		this->initTopics();
	}

	#pragma region Getter/Setter
	geometry_msgs::PolygonStamped RobotFootprintRos::getRobotFootprint()
	{
		geometry_msgs::PolygonStamped robot_footprint_msg;
        robot_footprint_msg.header.frame_id = "map";
        robot_footprint_msg.header.stamp = ros::Time::now();

        std::vector<Eigen::Vector2f> robot_corner_points = this->getCornerPointsWorldCS();

        for(Eigen::Vector2f corner: robot_corner_points)
        {
            geometry_msgs::Point32 corner_point;
            corner_point.x = corner[0];
            corner_point.y = corner[1];
            corner_point.z = 0.0;
            robot_footprint_msg.polygon.points.push_back(corner_point);
        }

		return robot_footprint_msg;
	}
	#pragma endregion

	void RobotFootprintRos::initTopics()
	{
		this->robot_pose_sub_ = this->nh_.subscribe(
			this->robot_namespace_ + "/" + this->robot_pose_topic_name_,
			10,
			&RobotFootprintRos::getRobotPoseCb,
			this);
	}

	void RobotFootprintRos::getRobotPoseCb(const geometry_msgs::PoseConstPtr &msg)
	{
		Eigen::Vector2f lead_vector_world_cs;
		lead_vector_world_cs_[0] = msg->position.x;
		lead_vector_world_cs_[1] = msg->position.y;
		
		float new_cs_rotation = tf::getYaw(msg->orientation);

		this->moveContour(lead_vector_world_cs_, new_cs_rotation);

		this->robot_pose_changed_handler_(this->robot_name_);
	}
}