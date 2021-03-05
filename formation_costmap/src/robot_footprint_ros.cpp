#include <formation_costmap/robot_footprint_ros.h>

namespace footprint_generation
{
	RobotFootprintRos::RobotFootprintRos(ros::NodeHandle &nh,
									 std::string robot_name,
									 std::string robot_namespace,
									 std::string robot_pose_topic_name)
		: GeometryContour(), nh_(nh)
	{
		this->robot_name_ = robot_name;
		this->robot_namespace_ = robot_namespace;
		this->robot_pose_topic_name_ = robot_pose_topic_name;

		this->initTopics();
	}

	void RobotFootprintRos::setRobotPoseChangedEventHandler(std::function<void(std::string)> robot_pose_changed_handler)
	{
		this->robot_pose_changed_handler_ = robot_pose_changed_handler;
	}

    std::string RobotFootprintRos::getRobotName()
    {
        return this->robot_name_;
    }

	void RobotFootprintRos::initTopics()
	{
		this->robot_pose_sub_ = this->nh_.subscribe(
			this->robot_namespace_ + "/" + this->robot_pose_topic_name_,
			10,
			&RobotFootprintRos::getRobotPoseCb,
			this);
	}

	void RobotFootprintRos::getRobotPoseCb(
		const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
	{
		Eigen::Vector2f lead_vector_world_cs;
		lead_vector_world_cs_[0] = msg->pose.pose.position.x;
		lead_vector_world_cs_[1] = msg->pose.pose.position.y;
		
		float new_cs_rotation = tf::getYaw(msg->pose.pose.orientation);

		this->moveContour(lead_vector_world_cs_, new_cs_rotation);

		this->robot_pose_changed_handler_(this->robot_name_);
	}
}