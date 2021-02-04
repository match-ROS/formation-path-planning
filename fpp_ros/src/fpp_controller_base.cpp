#include <fpp_ros/fpp_controller_base.h>

namespace fpp
{
	FPPControllerBase::FPPControllerBase(std::vector<std::shared_ptr<fpp_data_classes::RobotInfo>> &robot_info_list,
										 std::shared_ptr<fpp_data_classes::RobotInfo> &robot_info,
										 ros::NodeHandle &nh,
										 ros::NodeHandle &planner_nh)
		: robot_info_list_(robot_info_list),
		  robot_info_(robot_info),
		  nh_(nh),
		  planner_nh_(planner_nh),
		  master_robot_info_(this->getMasterRobotInfo(robot_info_list))
	{

    }

	std::shared_ptr<fpp_data_classes::RobotInfo> FPPControllerBase::getMasterRobotInfo(const std::vector<std::shared_ptr<fpp_data_classes::RobotInfo>> &robot_info_list)
	{
		for(int robot_info_counter = 0; robot_info_counter < robot_info_list.size(); robot_info_counter++)
		{
			if(robot_info_list[robot_info_counter]->fpp_master)
			{
				return robot_info_list[robot_info_counter];
			}
		}
		ROS_ERROR_STREAM("FPPControllerBase::getMasterRobotInfo: No master robot info found. Please check if flag is set in config.");
		return NULL;
	}

    void FPPControllerBase::initialize(std::string planner_name,
                                       costmap_2d::Costmap2D *costmap,
                                       std::string global_frame)
    {
        this->planner_name_ = planner_name;
        this->costmap_ = costmap;
        this->global_frame_ = global_frame;
    }

	void FPPControllerBase::initServices() { }

	void FPPControllerBase::initTopics()
	{
		// Advertise new topic but dont wait for subscribers, as this topic is not init relevant
        this->robot_plan_pub_ = this->nh_.advertise<nav_msgs::Path>("move_base_flex/plan", 10);
	}

	void FPPControllerBase::initTimers() { }

	void FPPControllerBase::publishPlan(const ros::Publisher &plan_publisher, const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        nav_msgs::Path path_to_publish;
        path_to_publish.header.stamp = ros::Time::now();
        path_to_publish.header.frame_id = plan[0].header.frame_id;

        path_to_publish.poses = plan;
		ROS_INFO_STREAM("publish: " << path_to_publish.poses.size() << " " << plan_publisher.getTopic());
        plan_publisher.publish(path_to_publish);
    }

	void FPPControllerBase::calcOffsetPlan(const std::vector<geometry_msgs::PoseStamped> &master_plan,
										   std::vector<geometry_msgs::PoseStamped> &offset_plan,
										   Eigen::Vector2f offset)
	{
		ROS_INFO_STREAM("offset: x:" << offset[0] << " y: " << offset[1]);
		for(geometry_msgs::PoseStamped master_pose: master_plan)
		{
			geometry_msgs::PoseStamped offset_pose = master_pose;
			offset_pose.pose.position.x = offset_pose.pose.position.x + offset[0];
			offset_pose.pose.position.y = offset_pose.pose.position.y + offset[1];
			offset_plan.push_back(offset_pose);
			ROS_INFO_STREAM("master: x:" << master_pose.pose.position.x << " y: " << master_pose.pose.position.y << " offset: x: " << offset_pose.pose.position.x << " offset: y: " << offset_pose.pose.position.y);
		}
	}
}