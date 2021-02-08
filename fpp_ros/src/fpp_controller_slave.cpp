#include <fpp_ros/fpp_controller_slave.h>

namespace fpp
{
    FPPControllerSlave::FPPControllerSlave(std::vector<std::shared_ptr<fpp_data_classes::RobotInfo>> &robot_info_list,
                                           std::shared_ptr<fpp_data_classes::RobotInfo> &robot_info,
                                           ros::NodeHandle &nh,
                                           ros::NodeHandle &planner_nh)
        : FPPControllerBase(robot_info_list, robot_info, nh, planner_nh)
    {
        this->master_plan_ = nullptr;

		this->initServices();
		this->initTopics();

		this->initTimers();
    }

    void FPPControllerSlave::execute(const geometry_msgs::PoseStamped &start,
                                     const geometry_msgs::PoseStamped &goal,
                                     std::vector<geometry_msgs::PoseStamped> &plan)
    {
		// ROS_INFO_STREAM("Received new goal " << this->robot_info_->robot_name);

		// // Wait for the plan of the master robot to be published
		// while(this->master_plan_ == nullptr)
		// {
		// 	ros::Duration(0.01).sleep();
		// }

		// this->calcOffsetPlan(this->master_plan_->poses, plan, this->robot_info_->offset);

		fpp_msgs::GetRobotPlan robot_plan_msg;
		robot_plan_msg.request.robot_name = this->robot_info_->robot_name;
		this->get_robot_plan_srv_client_.call(robot_plan_msg);

		plan = robot_plan_msg.response.robot_plan.poses;

		this->publishPlan(this->robot_plan_pub_, plan);
    }

	void FPPControllerSlave::initTopics()
	{
		FPPControllerBase::initTopics();

		this->master_plan_subscriber_ = this->nh_.subscribe(this->master_robot_info_->robot_namespace + "/move_base_flex/plan",
															10, &FPPControllerSlave::masterPlanCb, this);
	}

	void FPPControllerSlave::initServices()
	{
		FPPControllerBase::initServices();

		this->get_robot_plan_srv_client_ = this->nh_.serviceClient<fpp_msgs::GetRobotPlan>(this->master_robot_info_->robot_namespace + "/get_robot_plan");
        this->get_robot_plan_srv_client_.waitForExistence();
	}

	void FPPControllerSlave::formationPlanCb(const nav_msgs::Path::ConstPtr& formation_plan)
	{
		this->formation_plan_ = std::make_shared<nav_msgs::Path>(*formation_plan);
	}

	void FPPControllerSlave::masterPlanCb(const nav_msgs::Path::ConstPtr& master_plan)
	{
		this->master_plan_ = std::make_shared<nav_msgs::Path>(*master_plan);
	}
}