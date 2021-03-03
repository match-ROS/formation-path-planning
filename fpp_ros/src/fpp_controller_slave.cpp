#include <fpp_ros/fpp_controller_slave.h>

namespace fpp
{
    FPPControllerSlave::FPPControllerSlave(const std::shared_ptr<fpp_data_classes::FPPParamManager> &fpp_params,
                                           ros::NodeHandle &nh,
                                           ros::NodeHandle &planner_nh)
        : FPPControllerBase(fpp_params, nh, planner_nh), master_plan_(nullptr)
    {
		this->initServices();
		this->initTopics();
		this->initTimers();
    }

    void FPPControllerSlave::execute(const geometry_msgs::PoseStamped &start,
                                     const geometry_msgs::PoseStamped &goal,
                                     std::vector<geometry_msgs::PoseStamped> &plan)
    {
		fpp_msgs::GetRobotPlan robot_plan_msg;
		robot_plan_msg.request.robot_name = this->fpp_params_->getCurrentRobotName();
		this->get_robot_plan_srv_client_.call(robot_plan_msg);

		plan = robot_plan_msg.response.robot_plan.poses;

		this->publishPlan(this->robot_plan_pub_, plan);
    }

	void FPPControllerSlave::initTopics()
	{
		FPPControllerBase::initTopics();

		this->master_plan_subscriber_ = this->nh_.subscribe(
			this->fpp_params_->getMasterRobotInfo()->robot_namespace + "/move_base_flex/plan",
			10, &FPPControllerSlave::masterPlanCb, this);
	}

	void FPPControllerSlave::initServices()
	{
		FPPControllerBase::initServices();

		this->get_robot_plan_srv_client_ = this->nh_.serviceClient<fpp_msgs::GetRobotPlan>(
			this->fpp_params_->getMasterRobotInfo()->robot_namespace + "/get_robot_plan");
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